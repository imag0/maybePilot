import pymem
import pymem.process
import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog, messagebox
import ctypes
import json
import math
import numpy as np
import time
import os
import sys
from PIL import Image
import pystray
from pystray import MenuItem as item
import threading
import copy

# Try to enable global hotkeys when app is in background (requires 'keyboard' package)
try:
    import keyboard  # pip install keyboard
    HAS_GLOBAL_HOTKEYS = True
except Exception:
    HAS_GLOBAL_HOTKEYS = False

# Windows API functions
user32 = ctypes.windll.user32
WM_NULL = 0x0000

# Connect to the process (non-fatal on failure)
pm = None
base_address = 0
try:
    pm = pymem.Pymem('smmm_simulation.exe')
    # Get base address of the main module
    base_address = pymem.process.module_from_name(pm.process_handle, 'smmm_simulation.exe').lpBaseOfDll
except Exception as e:
    # Start without connection; user can retry from UI
    try:
        print(f"Failed to connect to process: {e}")
    except Exception:
        pass

# Your offsets from Cheat Engine
Y_OFFSET = 0x1E4810
X_OFFSET = 0x1E4818
# Use absolute heading address provided by user
HEADING_ADDRESS = 0x5E482C

# Calculate actual addresses (based on base_address if available)
Y_ADDRESS = base_address + Y_OFFSET
X_ADDRESS = base_address + X_OFFSET

print(f"Base address: {hex(base_address)}")
print(f"Y address: {hex(Y_ADDRESS)}")
print(f"X address: {hex(X_ADDRESS)}")
print(f"Heading address: {hex(HEADING_ADDRESS)}")

# Waypoint storage
waypoints = []
buoys = []  # Static buoy positions
buoys_visible = True  # Toggle buoy visibility on map
smoothed_waypoints = []
replay_index = 0
replay_active = False
replay_start_time = 0
selected_waypoint_index = None
target_waypoint_index = None # For autopilot

# Autopilot
autopilot_active = False

# PID Controller state for lateral correction
pid_integral = 0.0
pid_last_error = 0.0
pid_last_time = 0.0
autopilot_start_time = 0.0  # Track when autopilot started
autopilot_movement_start_time = 0.0  # Track when ship actually started moving
autopilot_initial_pos = None  # Initial position when autopilot started

# Auto-recording
auto_recording = False
auto_record_start_time = 0
auto_record_waiting = False
auto_record_initial_pos = None

# Heading control
current_heading = 0  # In degrees (0-360)
heading_locked = False
locked_heading = 0

# Position locks
x_locked = False
y_locked = False
locked_x = 0
locked_y = 0

# Speed measurement
speed_measurement_start = {'x': 0, 'y': 0, 'time': 0}
ship_speed = 0.0 # In units per second
# Cache current position for compass bearing rendering
current_pos_x = 0.0
current_pos_y = 0.0

# Calculated heading measurement
calc_heading_start = {'x': 0, 'y': 0, 'time': 0}
calc_heading = 0.0  # In degrees (0-360)

# Smooth heading animation target (for ±2.5° buttons)
heading_anim_target = None  # degrees

# Flag to prevent slider feedback loops
updating_sliders = False
updating_relative_sliders = False

# Button hold tracking
button_hold_active = False
button_hold_action = None
button_hold_delay = 50  # milliseconds between repeats

# Turn inertia state (maintain residual velocity during heading tweaks)
turn_inertia_active = False
turn_inertia_vector = (0.0, 0.0)  # unit vector of last motion
turn_inertia_speed = 0.0          # speed captured at start of turn
turn_inertia_start_time = 0.0
last_update_time = 0.0            # for dt in update loop

# Inertia tuning
TURN_INERTIA_DURATION = 4.0       # seconds the residual drift persists
TURN_INERTIA_STRENGTH = 0.8      # fraction of speed retained initially

def degrees_to_sim_value(degrees):
    """Convert degrees to sim's heading format"""
    lookup = {
        0: 0, 0.3: 0.9641, 0.6: 1.0550, 0.9: 1.1312,
        45: 1.8212, 90: 1.9463, 180: 2.1427,
        270: 2.2945, 359.9: 2.3925, 360: 2.3927
    }
    
    # Normalize to 0-360
    degrees = degrees % 360
    keys = sorted(lookup.keys())
    
    if degrees in lookup:
        return lookup[degrees]
    
    for i in range(len(keys) - 1):
        if keys[i] <= degrees <= keys[i+1]:
            x0, y0 = keys[i], lookup[keys[i]]
            x1, y1 = keys[i+1], lookup[keys[i+1]]
            ratio = (degrees - x0) / (x1 - x0)
            return y0 + ratio * (y1 - y0)
    
    return 0

def sim_value_to_degrees(sim_value):
    """Approximate conversion from sim value to degrees (for display)"""
    # This is a rough inverse - not exact due to non-linear relationship
    lookup = {
        0: 0, 0.9641: 0.3, 1.0550: 0.6, 1.1312: 0.9,
        1.8212: 45, 1.9463: 90, 2.1427: 180,
        2.2945: 270, 2.3925: 359.9, 2.3927: 360
    }
    
    keys = sorted(lookup.keys())
    
    if sim_value in lookup:
        return lookup[sim_value]
    
    for i in range(len(keys) - 1):
        if keys[i] <= sim_value <= keys[i+1]:
            x0, y0 = keys[i], lookup[keys[i]]
            x1, y1 = keys[i+1], lookup[keys[i+1]]
            ratio = (sim_value - x0) / (x1 - x0)
            return y0 + ratio * (y1 - y0)
    
    return 0

last_read_error_log = 0.0

def read_position():
    """Read X, Y, Heading from memory"""
    global last_read_error_log
    try:
        if pm is None:
            # Not connected yet; avoid console spam
            return None, None, None
        x = pm.read_double(X_ADDRESS)
        y = pm.read_double(Y_ADDRESS)
        heading_raw = pm.read_float(HEADING_ADDRESS)
        return x, y, heading_raw
    except Exception as e:
        # Throttle error logs to avoid spam
        now = time.time()
        if now - last_read_error_log > 2.0:
            try:
                print(f"Read position unavailable: {e}")
            except Exception:
                pass
            last_read_error_log = now
        return None, None, None

def write_position(x, y, heading_degrees):
    """Write X, Y, Heading to memory"""
    try:
        pm.write_double(X_ADDRESS, x)
        pm.write_double(Y_ADDRESS, y)
        heading_sim = degrees_to_sim_value(heading_degrees)
        pm.write_float(HEADING_ADDRESS, heading_sim)
        return True
    except Exception as e:
        print(f"Write error: {e}")
        return False

def add_waypoint():
    """Add current position as waypoint with timing"""
    x, y, heading_raw = read_position()
    if x is not None:
        # Calculate time from last waypoint (default 1 second)
        if waypoints:
            time_from_start = waypoints[-1]['time'] + 1.0
        else:
            time_from_start = 0.0
        
        waypoints.append({
            'x': x, 
            'y': y, 
            'heading_raw': heading_raw,
            'time': time_from_start
        })
        update_waypoint_list()
        status_label.config(text=f"Added waypoint {len(waypoints)}: X={x:.2f}, Y={y:.2f} at t={time_from_start:.1f}s")

def toggle_auto_record():
    """Toggle automatic waypoint recording"""
    global auto_recording, auto_record_start_time, auto_record_waiting, auto_record_initial_pos
    
    auto_recording = auto_record_var.get()
    
    if auto_recording:
        # Clear existing waypoints and start fresh
        waypoints.clear()
        auto_record_waiting = True
        x, y, _ = read_position()
        if x is not None:
            auto_record_initial_pos = (x, y)
        auto_record_btn.config(text="⏺ Recording...")
        status_label.config(text="Waiting for movement in sim...")
        auto_record_step()
    else:
        auto_record_waiting = False
        auto_record_initial_pos = None
        auto_record_btn.config(text="⏺ Auto Record")
        status_label.config(text=f"Auto-recording stopped - captured {len(waypoints)} waypoints")

def auto_record_step():
    """Capture waypoint during auto-recording"""
    global auto_recording, auto_record_waiting, auto_record_start_time, auto_record_initial_pos
    
    if not auto_recording:
        return
    
    x, y, heading_raw = read_position()
    if x is not None:
        # Check if we're waiting for movement
        if auto_record_waiting:
            if auto_record_initial_pos is not None:
                initial_x, initial_y = auto_record_initial_pos
                distance = math.sqrt((x - initial_x)**2 + (y - initial_y)**2)
                
                # Start recording if moved more than 0.5 units
                if distance > 0.5:
                    auto_record_waiting = False
                    auto_record_start_time = time.time()
                    status_label.config(text="Movement detected! Recording started...")
                else:
                    status_label.config(text="Waiting for movement in sim...")
        
        # Record waypoint if not waiting
        if not auto_record_waiting:
            elapsed_time = time.time() - auto_record_start_time
            
            waypoints.append({
                'x': x,
                'y': y,
                'heading_raw': heading_raw,
                'time': elapsed_time
            })
            
            update_waypoint_list()
            status_label.config(text=f"Recording... {len(waypoints)} waypoints at t={elapsed_time:.1f}s")
    
    # Schedule next recording in 1 second
    if auto_recording:
        root.after(1000, auto_record_step)

def add_manual_waypoint():
    """Add waypoint from manual input"""
    try:
        x = float(x_entry.get())
        y = float(y_entry.get())
        heading = float(heading_entry.get())
        time_val = float(time_entry.get())
        
        heading_sim = degrees_to_sim_value(heading)
        waypoints.append({
            'x': x, 
            'y': y, 
            'heading_raw': heading_sim,
            'time': time_val
        })
        
        # Sort waypoints by time
        waypoints.sort(key=lambda w: w['time'])
        
        update_waypoint_list()
        status_label.config(text=f"Added manual waypoint: X={x:.2f}, Y={y:.2f}, H={heading:.1f}° at t={time_val:.1f}s")
    except ValueError:
        status_label.config(text="Error: Invalid input values")

def clear_waypoints():
    """Clear all waypoints"""
    global waypoints, smoothed_waypoints, selected_waypoint_index
    waypoints = []
    smoothed_waypoints = []
    selected_waypoint_index = None
    update_waypoint_list()
    status_label.config(text="Cleared all waypoints")

def select_waypoint():
    """Selects a waypoint from the list."""
    global selected_waypoint_index
    
    selected_items = waypoint_tree.selection()
    if not selected_items:
        status_label.config(text="No waypoint selected in the list.")
        return
        
    selected_iid = selected_items[0]
    selected_waypoint_index = waypoint_tree.index(selected_iid)
    update_waypoint_list() # To refresh highlighting
    status_label.config(text=f"Selected waypoint {selected_waypoint_index + 1}")

def delete_selected_waypoint():
    """Delete the currently selected waypoint"""
    global selected_waypoint_index
    
    if selected_waypoint_index is None:
        status_label.config(text="No waypoint selected. Click one in the list.")
        return
    
    if 0 <= selected_waypoint_index < len(waypoints):
        wp = waypoints[selected_waypoint_index]
        if messagebox.askyesno("Delete Waypoint", f"Are you sure you want to delete waypoint {selected_waypoint_index + 1}?"):
            waypoints.pop(selected_waypoint_index)
            selected_waypoint_index = None
            update_waypoint_list()
            status_label.config(text=f"Deleted waypoint at X={wp['x']:.2f}, Y={wp['y']:.2f}")
    else:
        selected_waypoint_index = None
        status_label.config(text="Invalid waypoint selection")

def edit_selected_waypoint():
    """Edit the currently selected waypoint"""
    global selected_waypoint_index
    
    if selected_waypoint_index is None:
        status_label.config(text="No waypoint selected. Click one in the list.")
        return
    
    if not (0 <= selected_waypoint_index < len(waypoints)):
        selected_waypoint_index = None
        status_label.config(text="Invalid waypoint selection")
        return
    
    wp = waypoints[selected_waypoint_index]
    
    # Create edit dialog
    edit_window = tk.Toplevel(root)
    edit_window.title(f"Edit Waypoint {selected_waypoint_index + 1}")
    edit_window.geometry("300x200")
    edit_window.attributes('-topmost', True)
    
    ttk.Label(edit_window, text=f"Editing Waypoint {selected_waypoint_index + 1}", font=("Arial", 10, "bold")).pack(pady=5)
    
    # Time entry
    time_frame = ttk.Frame(edit_window)
    time_frame.pack(pady=5)
    ttk.Label(time_frame, text="Time (s):").pack(side=tk.LEFT, padx=5)
    time_edit = ttk.Entry(time_frame, width=10)
    time_edit.pack(side=tk.LEFT)
    time_edit.insert(0, f"{wp['time']:.2f}")
    
    # X entry
    x_frame = ttk.Frame(edit_window)
    x_frame.pack(pady=5)
    ttk.Label(x_frame, text="X:").pack(side=tk.LEFT, padx=5)
    x_edit = ttk.Entry(x_frame, width=10)
    x_edit.pack(side=tk.LEFT)
    x_edit.insert(0, f"{wp['x']:.2f}")
    
    # Y entry
    y_frame = ttk.Frame(edit_window)
    y_frame.pack(pady=5)
    ttk.Label(y_frame, text="Y:").pack(side=tk.LEFT, padx=5)
    y_edit = ttk.Entry(y_frame, width=10)
    y_edit.pack(side=tk.LEFT)
    y_edit.insert(0, f"{wp['y']:.2f}")
    
    # Heading entry (in degrees)
    heading_frame = ttk.Frame(edit_window)
    heading_frame.pack(pady=5)
    ttk.Label(heading_frame, text="Heading (°):").pack(side=tk.LEFT, padx=5)
    heading_edit = ttk.Entry(heading_frame, width=10)
    heading_edit.pack(side=tk.LEFT)
    current_heading_deg = sim_value_to_degrees(wp['heading_raw'])
    heading_edit.insert(0, f"{current_heading_deg:.1f}")
    
    def save_changes():
        try:
            new_time = float(time_edit.get())
            new_x = float(x_edit.get())
            new_y = float(y_edit.get())
            new_heading_deg = float(heading_edit.get())
            
            waypoints[selected_waypoint_index] = {
                'time': new_time,
                'x': new_x,
                'y': new_y,
                'heading_raw': degrees_to_sim_value(new_heading_deg)
            }
            
            # Re-sort waypoints by time
            waypoints.sort(key=lambda w: w['time'])
            update_waypoint_list()
            status_label.config(text=f"Updated waypoint {selected_waypoint_index + 1}")
            edit_window.destroy()
        except ValueError:
            status_label.config(text="Error: Invalid input values")
    
    # Buttons
    btn_frame = ttk.Frame(edit_window)
    btn_frame.pack(pady=10)
    ttk.Button(btn_frame, text="Save", command=save_changes).pack(side=tk.LEFT, padx=5)
    ttk.Button(btn_frame, text="Cancel", command=edit_window.destroy).pack(side=tk.LEFT, padx=5)

def jump_to_selected_waypoint():
    """Jump ship to the selected waypoint position"""
    global selected_waypoint_index
    
    if selected_waypoint_index is None:
        status_label.config(text="No waypoint selected")
        return
    
    if 0 <= selected_waypoint_index < len(waypoints):
        wp = waypoints[selected_waypoint_index]
        try:
            if not x_locked:
                pm.write_double(X_ADDRESS, float(wp['x']))
            if not y_locked:
                pm.write_double(Y_ADDRESS, float(wp['y']))
            # Write heading if available
            if 'heading_raw' in wp:
                try:
                    pm.write_float(HEADING_ADDRESS, float(wp['heading_raw']))
                except Exception:
                    pass
            status_label.config(text=f"Jumped to WP {selected_waypoint_index + 1}")
        except Exception as e:
            print(f"Jump error: {e}")
            status_label.config(text=f"Jump error: {e}")
    else:
        selected_waypoint_index = None
        status_label.config(text="Invalid waypoint selection")

# Teleport to the current autopilot target waypoint (instant move)
def teleport_to_autopilot_target():
    try:
        if autopilot_active and target_waypoint_index is not None and 0 <= target_waypoint_index < len(waypoints):
            wp = waypoints[target_waypoint_index]
            # Respect position locks
            if not x_locked:
                pm.write_double(X_ADDRESS, float(wp['x']))
            if not y_locked:
                pm.write_double(Y_ADDRESS, float(wp['y']))
            # Align heading to waypoint if available
            if 'heading_raw' in wp:
                try:
                    heading_deg = sim_value_to_degrees(float(wp['heading_raw']))
                    set_heading(heading_deg)
                except Exception:
                    pass
            status_label.config(text=f"Teleported to WP {target_waypoint_index + 1} and aligned heading")
        else:
            status_label.config(text="No autopilot target to teleport to")
    except Exception as e:
        print(f"Teleport error: {e}")
        status_label.config(text=f"Teleport error: {e}")

def find_best_next_waypoint(current_x, current_y, current_heading_deg):
    """Finds the best waypoint to target, prioritizing those in front of the ship."""
    if not waypoints:
        return None

    best_waypoint_idx = None
    best_score = -float('inf')

    # Ship's heading vector
    heading_rad = math.radians(current_heading_deg)
    # In our coordinate system, 0 deg is North (+Y), 90 deg is East (+X)
    # So, a heading of 0 deg should be vector (0, 1)
    # A heading of 90 deg should be vector (1, 0)
    # This corresponds to (sin(rad), cos(rad))
    ship_heading_vec = (math.sin(heading_rad), math.cos(heading_rad))

    for i, wp in enumerate(waypoints):
        # Vector from ship to waypoint
        wp_vec = (wp['x'] - current_x, wp['y'] - current_y)
        
        distance = math.sqrt(wp_vec[0]**2 + wp_vec[1]**2)
        if distance < 1.0:  # Too close, probably the current location
            continue

        # Normalize the waypoint vector
        norm_wp_vec = (wp_vec[0] / distance, wp_vec[1] / distance)

        # Dot product to find how "in front" the waypoint is.
        # A value of 1 is directly in front, 0 is 90 degrees, -1 is behind.
        dot_product = ship_heading_vec[0] * norm_wp_vec[0] + ship_heading_vec[1] * norm_wp_vec[1]

        # We want to prioritize waypoints in front.
        # A simple score: prioritize being in front, then by inverse distance.
        # The dot_product acts as a weight. Negative values (behind) are heavily penalized.
        score = dot_product / (distance * 0.1) # Scale distance to not overwhelm the dot product

        if score > best_score:
            best_score = score
            best_waypoint_idx = i
            
    return best_waypoint_idx

def toggle_autopilot():
    """Toggle the waypoint attraction autopilot."""
    global autopilot_active, target_waypoint_index, pid_integral, pid_last_error, pid_last_time
    global autopilot_start_time, autopilot_movement_start_time, autopilot_initial_pos
    
    autopilot_active = not autopilot_active
    
    if autopilot_active:
        if not waypoints:
            status_label.config(text="No waypoints to follow.")
            autopilot_active = False
            autopilot_btn.config(text="▶️ Autopilot")
            return
            
        # Reset PID controller state
        pid_integral = 0.0
        pid_last_error = 0.0
        pid_last_time = time.time()
        autopilot_start_time = time.time()  # Track startup time
        autopilot_movement_start_time = 0.0  # Reset movement timer
        
        # Store initial position to detect movement
        x, y, _ = read_position()
        if x is None:
            status_label.config(text="Cannot read ship position to start autopilot.")
            autopilot_active = False
            autopilot_btn.config(text="▶️ Autopilot")
            return
        
        autopilot_initial_pos = (x, y)
            
        target_waypoint_index = find_best_next_waypoint(x, y, current_heading)

        if target_waypoint_index is None:
            status_label.config(text="No suitable forward waypoint found.")
            autopilot_active = False
            autopilot_btn.config(text="▶️ Autopilot")
            return

        autopilot_btn.config(text="⏹️ Autopilot ON")
        status_label.config(text=f"Autopilot enabled. Target: WP {target_waypoint_index + 1}")
        autopilot_step()
    else:
        autopilot_btn.config(text="▶️ Autopilot")
        status_label.config(text="Autopilot disabled.")
        update_waypoint_list() # Clear target highlight

def autopilot_step():
    """Uses ship's velocity vector to gently guide it towards the target waypoint."""
    global autopilot_active, target_waypoint_index, pid_integral, pid_last_error, pid_last_time
    global autopilot_movement_start_time, autopilot_initial_pos
    
    if not autopilot_active or target_waypoint_index is None:
        return

    if not (0 <= target_waypoint_index < len(waypoints)):
        status_label.config(text="Autopilot finished: Reached end of path.")
        toggle_autopilot()
        return

    # Get current state
    x, y, heading_raw = read_position()
    if x is None:
        root.after(200, autopilot_step) # Retry if read fails
        return
    
    # --- Movement Detection Logic ---
    # Check if ship has started moving (distance > 0.5 units from initial position)
    if autopilot_movement_start_time == 0.0 and autopilot_initial_pos is not None:
        initial_x, initial_y = autopilot_initial_pos
        distance_moved = math.sqrt((x - initial_x)**2 + (y - initial_y)**2)
        
        if distance_moved > 0.5:
            # Ship has started moving - start the movement timer
            autopilot_movement_start_time = time.time()
            status_label.config(text=f"Movement detected! Autopilot tracking started.")
        # If not moving yet, keep using memory heading
        
    # Get target waypoint
    target_wp = waypoints[target_waypoint_index]
    
    # --- Waypoint Switching Logic ---
    distance_to_target = math.sqrt((target_wp['x'] - x)**2 + (target_wp['y'] - y)**2)
    
    # Radius to switch to the next waypoint:
    # - 25 unit buffer around waypoint
    # - 10 unit buffer around ship
    # Ship passes waypoint when distance < (waypoint_buffer + ship_buffer)
    waypoint_buffer = 25.0
    ship_buffer = 10.0
    arrival_radius = waypoint_buffer + ship_buffer  # 35 units total
    if distance_to_target < arrival_radius:
        status_label.config(text=f"Approaching WP {target_waypoint_index + 1}. Targeting next.")
        target_waypoint_index = (target_waypoint_index + 1) % len(waypoints)
        update_waypoint_list()
        root.after(100, autopilot_step)
        return

    # Target the current waypoint directly (no look-ahead)
    aim_x = target_wp['x']
    aim_y = target_wp['y']

    # Turn anticipation: gently blend aim toward the next leg before reaching the waypoint
    if len(waypoints) > 1:
        next_index = (target_waypoint_index + 1) % len(waypoints)
        if next_index != target_waypoint_index:
            next_wp = waypoints[next_index]
            anticipation_radius = 80.0 + ship_speed * 20.0  # dynamic based on speed
            if distance_to_target < anticipation_radius:
                blend = 1.0 - (distance_to_target / anticipation_radius)
                blend = max(0.0, min(0.6, blend))  # cap blending for stability
                aim_x = target_wp['x'] * (1 - blend) + next_wp['x'] * blend
                aim_y = target_wp['y'] * (1 - blend) + next_wp['y'] * blend

    # --- Vector-Based Guidance ---
    # Use heading from memory until ship has been moving for 2 seconds
    # This prevents spinning out due to delays in COG calculation
    time_since_movement = 0.0
    if autopilot_movement_start_time > 0:
        time_since_movement = time.time() - autopilot_movement_start_time
    
    # Use memory heading if:
    # - Ship hasn't started moving yet (movement_start_time == 0)
    # - Ship has been moving for less than 2.0 seconds
    # - Ship is moving very slowly (COG unreliable)
    use_memory_heading = (autopilot_movement_start_time == 0.0) or (time_since_movement < 2.0) or (ship_speed < 1.0)
    
    if use_memory_heading:
        # Use heading from memory (bow direction)
        heading_deg = sim_value_to_degrees(heading_raw)
    else:
        # Use calc_heading (actual movement direction / COG)
        heading_deg = calc_heading
    
    heading_rad = math.radians(heading_deg)
    
    # Ship's velocity vector (using heading convention: 0° = North)
    velocity_x = ship_speed * math.sin(heading_rad)
    velocity_y = ship_speed * math.cos(heading_rad)
    
    # Direction vector to aim point
    to_target_x = aim_x - x
    to_target_y = aim_y - y
    aim_distance = math.sqrt(to_target_x**2 + to_target_y**2)
    
    # Normalize the direction to target
    if aim_distance > 0:
        to_target_x /= aim_distance
        to_target_y /= aim_distance
    
    # Calculate bearing to aim point
    bearing_to_target = math.degrees(math.atan2(to_target_x, to_target_y)) % 360
    
    # Calculate heading error
    heading_error = bearing_to_target - heading_deg
    if heading_error > 180:
        heading_error -= 360
    elif heading_error < -180:
        heading_error += 360

    abs_heading_error = abs(heading_error)

    # Assistance-only: position attraction toward aim point (no heading writes)
    max_distance = 200.0
    distance_factor = min(1.0, distance_to_target / max_distance)
    assist_gain = 0.05  # gentler pull for assistance
    position_strength = assist_gain * distance_factor

    correction_x = to_target_x * position_strength
    correction_y = to_target_y * position_strength
    
    # --- Full Waypoint Attraction (Position + Heading) ---
    # Simple distance-based position correction
    max_distance = 200.0  # Distance at which correction reaches maximum
    distance_factor = min(1.0, distance_to_target / max_distance)
    
    # Position pull strength
    position_strength = 0.08 * distance_factor
    
    # Apply direct position correction toward waypoint
    correction_x = to_target_x * position_strength
    correction_y = to_target_y * position_strength
    
    # Apply the correction to the ship's position and heading
    nudged_x = x + correction_x
    nudged_y = y + correction_y

    # Write the gently nudged position and heading back to memory, but only if not locked
    try:
        if not x_locked:
            pm.write_double(X_ADDRESS, nudged_x)
        if not y_locked:
            pm.write_double(Y_ADDRESS, nudged_y)

        # Status shows current course vs goal bearing (assistance only)
        status_label.config(text=f"AP Assist: WP {target_waypoint_index + 1} | Dist: {distance_to_target:.0f} | CRS {heading_deg:.0f}° → BRG {bearing_to_target:.0f}° | Δ{heading_error:.0f}°")
    except Exception as e:
        print(f"Autopilot write error: {e}")
        status_label.config(text="Autopilot write error.")

    # Schedule the next guidance step
    root.after(100, autopilot_step)

def move_waypoint(direction):
    """Move the selected waypoint up or down in the list."""
    global selected_waypoint_index
    
    if selected_waypoint_index is None:
        status_label.config(text="No waypoint selected to move.")
        return

    if not (0 <= selected_waypoint_index < len(waypoints)):
        return

    new_index = selected_waypoint_index + direction
    
    if not (0 <= new_index < len(waypoints)):
        return # Can't move past the start or end

    # Swap waypoints
    waypoints.insert(new_index, waypoints.pop(selected_waypoint_index))
    
    # Re-calculate time based on new order, preserving duration between points
    for i in range(1, len(waypoints)):
        # Assume 1 second between points for simplicity after reordering
        waypoints[i]['time'] = waypoints[i-1]['time'] + 1.0

    selected_waypoint_index = new_index
    update_waypoint_list()
    globals()['map_static_dirty'] = True
    draw_map()
    status_label.config(text=f"Moved waypoint to position {new_index + 1}")

def update_waypoint_list():
    """Update the waypoint display in the Treeview"""
    # Clear existing items
    for item in waypoint_tree.get_children():
        waypoint_tree.delete(item)
    
    # Add new items
    for i, wp in enumerate(waypoints):
        heading_deg = sim_value_to_degrees(wp['heading_raw'])
        
        # Define tags for styling
        tags = ()
        if i == selected_waypoint_index:
            tags = ('selected',)
        if i == target_waypoint_index and autopilot_active:
            tags = ('target',)
            
        waypoint_tree.insert('', tk.END, iid=i, values=(
            f"{i+1}",
            f"{wp['time']:.1f}",
            f"{wp['x']:.2f}",
            f"{wp['y']:.2f}",
            f"{heading_deg:.1f}°"
        ), tags=tags)

    # Configure tag styles
    waypoint_tree.tag_configure('selected', background='lightblue')
    waypoint_tree.tag_configure('target', background='lightgreen', foreground='black')
    
    # Update frame label to show counts
    if buoys:
        waypoint_frame.config(text=f"Waypoints ({len(waypoints)}) | Buoys ({len(buoys)})")
    else:
        waypoint_frame.config(text=f"Waypoints ({len(waypoints)})")

def on_waypoint_select(event):
    """Handle waypoint selection in the Treeview."""
    global selected_waypoint_index
    selected_items = waypoint_tree.selection()
    if selected_items:
        selected_iid = selected_items[0]
        selected_waypoint_index = waypoint_tree.index(selected_iid)
        # No need to call update_waypoint_list here to avoid loops, selection is visual
        wp = waypoints[selected_waypoint_index]
        status_label.config(text=f"Selected WP {selected_waypoint_index+1}: X={wp['x']:.2f}, Y={wp['y']:.2f}")
        update_waypoint_list() # Refresh to show highlight

def smooth_waypoints():
    """Generate smoothed path between waypoints using cubic spline interpolation"""
    global smoothed_waypoints
    
    if len(waypoints) < 2:
        status_label.config(text="Need at least 2 waypoints to smooth")
        return
    
    # Extract coordinates and times
    x_points = [wp['x'] for wp in waypoints]
    y_points = [wp['y'] for wp in waypoints]
    heading_points = [wp['heading_raw'] for wp in waypoints]
    time_points = [wp['time'] for wp in waypoints]
    
    # Total duration
    total_duration = time_points[-1] - time_points[0]
    
    # Number of interpolated points based on duration (1 point per second at 1 FPS)
    num_points = int(total_duration) + 1
    
    # Create time array for interpolation
    t_smooth = np.linspace(time_points[0], time_points[-1], num_points)
    
    # Linear interpolation (numpy built-in, no scipy needed)
    x_smooth = np.interp(t_smooth, time_points, x_points)
    y_smooth = np.interp(t_smooth, time_points, y_points)
    heading_smooth = np.interp(t_smooth, time_points, heading_points)
    
    smoothed_waypoints = [
        {
            'x': x_smooth[i], 
            'y': y_smooth[i], 
            'heading_raw': heading_smooth[i],
            'time': t_smooth[i]
        }
        for i in range(num_points)
    ]
    
    status_label.config(text=f"Generated {num_points} smoothed waypoints")
    smooth_text.config(state='normal')
    smooth_text.delete('1.0', tk.END)
    smooth_text.insert(tk.END, f"Smoothed path: {num_points} points\n")
    smooth_text.insert(tk.END, f"Total duration: {total_duration:.1f} seconds\n")
    smooth_text.config(state='disabled')

def start_replay():
    """Start replaying smoothed waypoints"""
    global replay_active, replay_index, replay_start_time
    
    if not smoothed_waypoints:
        status_label.config(text="No smoothed waypoints. Generate smooth path first.")
        return
    
    replay_active = True
    replay_index = 0
    replay_start_time = time.time()
    replay_btn.config(state='disabled')
    stop_btn.config(state='normal')
    status_label.config(text="Replay started")
    replay_step()

def stop_replay():
    """Stop replay"""
    global replay_active
    replay_active = False
    replay_btn.config(state='normal')
    stop_btn.config(state='disabled')
    status_label.config(text="Replay stopped")

def replay_step():
    """Execute replay based on timing"""
    global replay_index, replay_active
    
    if not replay_active:
        return
    
    try:
        current_time = time.time() - replay_start_time
        
        # Find the appropriate waypoint based on current time
        while replay_index < len(smoothed_waypoints):
            wp = smoothed_waypoints[replay_index]
            
            # Check if it's time to execute this waypoint
            if wp['time'] <= current_time:
                # Write position to memory
                try:
                    pm.write_double(X_ADDRESS, wp['x'])
                    pm.write_double(Y_ADDRESS, wp['y'])
                    pm.write_float(HEADING_ADDRESS, wp['heading_raw'])
                except Exception as e:
                    print(f"Error writing position during replay: {e}")
                    stop_replay()
                    status_label.config(text=f"Replay error: {e}")
                    return
                
                progress = (replay_index + 1) / len(smoothed_waypoints) * 100
                elapsed = current_time
                total_time = smoothed_waypoints[-1]['time']
                status_label.config(text=f"Replay: {elapsed:.1f}/{total_time:.1f}s ({progress:.1f}%)")
                
                replay_index += 1
            else:
                # Not time yet, wait for next check
                break
        
        # Check if replay is complete
        if replay_index >= len(smoothed_waypoints):
            stop_replay()
            status_label.config(text="Replay complete!")
            return
        
        # Schedule next check (every 100ms for smoother timing)
        if replay_active:
            root.after(100, replay_step)
    except Exception as e:
        print(f"Error in replay_step: {e}")
        stop_replay()
        status_label.config(text=f"Replay error: {e}")

def save_waypoints():
    """Save waypoints and buoys to file with custom name"""
    if not waypoints and not buoys:
        status_label.config(text="No waypoints or buoys to save")
        return
    
    # Ask for filename
    filename = filedialog.asksaveasfilename(
        defaultextension=".json",
        filetypes=[("JSON files", "*.json"), ("Text files", "*.txt"), ("All files", "*.*")],
        title="Save Waypoints As"
    )
    
    if not filename:  # User cancelled
        return
    
    try:
        # Save in new format with waypoints and buoys sections
        save_data = {
            'waypoints': waypoints
        }
        if buoys:
            save_data['buoys'] = buoys
        
        with open(filename, 'w') as f:
            json.dump(save_data, f, indent=2)
        
        if buoys:
            status_label.config(text=f"Saved {len(waypoints)} waypoints and {len(buoys)} buoys to {filename}")
        else:
            status_label.config(text=f"Saved {len(waypoints)} waypoints to {filename}")
    except Exception as e:
        status_label.config(text=f"Save error: {e}")

def load_waypoints():
    """Load waypoints and buoys from file"""
    global waypoints, buoys
    
    # Ask for file to open
    filename = filedialog.askopenfilename(
        filetypes=[("JSON files", "*.json"), ("Text files", "*.txt"), ("All files", "*.*")],
        title="Load"
    )
    
    if not filename:  # User cancelled
        return
    
    try:
        with open(filename, 'r') as f:
            loaded_data = json.load(f)
        
        # Check if it's new format with waypoints and buoys sections
        if isinstance(loaded_data, dict) and 'waypoints' in loaded_data:
            # New format with separate waypoints and buoys
            waypoints.clear()
            for i, wp_data in enumerate(loaded_data['waypoints']):
                wp = {
                    'x': wp_data.get('x', 0.0),
                    'y': wp_data.get('y', 0.0),
                    'heading_raw': wp_data.get('heading_raw', 0.0),
                    'time': wp_data.get('time', float(i))  # Assign index as time if missing
                }
                waypoints.append(wp)
            
            # Load buoys if present
            buoys.clear()
            if 'buoys' in loaded_data:
                for buoy_data in loaded_data['buoys']:
                    buoy = {
                        'x': buoy_data.get('x', 0.0),
                        'y': buoy_data.get('y', 0.0)
                    }
                    buoys.append(buoy)
                status_msg = f"Loaded {len(waypoints)} waypoints and {len(buoys)} buoys from {filename}"
            else:
                status_msg = f"Loaded {len(waypoints)} waypoints from {filename}"
        else:
            # Old format - just a list of waypoints
            waypoints.clear()
            for i, wp_data in enumerate(loaded_data):
                wp = {
                    'x': wp_data.get('x', 0.0),
                    'y': wp_data.get('y', 0.0),
                    'heading_raw': wp_data.get('heading_raw', 0.0),
                    'time': wp_data.get('time', float(i))  # Assign index as time if missing
                }
                waypoints.append(wp)
            status_msg = f"Loaded {len(waypoints)} waypoints from {filename}"

        waypoints.sort(key=lambda w: w['time'])
        update_waypoint_list()
        globals()['map_static_dirty'] = True
        draw_map()
        status_label.config(text=status_msg)
    except FileNotFoundError:
        status_label.config(text="File not found")
    except Exception as e:
        status_label.config(text=f"Load error: {e}")

def update_display():
    """Update display with current position from memory and sync sliders"""
    global updating_sliders, current_heading, speed_measurement_start, ship_speed
    global calc_heading_start, calc_heading
    global current_pos_x, current_pos_y
    global heading_anim_target
    global last_update_time, turn_inertia_active, turn_inertia_vector, turn_inertia_speed, turn_inertia_start_time
    
    x, y, heading_raw = read_position()
    if x is not None:
        try:
            # Cache latest position for compass bearing rendering
            current_pos_x, current_pos_y = x, y
            
            # --- Speed Calculation (2-second average) ---
            current_time = time.time()
            
            # Initialize dt for inertia/animation
            if last_update_time == 0.0:
                last_update_time = current_time
            dt = max(0.01, min(0.25, current_time - last_update_time))
            last_update_time = current_time
            
            # Initialize measurement start if needed
            if speed_measurement_start['time'] == 0:
                speed_measurement_start = {'x': x, 'y': y, 'time': current_time}
                calc_heading_start = {'x': x, 'y': y, 'time': current_time}
            
            time_delta = current_time - speed_measurement_start['time']
            
            # Calculate speed over 2-second period
            if time_delta >= 2.0:
                distance_delta = math.sqrt((x - speed_measurement_start['x'])**2 + (y - speed_measurement_start['y'])**2)
                ship_speed = distance_delta / time_delta
                
                # Reset the measurement start to current position
                speed_measurement_start = {'x': x, 'y': y, 'time': current_time}
            
            # Calculate heading from movement direction (1-second average)
            calc_time_delta = current_time - calc_heading_start['time']
            if calc_time_delta >= 1.0:
                dx = x - calc_heading_start['x']
                dy = y - calc_heading_start['y']
                
                # Calculate heading from movement vector if ship is moving
                if math.sqrt(dx**2 + dy**2) > 0.5:
                    calc_heading = (math.degrees(math.atan2(dx, dy)) + 360) % 360
                
                # Reset calc heading measurement
                calc_heading_start = {'x': x, 'y': y, 'time': current_time}

            # --- Apply position and heading locks (write locked values back to memory) ---
            # This ensures locks are enforced even when the game/simulation changes values
            try:
                if x_locked:
                    pm.write_double(X_ADDRESS, locked_x)
                    x = locked_x  # keep local state in sync
                if y_locked:
                    pm.write_double(Y_ADDRESS, locked_y)
                    y = locked_y
                if heading_locked:
                    pm.write_float(HEADING_ADDRESS, degrees_to_sim_value(locked_heading))
                    heading_raw = degrees_to_sim_value(locked_heading)
            except Exception as e:
                print(f"Lock write error: {e}")

            # --- Apply short residual drift during heading tweaks (turn inertia) ---
            # Only when not under autopilot and positions aren't locked
            if turn_inertia_active and not autopilot_active and (not x_locked or not y_locked):
                t_elapsed = current_time - turn_inertia_start_time
                if t_elapsed <= TURN_INERTIA_DURATION:
                    decay = 1.0 - (t_elapsed / TURN_INERTIA_DURATION)  # linear decay
                    residual_speed = turn_inertia_speed * TURN_INERTIA_STRENGTH * decay
                    dx_drift = turn_inertia_vector[0] * residual_speed * dt
                    dy_drift = turn_inertia_vector[1] * residual_speed * dt

                    nudged_x = x + dx_drift
                    nudged_y = y + dy_drift

                    try:
                        if not x_locked:
                            pm.write_double(X_ADDRESS, nudged_x)
                            x = nudged_x  # keep local state in sync
                        if not y_locked:
                            pm.write_double(Y_ADDRESS, nudged_y)
                            y = nudged_y
                    except Exception as e:
                        print(f"Inertia write error: {e}")
                else:
                    turn_inertia_active = False

            # --- UI Updates ---
            read_label.config(text=f"Current: X={x:.2f}, Y={y:.2f}, H_raw={heading_raw:.4f}")
            speed_label.config(text=f"Speed: {ship_speed:.2f} u/s | COG: {calc_heading:.1f}°")
            
            # Update sliders to match current position (prevent feedback loop)
            updating_sliders = True
            x_slider.set(x)
            y_slider.set(y)
            x_value_label.config(text=f"{x:.1f}")
            y_value_label.config(text=f"{y:.1f}")
            updating_sliders = False
            
            # Update heading display (read from memory)
            current_heading = sim_value_to_degrees(heading_raw)
            heading_value_label.config(text=f"{current_heading:.1f}°")
            
            # Smoothly animate heading toward target if set (assistance for ±2.5° buttons)
            if not heading_locked and heading_anim_target is not None:
                # Shortest signed angular difference [-180, 180]
                diff = (heading_anim_target - current_heading + 540) % 360 - 180
                if abs(diff) < 0.2:
                    heading_anim_target = None
                else:
                    # Eased step: proportional with cap to avoid jumps; fast but smooth
                    step = min(4.5, abs(diff) * 0.6)  # deg per tick (~45 deg/s max)
                    new_heading = (current_heading + math.copysign(step, diff)) % 360
                    try:
                        pm.write_float(HEADING_ADDRESS, degrees_to_sim_value(new_heading))
                    except Exception as e:
                        print(f"Heading animation write error: {e}")
            
            # Refresh dynamic layers; throttling occurs inside draw functions
            try:
                draw_compass()
                if map_canvas_global is not None:
                    draw_map()
            except Exception:
                pass
        except Exception as e:
            # Swallow unexpected read/update errors to prevent flicker/log spam
            pass
    
    root.after(100, update_display)

def on_x_change(value):
    if not updating_sliders and not x_locked:
        try:
            pm.write_double(X_ADDRESS, float(value))
            x_value_label.config(text=f"{float(value):.1f}")
        except Exception as e:
            print(f"Error writing X: {e}")
            status_label.config(text=f"Error writing X position")

def on_y_change(value):
    if not updating_sliders and not y_locked:
        try:
            pm.write_double(Y_ADDRESS, float(value))
            y_value_label.config(text=f"{float(value):.1f}")
        except Exception as e:
            print(f"Error writing Y: {e}")
            status_label.config(text=f"Error writing Y position")

def adjust_x(delta):
    """Adjust X position by delta amount"""
    if not x_locked:
        x, _, _ = read_position()
        if x is not None:
            try:
                new_x = x + delta
                pm.write_double(X_ADDRESS, new_x)
                x_slider.set(new_x)
                x_value_label.config(text=f"{new_x:.1f}")
            except Exception as e:
                print(f"Error adjusting X: {e}")
                status_label.config(text=f"Error adjusting X position")

def adjust_y(delta):
    """Adjust Y position by delta amount"""
    if not y_locked:
        _, y, _ = read_position()
        if y is not None:
            try:
                new_y = y + delta
                pm.write_double(Y_ADDRESS, new_y)
                y_slider.set(new_y)
                y_value_label.config(text=f"{new_y:.1f}")
            except Exception as e:
                print(f"Error adjusting Y: {e}")
                status_label.config(text=f"Error adjusting Y position")

def adjust_heading(delta):
    """Adjust heading by delta degrees smoothly (ease in/out)"""
    global heading_anim_target
    global turn_inertia_active, turn_inertia_vector, turn_inertia_speed, turn_inertia_start_time
    if not heading_locked:
        x, y, heading_raw = read_position()
        if heading_raw is not None:
            try:
                current_deg = sim_value_to_degrees(heading_raw)
                # Start from existing target if animating, else from current
                base = heading_anim_target if heading_anim_target is not None else current_deg
                heading_anim_target = (base + delta) % 360

                # Capture a short-lived inertia vector in the direction of recent motion
                # Prefer course-over-ground when moving; fall back to bow heading otherwise
                if ship_speed > 0.1:
                    inertia_heading = calc_heading
                else:
                    inertia_heading = current_deg
                rad = math.radians(inertia_heading)
                turn_inertia_vector = (math.sin(rad), math.cos(rad))  # unit vector
                turn_inertia_speed = max(0.0, ship_speed)
                turn_inertia_start_time = time.time()
                turn_inertia_active = True

                # No immediate jump; update_display will animate toward target and apply drift
            except Exception as e:
                print(f"Error adjusting heading: {e}")
                status_label.config(text=f"Error adjusting heading")

def adjust_relative_x(delta):
    """Adjust X position relative to ship's heading"""
    global updating_relative_sliders
    if not x_locked:
        x, y, heading_raw = read_position()
        if x is not None and heading_raw is not None:
            try:
                heading_deg = sim_value_to_degrees(heading_raw)
                heading_rad = math.radians(heading_deg)
                
                # Calculate new position relative to heading
                # Right is positive X relative.
                # For 0 deg (North), right is +X. cos(0)=1, sin(0)=0. Correct.
                # For 90 deg (East), right is +Y. cos(90)=0, sin(90)=1. Correct.
                new_x = x + delta * math.cos(heading_rad)
                new_y = y + delta * math.sin(heading_rad)
                
                pm.write_double(X_ADDRESS, new_x)
                pm.write_double(Y_ADDRESS, new_y)
                
                updating_sliders = True
                try:
                    x_slider.set(new_x)
                    y_slider.set(new_y)
                    x_value_label.config(text=f"{new_x:.1f}")
                    y_value_label.config(text=f"{new_y:.1f}")
                except:
                    pass
                updating_sliders = False
            except Exception as e:
                print(f"Error adjusting relative X: {e}")
                status_label.config(text=f"Error adjusting relative position")

def adjust_relative_y(delta):
    """Adjust Y position relative to ship's heading"""
    global updating_relative_sliders
    if not y_locked:
        x, y, heading_raw = read_position()
        if x is not None and heading_raw is not None:
            try:
                heading_deg = sim_value_to_degrees(heading_raw)
                heading_rad = math.radians(heading_deg)
                
                # Calculate new position relative to heading
                # Forward is positive Y relative.
                # For 0 deg (North), forward is +Y. sin(0)=0, cos(0)=1. Correct.
                # For 90 deg (East), forward is +X. sin(90)=1, cos(90)=0. Correct.
                new_x = x + delta * math.sin(heading_rad)
                new_y = y + delta * math.cos(heading_rad)
                
                pm.write_double(X_ADDRESS, new_x)
                pm.write_double(Y_ADDRESS, new_y)
                
                updating_sliders = True
                try:
                    x_slider.set(new_x)
                    y_slider.set(new_y)
                    x_value_label.config(text=f"{new_x:.1f}")
                    y_value_label.config(text=f"{new_y:.1f}")
                except:
                    pass
                updating_sliders = False
            except Exception as e:
                print(f"Error adjusting relative Y: {e}")
                status_label.config(text=f"Error adjusting relative position")

def start_button_hold(action, repeat_delay=50):
    """Start holding a button and execute action repeatedly"""
    global button_hold_active, button_hold_action, button_hold_delay
    button_hold_active = True
    button_hold_action = action
    button_hold_delay = repeat_delay
    action()  # Execute immediately
    root.after(150, button_hold_repeat)  # Start repeating after 150ms

def button_hold_repeat():
    """Repeat button action while held"""
    global button_hold_active, button_hold_action, button_hold_delay
    if button_hold_active and button_hold_action:
        button_hold_action()
        root.after(button_hold_delay, button_hold_repeat)  # Use configured delay

def stop_button_hold(event=None):
    """Stop button hold"""
    global button_hold_active, button_hold_action
    button_hold_active = False
    button_hold_action = None

def on_relative_x_change(value):
    """Handle relative X slider change"""
    global updating_relative_sliders
    if not updating_sliders and not x_locked and not updating_relative_sliders and abs(value) > 1:
        updating_relative_sliders = True
        adjust_relative_x(value)
        try:
            rel_x_slider.set(0)  # Reset slider to center
        except:
            pass
        updating_relative_sliders = False

def on_relative_y_change(value):
    """Handle relative Y slider change"""
    global updating_relative_sliders
    if not updating_sliders and not y_locked and not updating_relative_sliders and abs(value) > 1:
        updating_relative_sliders = True
        adjust_relative_y(value)
        try:
            rel_y_slider.set(0)  # Reset slider to center
        except:
            pass
        updating_relative_sliders = False

def set_heading(degrees):
    """Set heading and update display"""
    global current_heading, locked_heading
    try:
        current_heading = degrees % 360
        heading_sim = degrees_to_sim_value(current_heading)
        pm.write_float(HEADING_ADDRESS, heading_sim)
        heading_value_label.config(text=f"{current_heading:.1f}°")
        
        # If heading is locked, update the locked value too
        if heading_locked:
            locked_heading = current_heading
        
        draw_compass()
    except Exception as e:
        print(f"Error setting heading: {e}")
        status_label.config(text=f"Error setting heading")

def toggle_heading_lock():
    """Toggle heading lock on/off"""
    global heading_locked, locked_heading
    
    heading_locked = heading_lock_var.get()
    
    if heading_locked:
        # Lock current heading
        locked_heading = current_heading
        heading_lock_label.config(text=f"Locked at {locked_heading:.1f}°", foreground="red")
        status_label.config(text=f"Heading locked at {locked_heading:.1f}°")
    else:
        heading_lock_label.config(text="Unlocked", foreground="green")
        status_label.config(text="Heading unlocked")

# Toggle heading lock via hotkey (Up arrow)
def toggle_heading_lock_hotkey():
    try:
        heading_lock_var.set(not heading_lock_var.get())
        toggle_heading_lock()
    except Exception as e:
        print(f"Heading lock toggle error: {e}")

# Restart autopilot via hotkey (Down arrow)
def restart_autopilot_hotkey():
    try:
        global autopilot_active
        # If autopilot is active, turn it off first
        if autopilot_active:
            toggle_autopilot()
            root.after(100, toggle_autopilot)  # Turn it back on after 100ms
        else:
            toggle_autopilot()  # Just turn it on
    except Exception as e:
        print(f"Restart autopilot error: {e}")

def toggle_x_lock():
    """Toggle X position lock on/off"""
    global x_locked, locked_x
    
    x_locked = x_lock_var.get()
    
    if x_locked:
        x, _, _ = read_position()
        if x is not None:
            locked_x = x
            x_lock_label.config(text=f"Locked", foreground="red")
            status_label.config(text=f"X position locked at {locked_x:.1f}")
        else:
            x_lock_var.set(False)
            x_locked = False
            status_label.config(text="Error: Cannot read position to lock")
    else:
        x_lock_label.config(text="Unlocked", foreground="green")
        status_label.config(text="X position unlocked")

def toggle_y_lock():
    """Toggle Y position lock on/off"""
    global y_locked, locked_y
    
    y_locked = y_lock_var.get()
    
    if y_locked:
        _, y, _ = read_position()
        if y is not None:
            locked_y = y
            y_lock_label.config(text=f"Locked", foreground="red")
            status_label.config(text=f"Y position locked at {locked_y:.1f}")
        else:
            y_lock_var.set(False)
            y_locked = False
            status_label.config(text="Error: Cannot read position to lock")
    else:
        y_lock_label.config(text="Unlocked", foreground="green")
        status_label.config(text="Y position unlocked")

def toggle_always_on_top():
    current_state = root.attributes('-topmost')
    root.attributes('-topmost', not current_state)
    topmost_btn.config(text="Always On Top: ON" if not current_state else "Always On Top: OFF")

# Tactical map window (separate window)
map_window = None
map_canvas_global = None
# Map view cache
map_view = None  # dict with keys: min_x, max_x, min_y, max_y, width, height, scale
# Map interaction state
map_panning = False
_map_pan_last = (0, 0)
# Undo/redo stacks
undo_stack = []  # list of waypoint lists snapshots
redo_stack = []

def push_undo_snapshot():
    global undo_stack, redo_stack
    undo_stack.append(copy.deepcopy(waypoints))
    # Clear redo on new change
    redo_stack.clear()

def perform_undo():
    global waypoints, undo_stack, redo_stack
    if not undo_stack:
        status_label.config(text="Nothing to undo")
        return
    redo_stack.append(copy.deepcopy(waypoints))
    waypoints = undo_stack.pop()
    update_waypoint_list()
    globals()['map_static_dirty'] = True
    draw_map()
    status_label.config(text="Undo")

def perform_redo():
    global waypoints, undo_stack, redo_stack
    if not redo_stack:
        status_label.config(text="Nothing to redo")
        return
    undo_stack.append(copy.deepcopy(waypoints))
    waypoints = redo_stack.pop()
    update_waypoint_list()
    globals()['map_static_dirty'] = True
    draw_map()
    status_label.config(text="Redo")

def find_nearest_waypoint(x, y):
    """Find the index of the nearest waypoint to the given coordinates."""
    nearest_idx = None
    nearest_dist = float('inf')
    for i, wp in enumerate(waypoints):
        dist = math.hypot(wp['x'] - x, wp['y'] - y)
        if dist < nearest_dist:
            nearest_dist = dist
            nearest_idx = i
    return nearest_idx

def map_world_to_canvas(wx, wy):
    global map_view
    if not map_view:
        return 0, 0
    scale = map_view['scale']
    width = map_view['width']
    height = map_view['height']
    min_x = map_view['min_x']
    min_y = map_view['min_y']
    cx = 20 + (wx - min_x) * scale
    cy = height - 20 - (wy - min_y) * scale
    return cx, cy

def map_canvas_to_world(cx, cy):
    global map_view
    if not map_view:
        return 0.0, 0.0
    scale = map_view['scale']
    width = map_view['width']
    height = map_view['height']
    min_x = map_view['min_x']
    min_y = map_view['min_y']
    wx = (cx - 20) / max(scale, 1e-9) + min_x
    wy = (height - 20 - cy) / max(scale, 1e-9) + min_y
    return wx, wy

# Compass drawing functions
_compass_cache = {}  # Cache for compass static elements

def draw_compass():
    """Draw the compass rose (optimized with caching)"""
    canvas.delete("all")
    
    # Get canvas size dynamically
    canvas.update_idletasks()
    width = canvas.winfo_width()
    height = canvas.winfo_height()
    
    if width < 50 or height < 50:  # Canvas not ready yet
        return
    
    center_x = width // 2
    center_y = height // 2
    radius = min(width, height) // 2 - 20
    
    # Check if we can reuse cached elements
    cache_key = f"{width}_{height}"
    if cache_key not in _compass_cache:
        # Cache static compass elements
        _compass_cache[cache_key] = {
            'center_x': center_x,
            'center_y': center_y,
            'radius': radius
        }
    
    # Draw outer circle
    canvas.create_oval(center_x - radius, center_y - radius,
                      center_x + radius, center_y + radius,
                      outline="black", width=2)
    
    # Draw degree marks every 30 degrees
    for angle in range(0, 360, 30):
        rad = math.radians(angle - 90)
        x1 = center_x + radius * 0.9 * math.cos(rad)
        y1 = center_y + radius * 0.9 * math.sin(rad)
        x2 = center_x + radius * 0.95 * math.cos(rad)
        y2 = center_y + radius * 0.95 * math.sin(rad)
        canvas.create_line(x1, y1, x2, y2, width=1)
        
        # Add degree labels
        label_x = center_x + radius * 0.8 * math.cos(rad)
        label_y = center_y + radius * 0.8 * math.sin(rad)
        canvas.create_text(label_x, label_y, text=f"{angle}°", font=("Arial", 8))
    
    # Draw inner circle
    inner_radius = 10
    canvas.create_oval(center_x - inner_radius, center_y - inner_radius,
                      center_x + inner_radius, center_y + inner_radius,
                      fill="gray", outline="black")
    
    # Draw cardinal directions
    directions = [
        (0, "N", "red"),
        (90, "E", "black"),
        (180, "S", "black"),
        (270, "W", "black")
    ]
    
    for angle, label, color in directions:
        rad = math.radians(angle - 90)
        x = center_x + radius * 1.1 * math.cos(rad)
        y = center_y + radius * 1.1 * math.sin(rad)
        
        canvas.create_text(x, y, text=label, font=("Arial", 16, "bold"), fill=color)
    
    # Draw heading pointer (ship's bow)
    rad = math.radians(current_heading - 90)
    pointer_length = radius * 0.7
    
    # Main pointer (arrow)
    end_x = center_x + pointer_length * math.cos(rad)
    end_y = center_y + pointer_length * math.sin(rad)
    
    # Arrow color based on lock status
    arrow_color = "red" if heading_locked else "blue"
    
    # Arrow body
    canvas.create_line(center_x, center_y, end_x, end_y,
                      fill=arrow_color, width=4)
    
    # Arrow head
    arrow_size = 15
    angle_offset = 150  # degrees
    left_rad = math.radians(current_heading - 90 + angle_offset)
    right_rad = math.radians(current_heading - 90 - angle_offset)
    
    left_x = end_x + arrow_size * math.cos(left_rad)
    left_y = end_y + arrow_size * math.sin(left_rad)
    right_x = end_x + arrow_size * math.cos(right_rad)
    right_y = end_y + arrow_size * math.sin(right_rad)
    
    canvas.create_polygon(end_x, end_y, left_x, left_y, right_x, right_y,
                         fill=arrow_color, outline="darkred" if heading_locked else "darkblue")
    
    # --- Bearing to next waypoint (green) ---
    try:
        wp_index = None
        if autopilot_active and target_waypoint_index is not None and 0 <= target_waypoint_index < len(waypoints):
            wp_index = target_waypoint_index
        elif selected_waypoint_index is not None and 0 <= selected_waypoint_index < len(waypoints):
            wp_index = selected_waypoint_index
        
        if wp_index is not None:
            wp = waypoints[wp_index]
            dx = wp['x'] - current_pos_x
            dy = wp['y'] - current_pos_y
            if abs(dx) + abs(dy) > 1e-6:
                bearing = (math.degrees(math.atan2(dx, dy)) + 360) % 360
                brad = math.radians(bearing - 90)
                blen = radius * 0.6
                bx = center_x + blen * math.cos(brad)
                by = center_y + blen * math.sin(brad)
                # Bearing arrow (dashed green)
                canvas.create_line(center_x, center_y, bx, by, fill="green", width=2, dash=(4, 2))
                # Bearing arrow head
                arrow_size_b = 10
                left_rad_b = math.radians(bearing - 90 + 150)
                right_rad_b = math.radians(bearing - 90 - 150)
                lx_b = bx + arrow_size_b * math.cos(left_rad_b)
                ly_b = by + arrow_size_b * math.sin(left_rad_b)
                rx_b = bx + arrow_size_b * math.cos(right_rad_b)
                ry_b = by + arrow_size_b * math.sin(right_rad_b)
                canvas.create_polygon(bx, by, lx_b, ly_b, rx_b, ry_b, fill="green", outline="darkgreen")
                # Bearing label inside compass
                canvas.create_text(center_x, center_y + radius - 12,
                                   text=f"BRG {bearing:.0f}° → WP {wp_index+1}",
                                   font=("Arial", 9, "bold"), fill="green")
    except Exception:
        pass


def on_canvas_resize(event):
    """Redraw compass when canvas is resized"""
    draw_compass()

def open_map_window():
    """Open a separate window with the tactical map."""
    global map_window, map_canvas_global
    # If window already exists, bring it to front
    if map_window is not None:
        try:
            map_window.lift()
            map_window.focus_force()
            return
        except Exception:
            map_window = None

    # Create new window
    map_window = tk.Toplevel(root)
    map_window.title("Tactical Map")
    map_window.geometry("800x600")
    map_window.attributes('-topmost', True)

    # Toolbar for edit mode
    toolbar = ttk.Frame(map_window)
    toolbar.pack(fill=tk.X, padx=5, pady=3)

    # Buoy visibility toggle
    buoys_var = tk.BooleanVar(value=True)
    def toggle_buoys():
        global buoys_visible, map_static_dirty
        buoys_visible = buoys_var.get()
        map_static_dirty = True
        draw_map()
        status_label.config(text=("Buoys: ON" if buoys_visible else "Buoys: OFF"))
    
    ttk.Checkbutton(toolbar, text="Show Buoys", variable=buoys_var, command=toggle_buoys).pack(side=tk.LEFT, padx=4)

    # Undo/Redo buttons
    ttk.Button(toolbar, text="Undo", command=perform_undo).pack(side=tk.LEFT, padx=4)
    ttk.Button(toolbar, text="Redo", command=perform_redo).pack(side=tk.LEFT)

    # Pan/Zoom controls
    def fit_to_route():
        global map_view, map_static_dirty
        map_view = None  # cause fresh bounds next draw
        map_static_dirty = True
        draw_map()
    ttk.Button(toolbar, text="Fit", command=fit_to_route).pack(side=tk.LEFT, padx=8)
    ttk.Label(toolbar, text="Left-drag: pan • Wheel: zoom", font=("Arial", 8)).pack(side=tk.LEFT, padx=8)

    # Create canvas
    map_canvas_global = tk.Canvas(map_window, bg="white")
    map_canvas_global.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    # Rebuild static layer when canvas size changes
    def on_map_configure(event):
        global map_static_dirty
        map_static_dirty = True
        draw_map()
    map_canvas_global.bind('<Configure>', on_map_configure)

    # Mark static layer dirty initially
    globals()['map_static_dirty'] = True

    # Pan with middle button or Shift+Left drag
    def on_pan_start(event):
        global map_panning, _map_pan_last
        map_panning = True
        _map_pan_last = (event.x, event.y)
    def on_pan_move(event):
        global map_panning, _map_pan_last, map_view
        if not map_panning or not map_view:
            return
        dx = event.x - _map_pan_last[0]
        dy = event.y - _map_pan_last[1]
        _map_pan_last = (event.x, event.y)
        # Convert pixel delta to world delta
        scale = map_view['scale']
        if scale <= 0:
            return
        ddx = -dx / scale
        ddy = dy / scale
        map_view['min_x'] += ddx
        map_view['max_x'] += ddx
        map_view['min_y'] += ddy
        map_view['max_y'] += ddy
        globals()['map_static_dirty'] = True
        draw_map()
    def on_pan_end(event):
        global map_panning
        map_panning = False
    map_canvas_global.bind('<Button-2>', on_pan_start)
    map_canvas_global.bind('<B2-Motion>', on_pan_move)
    map_canvas_global.bind('<ButtonRelease-2>', on_pan_end)
    # Shift + Left for pan
    def on_shift_left_start(event):
        if event.state & 0x0001:  # Shift mask in Tk
            on_pan_start(event)
    def on_shift_left_move(event):
        if map_panning:
            on_pan_move(event)
    def on_shift_left_end(event):
        if map_panning:
            on_pan_end(event)
    map_canvas_global.bind('<Shift-Button-1>', on_shift_left_start)
    map_canvas_global.bind('<Shift-B1-Motion>', on_shift_left_move)
    map_canvas_global.bind('<Shift-ButtonRelease-1>', on_shift_left_end)

    # Zoom with mouse wheel
    def on_mouse_wheel(event):
        global map_view
        if not map_view:
            return
        delta = event.delta
        zoom_in = delta > 0
        factor = 0.9 if zoom_in else 1.1
        wx, wy = map_canvas_to_world(event.x, event.y)
        min_x, max_x = map_view['min_x'], map_view['max_x']
        min_y, max_y = map_view['min_y'], map_view['max_y']
        def scale_bound(minv, maxv, focus, f):
            return focus + (minv - focus) * f, focus + (maxv - focus) * f
        new_min_x, new_max_x = scale_bound(min_x, max_x, wx, factor)
        new_min_y, new_max_y = scale_bound(min_y, max_y, wy, factor)
        map_view['min_x'], map_view['max_x'] = new_min_x, new_max_x
        map_view['min_y'], map_view['max_y'] = new_min_y, new_max_y
        globals()['map_static_dirty'] = True
        draw_map()
    # Windows wheel binding
    map_canvas_global.bind('<MouseWheel>', on_mouse_wheel)
    # Trackpad/touchpads may send different events; leave defaults for now

    # Handle window close
    def on_map_close():
        global map_window, map_canvas_global, map_panning
        try:
            map_window.destroy()
        except Exception:
            pass
        map_window = None
        map_canvas_global = None
        map_panning = False

   

    map_window.protocol("WM_DELETE_WINDOW", on_map_close)

    # Draw initial map
    draw_map()
    status_label.config(text="Tactical map opened")

# Tactical map flicker reduction: split static vs dynamic layers
map_static_dirty = True
_last_map_draw = 0.0


def draw_map():
    """Draw the live tactical map with reduced flicker using layered redraws."""
    global map_canvas_global, map_view, map_static_dirty, _last_map_draw
    if map_canvas_global is None:
        return
    
    try:
        map_canvas_global.update_idletasks()
        width = map_canvas_global.winfo_width()
        height = map_canvas_global.winfo_height()
        if width < 50 or height < 50:
            return
    except Exception as e:
        print(f"Map canvas error: {e}")
        return

    try:
        # Calculate map bounds and scale
        if map_view and all(k in map_view for k in ('min_x', 'max_x', 'min_y', 'max_y', 'width', 'height')):
            min_x = map_view['min_x']
            max_x = map_view['max_x']
            min_y = map_view['min_y']
            max_y = map_view['max_y']
        else:
            if not waypoints and not buoys:
                min_x, max_x = current_pos_x - 200, current_pos_x + 200
                min_y, max_y = current_pos_y - 200, current_pos_y + 200
            else:
                # Build lists of coordinates, handling empty lists safely
                all_x = [current_pos_x]
                all_y = [current_pos_y]
                
                if waypoints:
                    all_x.extend([wp['x'] for wp in waypoints])
                    all_y.extend([wp['y'] for wp in waypoints])
                
                if buoys:
                    all_x.extend([buoy['x'] for buoy in buoys])
                    all_y.extend([buoy['y'] for buoy in buoys])
                
                min_x, max_x = min(all_x) - 100, max(all_x) + 100
                min_y, max_y = min(all_y) - 100, max(all_y) + 100

        range_x = max(max_x - min_x, 100)
        range_y = max(max_y - min_y, 100)
        scale_x = (width - 40) / range_x
        scale_y = (height - 40) / range_y
        scale = min(scale_x, scale_y)

        map_view = {
            'min_x': min_x,
            'max_x': max_x,
            'min_y': min_y,
            'max_y': max_y,
            'width': width,
            'height': height,
            'scale': scale,
        }

        def world_to_canvas(x, y):
            cx = 20 + (x - min_x) * scale
            cy = height - 20 - (y - min_y) * scale
            return cx, cy

        # Rebuild static layer only when flagged (grid, waypoints, path, title)
        if map_static_dirty:
            try:
                map_canvas_global.delete('map_static')
            except Exception:
                pass
            # Grid
            grid_spacing = 50
            start_x = int(min_x // grid_spacing) * grid_spacing
            end_x = int(max_x) + grid_spacing
            for x in range(start_x, end_x, grid_spacing):
                cx1, cy1 = world_to_canvas(x, min_y)
                cx2, cy2 = world_to_canvas(x, max_x)
                map_canvas_global.create_line(cx1, cy1, cx2, cy2, fill='#e0e0e0', width=1, tags=('map_static',))
                map_canvas_global.create_text(cx1, height - 5, text=f"{x}", font=("Arial", 7), fill='gray', tags=('map_static',))
            start_y = int(min_y // grid_spacing) * grid_spacing
            end_y = int(max_y) + grid_spacing
            for y in range(start_y, end_y, grid_spacing):
                cx1, cy1 = world_to_canvas(min_x, y)
                cx2, cy2 = world_to_canvas(max_x, y)
                map_canvas_global.create_line(cx1, cy1, cx2, cy2, fill='#e0e0e0', width=1, tags=('map_static',))
                map_canvas_global.create_text(5, cy1, text=f"{y}", font=("Arial", 7), fill='gray', tags=('map_static',))
            # Path between waypoints
            if len(waypoints) > 1:
                for i in range(len(waypoints) - 1):
                    wp1 = waypoints[i]
                    wp2 = waypoints[i + 1]
                    x1, y1 = world_to_canvas(wp1['x'], wp1['y'])
                    x2, y2 = world_to_canvas(wp2['x'], wp2['y'])
                    map_canvas_global.create_line(x1, y1, x2, y2, fill='#ccccff', width=2, dash=(5, 3), tags=('map_static',))
            # Waypoints (uniform color; target highlight is dynamic overlay)
            for i, wp in enumerate(waypoints):
                cx, cy = world_to_canvas(wp['x'], wp['y'])
                buffer_radius = 25 * scale
                color = 'lightblue'
                map_canvas_global.create_oval(cx - buffer_radius, cy - buffer_radius,
                                              cx + buffer_radius, cy + buffer_radius,
                                              outline=color, width=2, dash=(3, 2), tags=('map_static',))
                wp_size = 8
                map_canvas_global.create_oval(cx - wp_size, cy - wp_size,
                                              cx + wp_size, cy + wp_size,
                                              fill=color, outline='darkblue', width=2, tags=('map_static',))
                map_canvas_global.create_text(cx, cy - wp_size - 10, text=f"WP{i+1}",
                                              font=("Arial", 9, 'bold'), fill='darkblue', tags=('map_static',))
                # Waypoint heading indicator (blue)
                wp_heading = sim_value_to_degrees(wp['heading_raw'])
                wp_rad = math.radians(wp_heading)
                arrow_len = 20
                arrow_x = cx + arrow_len * math.sin(wp_rad)
                arrow_y = cy - arrow_len * math.cos(wp_rad)
                map_canvas_global.create_line(cx, cy, arrow_x, arrow_y, fill='blue', width=2,
                                              arrow=tk.LAST, arrowshape=(10, 12, 5), tags=('map_static',))
            
            # Buoys (static markers - drawn in orange/yellow)
            if buoys_visible:
                for i, buoy in enumerate(buoys):
                    cx, cy = world_to_canvas(buoy['x'], buoy['y'])
                    buoy_size = 6
                    # Draw buoy as a diamond shape
                    map_canvas_global.create_polygon(
                        cx, cy - buoy_size,      # top
                        cx + buoy_size, cy,       # right
                        cx, cy + buoy_size,       # bottom
                        cx - buoy_size, cy,       # left
                        fill='orange', outline='darkorange', width=2, tags=('map_static',)
                    )
                    # Add label
                    map_canvas_global.create_text(cx, cy - buoy_size - 8, text=f"B{i+1}",
                                                  font=("Arial", 7), fill='darkorange', tags=('map_static',))
            
            # Title
            map_canvas_global.create_text(width / 2, 10, text='Tactical Map',
                                          font=("Arial", 12, 'bold'), fill='black', tags=('map_static',))
            map_static_dirty = False

        # Throttle dynamic layer redraws (~20 FPS)
        now = time.time()
        if now - _last_map_draw < 0.05:
            return
        _last_map_draw = now

        # Dynamic layer: ship, vectors, bearing, labels, target highlight, scale bar
        try:
            map_canvas_global.delete('map_dynamic')
        except Exception:
            pass

        # Ship
        ship_x, ship_y = world_to_canvas(current_pos_x, current_pos_y)
        ship_buffer_radius = 10 * scale
        map_canvas_global.create_oval(ship_x - ship_buffer_radius, ship_y - ship_buffer_radius,
                                      ship_x + ship_buffer_radius, ship_y + ship_buffer_radius,
                                      outline='orange', width=2, dash=(4, 2), tags=('map_dynamic',))
        ship_heading_rad = math.radians(current_heading)
        ship_size = 12
        nose_x = ship_x + ship_size * math.sin(ship_heading_rad)
        nose_y = ship_y - ship_size * math.cos(ship_heading_rad)
        left_x = ship_x + ship_size * 0.5 * math.sin(ship_heading_rad - 2.5)
        left_y = ship_y - ship_size * 0.5 * math.cos(ship_heading_rad - 2.5)
        right_x = ship_x + ship_size * 0.5 * math.sin(ship_heading_rad + 2.5)
        right_y = ship_y - ship_size * 0.5 * math.cos(ship_heading_rad + 2.5)
        map_canvas_global.create_polygon(nose_x, nose_y, left_x, left_y, right_x, right_y,
                                         fill='red', outline='darkred', width=2, tags=('map_dynamic',))

        # COG vector
        if ship_speed > 0.5:
            cog_rad = math.radians(calc_heading)
            vector_len = min(50, ship_speed * 5) * scale
            vec_end_x = ship_x + vector_len * math.sin(cog_rad)
            vec_end_y = ship_y - vector_len * math.cos(cog_rad)
            map_canvas_global.create_line(ship_x, ship_y, vec_end_x, vec_end_y,
                                          fill='purple', width=2, arrow=tk.LAST, arrowshape=(8, 10, 4), tags=('map_dynamic',))
            map_canvas_global.create_text(vec_end_x + 10, vec_end_y, text=f"COG {calc_heading:.0f}°",
                                          font=("Arial", 8), fill='purple', tags=('map_dynamic',))

        # Bearing to target and distance label
        if 'autopilot_active' in globals() and autopilot_active and 'target_waypoint_index' in globals() and target_waypoint_index is not None and 0 <= target_waypoint_index < len(waypoints):
            target_wp = waypoints[target_waypoint_index]
            target_x, target_y = world_to_canvas(target_wp['x'], target_wp['y'])
            map_canvas_global.create_line(ship_x, ship_y, target_x, target_y,
                                          fill='green', width=2, dash=(6, 4), arrow=tk.LAST, arrowshape=(10, 12, 5), tags=('map_dynamic',))
            distance = math.sqrt((target_wp['x'] - current_pos_x)**2 + (target_wp['y'] - current_pos_y)**2)
            mid_x = (ship_x + target_x) / 2
            mid_y = (ship_y + target_y) / 2
            map_canvas_global.create_text(mid_x, mid_y - 10, text=f"{distance:.0f} units",
                                          font=("Arial", 9, 'bold'), fill='green', tags=('map_dynamic',))
            # Target highlight ring
            buffer_radius = 25 * scale
            map_canvas_global.create_oval(target_x - buffer_radius, target_y - buffer_radius,
                                          target_x + buffer_radius, target_y + buffer_radius,
                                          outline='lightgreen', width=3, dash=(3, 2), tags=('map_dynamic',))

        # Ship label
        map_canvas_global.create_text(ship_x, ship_y + ship_size + 15, text='SHIP',
                                      font=("Arial", 10, 'bold'), fill='red', tags=('map_dynamic',))

        # Scale bar (dynamic because scale changes with zoom)
        scale_length = 50  # world units
        scale_pixels = scale_length * scale
        scale_x = width - 80
        scale_y = height - 20
        map_canvas_global.create_line(scale_x, scale_y, scale_x + scale_pixels, scale_y,
                                      fill='black', width=2, tags=('map_dynamic',))
        map_canvas_global.create_text(scale_x + scale_pixels / 2, scale_y - 10,
                                      text=f"{scale_length} units", font=("Arial", 8), fill='black', tags=('map_dynamic',))
    
    except Exception as e:
        print(f"Error drawing map: {e}")
        import traceback
        traceback.print_exc()
        # Don't let the error close the window
        try:
            if map_canvas_global:
                map_canvas_global.create_text(400, 300, text=f"Map Error: {e}", 
                                            fill='red', font=("Arial", 12, "bold"))
        except:
            pass

def on_compass_click(event):
    """Handle compass clicks"""
    if heading_locked:
        status_label.config(text="Cannot change heading - it's locked!")
        return
    
    canvas.update_idletasks()
    width = canvas.winfo_width()
    height = canvas.winfo_height()
    center_x = width // 2
    center_y = height // 2
    
    # Calculate angle from center
    dx = event.x - center_x
    dy = event.y - center_y
    
    # Convert to degrees (0° = North, clockwise)
    angle = math.degrees(math.atan2(dy, dx)) + 90
    angle = angle % 360
    
    set_heading(angle)

def on_compass_drag(event):
    """Handle compass dragging"""
    on_compass_click(event)

def on_canvas_resize(event):
    """Redraw compass when canvas is resized"""
    draw_compass()

# Global hotkeys setup (work when app is minimized and sim is focused)
# Requires the optional 'keyboard' package; falls back silently if unavailable.

# System tray icon support
tray_icon = None
_tray_icon_cache = None  # Cache the icon to avoid regenerating

def create_tray_icon():
    """Load the system tray icon (cached)"""
    global _tray_icon_cache
    if _tray_icon_cache is not None:
        return _tray_icon_cache
    
    # Load icon.ico
    try:
        # Get the correct base path for both frozen and script modes
        if getattr(sys, 'frozen', False):
            # Running as compiled executable - PyInstaller unpacks to _MEIPASS
            base_path = sys._MEIPASS
        else:
            # Running as script
            base_path = os.path.dirname(os.path.abspath(__file__))
        
        icon_path = os.path.join(base_path, 'icon', 'favicon.ico')
        if os.path.exists(icon_path):
            _tray_icon_cache = Image.open(icon_path)
            return _tray_icon_cache
        else:
            raise FileNotFoundError(f"Icon file not found: {icon_path}")
    except Exception as e:
        print(f"ERROR: Could not load favicon.ico for tray: {e}")
        raise

def show_window(icon=None, item=None):
    """Show the main window"""
    root.after(0, root.deiconify)

def hide_window():
    """Hide to system tray"""
    root.withdraw()

def quit_app(icon=None, item=None):
    """Quit the application"""
    if icon:
        icon.stop()
    root.after(0, on_close)

def tray_load_waypoints(icon=None, item=None):
    """Load waypoints from tray menu (opens file dialog)"""
    root.after(0, load_waypoints)

def tray_load_specific_file(filepath):
    """Load a specific waypoint file"""
    def _load():
        global waypoints, buoys
        try:
            with open(filepath, 'r') as f:
                loaded_data = json.load(f)
            
            # Check if it's new format with waypoints and buoys sections
            if isinstance(loaded_data, dict) and 'waypoints' in loaded_data:
                waypoints = loaded_data.get('waypoints', [])
                buoys = loaded_data.get('buoys', [])
                status_msg = f"Loaded {len(waypoints)} waypoints and {len(buoys)} buoys from {os.path.basename(filepath)}"
            else:
                # Old format - just waypoints
                waypoints = loaded_data
                buoys = []
                status_msg = f"Loaded {len(waypoints)} waypoints from {os.path.basename(filepath)}"
            
            waypoints.sort(key=lambda w: w['time'])
            update_waypoint_list()
            globals()['map_static_dirty'] = True
            draw_map()
            status_label.config(text=status_msg)
        except Exception as e:
            status_label.config(text=f"Load error: {e}")
    
    root.after(0, _load)

def get_json_files_menu():
    """Get list of JSON files in executable directory for tray submenu"""
    try:
        # Get directory where exe/script is located
        if getattr(sys, 'frozen', False):
            # Running as compiled executable
            exe_dir = os.path.dirname(sys.executable)
        else:
            # Running as script
            exe_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Find all .json files
        json_files = [f for f in os.listdir(exe_dir) if f.lower().endswith('.json')]
        json_files.sort()
        
        if not json_files:
            return [item('(No JSON files found)', lambda icon, item: None, enabled=False)]
        
        # Create menu items for each JSON file
        menu_items = []
        for json_file in json_files:
            filepath = os.path.join(exe_dir, json_file)
            # Create a proper callback that captures filepath
            # pystray expects callbacks with signature (icon, item)
            def make_callback(fp):
                return lambda icon, item: tray_load_specific_file(fp)
            
            menu_items.append(item(json_file, make_callback(filepath)))
        
        return menu_items
    except Exception as e:
        # Write error to a log file for debugging
        try:
            log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tray_error.log')
            with open(log_path, 'a') as log:
                log.write(f"Error in get_json_files_menu: {e}\n")
                import traceback
                traceback.print_exc(file=log)
        except:
            pass
        return [item('(Error loading files)', lambda icon, item: None, enabled=False)]

def tray_start_autopilot(icon=None, item=None):
    """Start autopilot from tray menu"""
    def _start():
        if not autopilot_active:
            toggle_autopilot()
    root.after(0, _start)

def setup_tray_icon():
    """Setup system tray icon (deferred to background thread)"""
    global tray_icon
    
    def _setup_tray():
        global tray_icon
        icon_image = create_tray_icon()
        
        # Create submenu for Load with JSON files
        load_submenu = pystray.Menu(*get_json_files_menu())
        
        menu = pystray.Menu(
            item('Show', show_window, default=True),
            item('Load', load_submenu),
            item('Start Autopilot', tray_start_autopilot),
            item('Quit', quit_app)
        )
        tray_icon = pystray.Icon("smmm_simuIation", icon_image, "smmm_simuIation", menu)
        tray_icon.run()
    
    # Run icon setup in separate thread to avoid blocking startup
    threading.Thread(target=_setup_tray, daemon=True).start()

def on_window_close():
    """Handle window close button - minimize to tray instead of closing"""
    hide_window()
    status_label.config(text="Minimized to system tray")

def setup_global_hotkeys():
    try:
        if not ('HAS_GLOBAL_HOTKEYS' in globals() and HAS_GLOBAL_HOTKEYS):
            return
        # Guard to avoid double registration
        if globals().get('GLOBAL_HOTKEYS_SET', False):
            return
        def _z_up(e):
            adjust_heading(-2.5)  # Z - turn left 2.5°
        def _x_up(e):
            adjust_heading(2.5)  # X - turn right 2.5°
        def _c_up(e):
            restart_autopilot_hotkey()  # C - restart autopilot
        def _shift_up(e):
            toggle_heading_lock_hotkey()  # Left Shift - lock heading
        def _space_up(e):
            teleport_to_autopilot_target()
        def _esc_up(e):
            # Global Escape key to quit app completely, even when minimized to tray
            quit_app()
        import keyboard  # type: ignore
        keyboard.on_release_key('z', _z_up, suppress=False)
        keyboard.on_release_key('x', _x_up, suppress=False)
        keyboard.on_release_key('c', _c_up, suppress=False)
        keyboard.on_release_key('shift', _shift_up, suppress=False)  # Left Shift
        keyboard.on_release_key('space', _space_up, suppress=False)
        keyboard.on_release_key('esc', _esc_up, suppress=False)
        globals()['GLOBAL_HOTKEYS_SET'] = True
    except Exception:
        pass

def teardown_global_hotskeys():
    try:
        if 'HAS_GLOBAL_HOTKEYS' in globals() and HAS_GLOBAL_HOTKEYS:
            import keyboard  # type: ignore
            keyboard.unhook_all()
        globals()['GLOBAL_HOTKEYS_SET'] = False
    except Exception:
        pass

# Ensure we unhook global hotkeys on exit to avoid dangling hooks
# Define close handler early so it exists before we register WM_DELETE_WINDOW
def on_close():
    """Full application shutdown"""
    global tray_icon
    try:
        teardown_global_hotskeys()
    except Exception:
        pass
    try:
        if tray_icon:
            tray_icon.stop()
    except Exception:
        pass
    try:
        root.destroy()
    except Exception:
        pass

# Create GUI
root = tk.Tk()
root.title("smmm_simuIation")
root.geometry("200x200")

# Set window icon (if available)
try:
    # Get the correct base path for both frozen and script modes
    if getattr(sys, 'frozen', False):
        base_path = sys._MEIPASS
    else:
        base_path = os.path.dirname(os.path.abspath(__file__))
    
    icon_path = os.path.join(base_path, 'icon', 'icon.ico')
    if os.path.exists(icon_path):
        root.iconbitmap(icon_path)
except Exception as e:
    print(f"Could not load icon: {e}")

# Start minimized to system tray (background mode)
root.overrideredirect(False)  # Keep window decorations
root.attributes('-topmost', True)
root.protocol("WM_DELETE_WINDOW", on_window_close)  # Minimize to tray instead of closing

# Defer heavy operations until after window is shown
def _deferred_setup():
    """Setup expensive operations after window is ready"""
    # Setup global hotkeys (work when minimized)
    setup_global_hotkeys()
    
    # Setup system tray icon
    try:
        setup_tray_icon()
        status_label.config(text="Ready - System tray icon active")
    except Exception as e:
        print(f"Failed to setup tray icon: {e}")
        status_label.config(text="Ready - Tray icon unavailable")
    
    # Hide window after initialization
    root.after(50, lambda: root.withdraw())
    root.after(100, lambda: status_label.config(text="Started in background - Double-click tray icon to show"))

# Status bar
status_frame = ttk.Frame(root, padding="2")
status_frame.pack(fill=tk.X)
status_label = ttk.Label(status_frame, text="Ready", relief=tk.SUNKEN)
status_label.pack(fill=tk.X)

# Current position display
info_frame = ttk.Frame(root, padding="3")
info_frame.pack(fill=tk.X)
read_label = ttk.Label(info_frame, text="Current: X=--, Y=--, H_raw=--", font=("Arial", 8))
read_label.pack()
speed_label = ttk.Label(info_frame, text="Speed: 0.0 u/s", font=("Arial", 8, "bold"))
speed_label.pack()

# Retry connect button if not connected
retry_btn = ttk.Button(info_frame, text="Retry Connect", command=lambda: retry_connect())
retry_btn.pack(pady=1)

topmost_btn = ttk.Button(info_frame, text="Always On Top: ON", command=toggle_always_on_top)
topmost_btn.pack(pady=1)

# Main control frame - split into left and right
main_control_frame = ttk.Frame(root)
main_control_frame.pack(fill=tk.BOTH, expand=True, padx=3, pady=1)

# Configure grid to split 50/50 with proper weights
main_control_frame.columnconfigure(0, weight=1)  # left_frame column
main_control_frame.columnconfigure(1, weight=1)  # right_frame column
main_control_frame.rowconfigure(0, weight=1)     # both frames in row 0

# Left side - Position controls (X and Y)
left_frame = ttk.LabelFrame(main_control_frame, text="Position Control", padding="3")
left_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 1))

# X slider
x_frame = ttk.Frame(left_frame)
x_frame.pack(fill=tk.X, pady=1)
ttk.Label(x_frame, text="X:", font=("Arial", 8)).pack(side=tk.LEFT)
x_value_label = ttk.Label(x_frame, text="0.0", width=6, font=("Arial", 8, "bold"))
x_value_label.pack(side=tk.RIGHT)

x_slider = ttk.Scale(left_frame, from_=-1200, to=1200, orient=tk.HORIZONTAL, command=on_x_change)
x_slider.pack(fill=tk.X, pady=1)

# X adjustment buttons
x_btn_frame = ttk.Frame(left_frame)
x_btn_frame.pack(fill=tk.X, pady=1)
x_btn_ll = ttk.Button(x_btn_frame, text="◄◄", width=3)
x_btn_ll.pack(side=tk.LEFT, padx=1)
x_btn_ll.bind('<ButtonPress-1>', lambda e: start_button_hold(lambda: adjust_x(-20)))
x_btn_ll.bind('<ButtonRelease-1>', stop_button_hold)

x_btn_l = ttk.Button(x_btn_frame, text="◄", width=3)
x_btn_l.pack(side=tk.LEFT, padx=1)
x_btn_l.bind('<ButtonPress-1>', lambda e: start_button_hold(lambda: adjust_x(-5)))
x_btn_l.bind('<ButtonRelease-1>', stop_button_hold)

x_btn_r = ttk.Button(x_btn_frame, text="►", width=3)
x_btn_r.pack(side=tk.LEFT, padx=1)
x_btn_r.bind('<ButtonPress-1>', lambda e: start_button_hold(lambda: adjust_x(5)))
x_btn_r.bind('<ButtonRelease-1>', stop_button_hold)

x_btn_rr = ttk.Button(x_btn_frame, text="►►", width=3)
x_btn_rr.pack(side=tk.LEFT, padx=1)
x_btn_rr.bind('<ButtonPress-1>', lambda e: start_button_hold(lambda: adjust_x(20)))
x_btn_rr.bind('<ButtonRelease-1>', stop_button_hold)

# X lock checkbox
x_lock_frame = ttk.Frame(left_frame)
x_lock_frame.pack(pady=1)

x_lock_var = tk.BooleanVar(value=False)
x_lock_check = ttk.Checkbutton(x_lock_frame, text="Lock X", 
                                variable=x_lock_var, 
                                command=toggle_x_lock)
x_lock_check.pack(side=tk.LEFT, padx=2)

x_lock_label = ttk.Label(x_lock_frame, text="Unlocked", foreground="green", 
                         font=("Arial", 8, "bold"))
x_lock_label.pack(side=tk.LEFT, padx=2)

# Y slider
y_frame = ttk.Frame(left_frame)
y_frame.pack(fill=tk.X, pady=1)
ttk.Label(y_frame, text="Y:", font=("Arial", 8)).pack(side=tk.LEFT)
y_value_label = ttk.Label(y_frame, text="0.0", width=6, font=("Arial", 8, "bold"))
y_value_label.pack(side=tk.RIGHT)

y_slider = ttk.Scale(left_frame, from_=-1200, to=1200 , orient=tk.HORIZONTAL, command=on_y_change)
y_slider.pack(fill=tk.X, pady=1)

# Y adjustment buttons
y_btn_frame = ttk.Frame(left_frame)
y_btn_frame.pack(fill=tk.X, pady=1)
y_btn_dd = ttk.Button(y_btn_frame, text="▼▼", width=3)
y_btn_dd.pack(side=tk.LEFT, padx=1)
y_btn_dd.bind('<ButtonPress-1>', lambda e: start_button_hold(lambda: adjust_y(-20)))
y_btn_dd.bind('<ButtonRelease-1>', stop_button_hold)

y_btn_d = ttk.Button(y_btn_frame, text="▼", width=3)
y_btn_d.pack(side=tk.LEFT, padx=1)
y_btn_d.bind('<ButtonPress-1>', lambda e: start_button_hold(lambda: adjust_y(-5)))
y_btn_d.bind('<ButtonRelease-1>', stop_button_hold)

y_btn_u = ttk.Button(y_btn_frame, text="▲", width=3)
y_btn_u.pack(side=tk.LEFT, padx=1)
y_btn_u.bind('<ButtonPress-1>', lambda e: start_button_hold(lambda: adjust_y(5)))
y_btn_u.bind('<ButtonRelease-1>', stop_button_hold)

y_btn_uu = ttk.Button(y_btn_frame, text="▲▲", width=3)
y_btn_uu.pack(side=tk.LEFT, padx=1)
y_btn_uu.bind('<ButtonPress-1>', lambda e: start_button_hold(lambda: adjust_y(20)))
y_btn_uu.bind('<ButtonRelease-1>', stop_button_hold)

# Y lock checkbox
y_lock_frame = ttk.Frame(left_frame)
y_lock_frame.pack(pady=1)

y_lock_var = tk.BooleanVar(value=False)
y_lock_check = ttk.Checkbutton(y_lock_frame, text="Lock Y", 
                                variable=y_lock_var, 
                                command=toggle_y_lock)
y_lock_check.pack(side=tk.LEFT, padx=2)

y_lock_label = ttk.Label(y_lock_frame, text="Unlocked", foreground="green", 
                         font=("Arial", 8, "bold"))
y_lock_label.pack(side=tk.LEFT, padx=2)

# Separator
ttk.Separator(left_frame, orient='horizontal').pack(fill=tk.X, pady=3)

# Relative position controls (ship-relative)
ttk.Label(left_frame, text="Ship-Relative Controls", font=("Arial", 8, "bold")).pack(pady=1)

# Relative X (right/left from ship's perspective)
rel_x_frame = ttk.Frame(left_frame)
rel_x_frame.pack(fill=tk.X, pady=1)
ttk.Label(rel_x_frame, text="Right/Left:", font=("Arial", 8)).pack(side=tk.LEFT)

# Relative X slider
rel_x_slider = ttk.Scale(left_frame, from_=-100, to=100, orient=tk.HORIZONTAL, command=lambda v: on_relative_x_change(float(v)))
rel_x_slider.pack(fill=tk.X, pady=1)
rel_x_slider.set(0)

rel_x_btn_frame = ttk.Frame(left_frame)
rel_x_btn_frame.pack(fill=tk.X, pady=1)
rel_x_btn_ll = ttk.Button(rel_x_btn_frame, text="◄◄", width=3)
rel_x_btn_ll.pack(side=tk.LEFT, padx=1)
rel_x_btn_ll.bind('<ButtonPress-1>', lambda e: start_button_hold(lambda: adjust_relative_x(-20)))
rel_x_btn_ll.bind('<ButtonRelease-1>', stop_button_hold)

rel_x_btn_l = ttk.Button(rel_x_btn_frame, text="◄", width=3)
rel_x_btn_l.pack(side=tk.LEFT, padx=1)
rel_x_btn_l.bind('<ButtonPress-1>', lambda e: start_button_hold(lambda: adjust_relative_x(-5)))
rel_x_btn_l.bind('<ButtonRelease-1>', stop_button_hold)

rel_x_btn_r = ttk.Button(rel_x_btn_frame, text="►", width=3)
rel_x_btn_r.pack(side=tk.LEFT, padx=1)
rel_x_btn_r.bind('<ButtonPress-1>', lambda e: start_button_hold(lambda: adjust_relative_x(5)))
rel_x_btn_r.bind('<ButtonRelease-1>', stop_button_hold)

rel_x_btn_rr = ttk.Button(rel_x_btn_frame, text="►►", width=3)
rel_x_btn_rr.pack(side=tk.LEFT, padx=1)
rel_x_btn_rr.bind('<ButtonPress-1>', lambda e: start_button_hold(lambda: adjust_relative_x(20)))
rel_x_btn_rr.bind('<ButtonRelease-1>', stop_button_hold)

# Relative Y (forward/backward from ship's perspective)
rel_y_frame = ttk.Frame(left_frame)
rel_y_frame.pack(fill=tk.X, pady=1)
ttk.Label(rel_y_frame, text="Fwd/Back:", font=("Arial", 8)).pack(side=tk.LEFT)

# Relative Y slider
rel_y_slider = ttk.Scale(left_frame, from_=-100, to=100, orient=tk.HORIZONTAL, command=lambda v: on_relative_y_change(float(v)))
rel_y_slider.pack(fill=tk.X, pady=1)
rel_y_slider.set(0)

rel_y_btn_frame = ttk.Frame(left_frame)
rel_y_btn_frame.pack(fill=tk.X, pady=1)
rel_y_btn_dd = ttk.Button(rel_y_btn_frame, text="▼▼", width=3)
rel_y_btn_dd.pack(side=tk.LEFT, padx=1)
rel_y_btn_dd.bind('<ButtonPress-1>', lambda e: start_button_hold(lambda: adjust_relative_y(-20)))
rel_y_btn_dd.bind('<ButtonRelease-1>', stop_button_hold)

rel_y_btn_d = ttk.Button(rel_y_btn_frame, text="▼", width=3)
rel_y_btn_d.pack(side=tk.LEFT, padx=1)
rel_y_btn_d.bind('<ButtonPress-1>', lambda e: start_button_hold(lambda: adjust_relative_y(-5)))
rel_y_btn_d.bind('<ButtonRelease-1>', stop_button_hold)

rel_y_btn_u = ttk.Button(rel_y_btn_frame, text="▲", width=3)
rel_y_btn_u.pack(side=tk.LEFT, padx=1)
rel_y_btn_u.bind('<ButtonPress-1>', lambda e: start_button_hold(lambda: adjust_relative_y(5)))
rel_y_btn_u.bind('<ButtonRelease-1>', stop_button_hold)

rel_y_btn_uu = ttk.Button(rel_y_btn_frame, text="▲▲", width=3)
rel_y_btn_uu.pack(side=tk.LEFT, padx=1)
rel_y_btn_uu.bind('<ButtonPress-1>', lambda e: start_button_hold(lambda: adjust_relative_y(20)))
rel_y_btn_uu.bind('<ButtonRelease-1>', stop_button_hold)

# Right side - Compass
right_frame = ttk.LabelFrame(main_control_frame, text="Heading Control", padding="3")
right_frame.grid(row=0, column=1, sticky='nsew', padx=(1, 0))

heading_value_label = ttk.Label(right_frame, text="0.0°", font=("Arial", 10, "bold"))
heading_value_label.pack()

# Heading adjustment buttons
heading_adj_frame = ttk.Frame(right_frame)
heading_adj_frame.pack(pady=1)
heading_btn_l = ttk.Button(heading_adj_frame, text="◄", width=3)
heading_btn_l.pack(side=tk.LEFT, padx=1)
heading_btn_l.bind('<ButtonPress-1>', lambda e: start_button_hold(lambda: adjust_heading(-2.5)))
heading_btn_l.bind('<ButtonRelease-1>', stop_button_hold)

heading_btn_r = ttk.Button(heading_adj_frame, text="►", width=3)
heading_btn_r.pack(side=tk.LEFT, padx=1)
heading_btn_r.bind('<ButtonPress-1>', lambda e: start_button_hold(lambda: adjust_heading(2.5)))
heading_btn_r.bind('<ButtonRelease-1>', stop_button_hold)

# Heading lock checkbox
lock_frame = ttk.Frame(right_frame)
lock_frame.pack(pady=1)

heading_lock_var = tk.BooleanVar(value=False)
heading_lock_check = ttk.Checkbutton(lock_frame, text="Lock", 
                                      variable=heading_lock_var, 
                                      command=toggle_heading_lock)
heading_lock_check.pack(side=tk.LEFT, padx=2)

heading_lock_label = ttk.Label(lock_frame, text="Unlocked", foreground="green", 
                               font=("Arial", 8, "bold"))
heading_lock_label.pack(side=tk.LEFT, padx=2)

canvas = tk.Canvas(right_frame, width=120, height=120, bg="white")
canvas.pack(fill=tk.BOTH, expand=True, pady=1)
canvas.bind("<Button-1>", on_compass_click)
canvas.bind("<B1-Motion>", on_compass_drag)
canvas.bind("<Configure>", on_canvas_resize)

# Defer compass drawing until after window is shown
root.after(10, draw_compass)

# Quick heading buttons
heading_btn_frame = ttk.Frame(right_frame)
heading_btn_frame.pack(pady=1)
ttk.Button(heading_btn_frame, text="N", command=lambda: set_heading(0), width=5).grid(row=0, column=1, padx=1, pady=1)
ttk.Button(heading_btn_frame, text="NE", command=lambda: set_heading(45), width=5).grid(row=0, column=2, padx=1, pady=1)
ttk.Button(heading_btn_frame, text="W", command=lambda: set_heading(270), width=5).grid(row=1, column=0, padx=1, pady=1)
ttk.Button(heading_btn_frame, text="E", command=lambda: set_heading(90), width=5).grid(row=1, column=2, padx=1, pady=1)
ttk.Button(heading_btn_frame, text="SW", command=lambda: set_heading(225), width=5).grid(row=2, column=0, padx=1, pady=1)
ttk.Button(heading_btn_frame, text="S", command=lambda: set_heading(180), width=5).grid(row=2, column=1, padx=1, pady=1)
ttk.Button(heading_btn_frame, text="SE", command=lambda: set_heading(135), width=5).grid(row=2, column=2, padx=1, pady=1)

# Compass will be drawn after window is ready (see after call above)

# Manual waypoint entry
entry_frame = ttk.LabelFrame(root, text="Add Manual Waypoint", padding="3")
entry_frame.pack(fill=tk.X, padx=3, pady=1)

ttk.Label(entry_frame, text="T(s):").grid(row=0, column=0, padx=1)
time_entry = ttk.Entry(entry_frame, width=5)
time_entry.grid(row=0, column=1, padx=1)
time_entry.insert(0, "0")

ttk.Label(entry_frame, text="X:").grid(row=0, column=2, padx=1)
x_entry = ttk.Entry(entry_frame, width=5)
x_entry.grid(row=0, column=3, padx=1)

ttk.Label(entry_frame, text="Y:").grid(row=0, column=4, padx=1)
y_entry = ttk.Entry(entry_frame, width=5)
y_entry.grid(row=0, column=5, padx=1)

ttk.Label(entry_frame, text="H:").grid(row=0, column=6, padx=1)
heading_entry = ttk.Entry(entry_frame, width=5)
heading_entry.grid(row=0, column=7, padx=1)

ttk.Button(entry_frame, text="Add", command=add_manual_waypoint).grid(row=0, column=8, padx=1)

# Waypoint control buttons
waypoint_control_frame = ttk.Frame(root, padding="3")
waypoint_control_frame.pack(fill=tk.X)

ttk.Button(waypoint_control_frame, text="Add Current", command=add_waypoint).pack(side=tk.LEFT, padx=1)

auto_record_var = tk.BooleanVar(value=False)
auto_record_btn = ttk.Checkbutton(waypoint_control_frame, text="⏺ Auto Record", 
                                   variable=auto_record_var, 
                                   command=toggle_auto_record)
auto_record_btn.pack(side=tk.LEFT, padx=1)

ttk.Button(waypoint_control_frame, text="Clear All", command=clear_waypoints).pack(side=tk.LEFT, padx=1)
ttk.Button(waypoint_control_frame, text="Save", command=save_waypoints).pack(side=tk.LEFT, padx=1)
ttk.Button(waypoint_control_frame, text="Load", command=load_waypoints).pack(side=tk.LEFT, padx=1)
ttk.Button(waypoint_control_frame, text="📍 Map", command=open_map_window).pack(side=tk.LEFT, padx=5)
autopilot_btn = ttk.Button(waypoint_control_frame, text="▶️ Autopilot", command=toggle_autopilot)
autopilot_btn.pack(side=tk.LEFT, padx=5)

# Waypoint editing buttons
waypoint_edit_frame = ttk.Frame(root, padding="3")
waypoint_edit_frame.pack(fill=tk.X)

ttk.Button(waypoint_edit_frame, text="Edit Selected", command=edit_selected_waypoint).pack(side=tk.LEFT, padx=1)
ttk.Button(waypoint_edit_frame, text="Delete Selected", command=delete_selected_waypoint).pack(side=tk.LEFT, padx=1)
ttk.Button(waypoint_edit_frame, text="Jump to Selected", command=jump_to_selected_waypoint).pack(side=tk.LEFT, padx=1)

# Move buttons
move_frame = ttk.Frame(waypoint_edit_frame)
move_frame.pack(side=tk.LEFT, padx=10)
ttk.Button(move_frame, text="▲ Move Up", command=lambda: move_waypoint(-1)).pack(side=tk.LEFT)
ttk.Button(move_frame, text="▼ Move Down", command=lambda: move_waypoint(1)).pack(side=tk.LEFT, padx=1)

# Waypoint list display
waypoint_frame = ttk.LabelFrame(root, text="Waypoints", padding="3")
waypoint_frame.pack(fill=tk.BOTH, expand=True, padx=3, pady=1)

# Create Treeview for waypoints
columns = ('#', 'time', 'x', 'y', 'heading')
waypoint_tree = ttk.Treeview(waypoint_frame, columns=columns, show='headings', height=5)

# Define headings
waypoint_tree.heading('#', text='#')
waypoint_tree.heading('time', text='Time (s)')
waypoint_tree.heading('x', text='X')
waypoint_tree.heading('y', text='Y')
waypoint_tree.heading('heading', text='Heading')

# Define column widths
waypoint_tree.column('#', width=40, anchor=tk.CENTER)
waypoint_tree.column('time', width=80, anchor=tk.CENTER)
waypoint_tree.column('x', width=100, anchor=tk.CENTER)
waypoint_tree.column('y', width=100, anchor=tk.CENTER)
waypoint_tree.column('heading', width=100, anchor=tk.CENTER)

waypoint_tree.pack(fill=tk.BOTH, expand=True)
waypoint_tree.bind('<<TreeviewSelect>>', on_waypoint_select)

# Smoothing controls
smooth_frame = ttk.LabelFrame(root, text="Path Smoothing", padding="3")
smooth_frame.pack(fill=tk.X, padx=3, pady=1)

smooth_btn_frame = ttk.Frame(smooth_frame)
smooth_btn_frame.pack(fill=tk.X)

ttk.Button(smooth_btn_frame, text="Generate Smooth Path", command=smooth_waypoints).pack(side=tk.LEFT, padx=1)

smooth_text = scrolledtext.ScrolledText(smooth_frame, height=1, state='disabled', font=("Arial", 8))
smooth_text.pack(fill=tk.X, pady=1)

# Replay controls
replay_frame = ttk.LabelFrame(root, text="Replay", padding="3")
replay_frame.pack(fill=tk.X, padx=3, pady=1)

replay_btn = ttk.Button(replay_frame, text="▶ Start", command=start_replay)
replay_btn.pack(side=tk.LEFT, padx=1)

stop_btn = ttk.Button(replay_frame, text="⏹ Stop", command=stop_replay, state='disabled')
stop_btn.pack(side=tk.LEFT, padx=1)

ttk.Label(replay_frame, text="Time-based execution", font=("Arial", 8)).pack(side=tk.LEFT, padx=3)

# Hotkeys (local - when window focused)
root.bind_all('<KeyRelease-z>', lambda e: adjust_heading(-1))  # Z - turn left 2.5°
root.bind_all('<KeyRelease-x>', lambda e: adjust_heading(1))   # X - turn right 2.5°
root.bind_all('<KeyRelease-c>', lambda e: restart_autopilot_hotkey())  # C - restart autopilot
root.bind_all('<KeyRelease-Shift_L>', lambda e: toggle_heading_lock_hotkey())  # Left Shift - lock heading
# Additional local hotkeys (keep these for compatibility)
root.bind_all('<KeyRelease-space>', lambda e: teleport_to_autopilot_target())

# Start updating display immediately
update_display()

# Defer expensive initialization to background
root.after(1, _deferred_setup)

def retry_connect():
    """Attempt to connect to the simulation process at runtime"""
    global pm, base_address, Y_ADDRESS, X_ADDRESS
    try:
        pm = pymem.Pymem('smmm_simulation.exe')
        base_address = pymem.process.module_from_name(pm.process_handle, 'smmm_simulation.exe').lpBaseOfDll
        Y_ADDRESS = base_address + Y_OFFSET
        X_ADDRESS = base_address + X_OFFSET
        status_label.config(text="Connected to smmm_simulation.exe")
    except Exception as e:
        status_label.config(text=f"Not connected: {e}")
        try:
            messagebox.showwarning("Connection failed", "Start the simulation and try again.")
        except Exception:
            pass

# --- Main loop ---
if __name__ == "__main__":
    try:
        # Enter Tkinter event loop so the UI stays open and responsive
        root.mainloop()
    except Exception as e:
        try:
            print(f"UI loop error: {e}")
        except Exception:
            pass
    finally:
        # Ensure cleanup on exit (tray icon, hotkeys, tk destroy)
        try:
            on_close()
        except Exception:
            pass