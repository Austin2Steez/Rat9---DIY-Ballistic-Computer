# PART 1/7 — Core Imports, Globals, Setup, UART/GPIO/SPI Initialization
# GPIO Pinout:
# KEY1 (Select)      -> GPIO 21
# KEY2 (Back)        -> GPIO 20
# KEY3 (Mode Toggle) -> GPIO 16
# Joystick (Optional) Up/Down/Left/Right -> To be defined in Part 2
# External Trigger   -> GPIO 17 (manual_button_pin)

import tkinter as tk
from tkinter import ttk
from sense_hat import SenseHat
try:
    from ballistics import simulate_trajectory, BULLETS
except ImportError:
    simulate_trajectory = None
    BULLETS = {'.308_M80': {}}
    logging.error("Failed to import ballistics module. GUI will load in safe mode.")
import time
import serial
import os
import math
import json
import RPi.GPIO as GPIO
import spidev
from gpiozero import Button
import threading
import traceback
import logging

# --- Logging Setup ---
logging.basicConfig(
    filename='ballistica_log.txt',
    filemode='a',
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# --- Sense HAT ---
sense = SenseHat()

# --- Globals ---
latest_temp = latest_pressure = latest_humidity = 0
initial_pitch = initial_roll = 0
heading_offset = 0
highlighted_field_index = 0

mode = "manual"
selected_bullet = BULLETS['.308_M80']
wind_speed = 0.0
wind_angle = 0.0
fields = ['bullet', 'wind_speed', 'wind_direction']
current_ballistics_info = {
    'time_of_flight': 'N/A', 'drop': 'N/A', 'mil_adjustment': 'N/A',
    'wind_drift': 'N/A', 'range': 'N/A', 'atmospheric': 'Temp: N/A | Pressure: N/A | Humidity: N/A'
}

# --- Filepaths ---
calibration_file = 'calibration.txt'
settings_file = 'settings.json'
wind_input_file = 'wind_input.json'

import atexit

# --- GPIO Setup ---
man_button = 17  # External manual trigger button
button1 = Button(21)  # KEY1: Select
button2 = Button(20)  # KEY2: Back
button3 = Button(16)  # KEY3: Toggle Mode

# --- SPI Setup (Waveshare screen) ---
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 50000
spi.mode = 0b00

# --- UART Setup with Safety ---
try:
    ser = serial.Serial("/dev/serial0", 9600, timeout=1)
except Exception as e:
    logging.warning("UART failed: %s", e)
    ser = None

# --- GPIO Pin Safety ---
try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(man_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
except RuntimeError:
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(man_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# --- Ensure GPIO cleanup on exit ---
atexit.register(GPIO.cleanup)

# --- Thread Safety Lock for Updates ---
update_lock = threading.Lock()
continuous_thread = None

# --- Proceed to Part 2 for GUI Layout & Field Highlighting ---
# PART 2/7 — GUI Layout, Splash Screen, Calibration Loading, First Launch Checks

# --- Initialize GUI Root ---
root = tk.Tk()
root.title("RAT 9 Ballistics Calculator")
main = ttk.Frame(root, padding="10")
main.pack()

# --- Style Configuration ---
style = ttk.Style()
style.configure("TLabel", font=("Helvetica", 10))
style.configure("Highlighted.TLabel", font=("Helvetica", 10, "bold"))

# --- Splash Screen ---
def show_splash_screen():
    splash = tk.Toplevel(root)
    splash.title("Initializing")
    ttk.Label(splash, text="RAT 9 Ballistics Calculator\nInitializing Sensors...", font=("Helvetica", 14)).pack(padx=20, pady=20)
    splash.update()
    time.sleep(1.5)
    splash.destroy()

# --- Safe Mode Warning ---
def safe_screen(message):
    safe_win = tk.Toplevel(root)
    safe_win.title("Safe Mode")
    ttk.Label(safe_win, text=message, font=("Helvetica", 12), wraplength=300).pack(padx=20, pady=20)
    ttk.Button(safe_win, text="Exit", command=root.destroy).pack(pady=10)
    safe_win.grab_set()
    root.wait_window(safe_win)

# --- First-Time Launch Instruction ---
def show_first_launch():
    welcome = tk.Toplevel(root)
    welcome.title("Welcome")
    ttk.Label(welcome, text="Welcome to RAT 9 Ballistics.\n\nIf this is your first time using the device,\nplease calibrate the pitch/roll and compass\nbefore beginning.\n\nUse the joystick to navigate.\nPress KEY1 to enter selections.", font=("Helvetica", 10), wraplength=300).pack(pady=10)
    ttk.Button(welcome, text="Got it!", command=welcome.destroy).pack(pady=5)

# --- Calibration Loaders ---
def load_calibration():
    global initial_pitch, initial_roll
    if os.path.exists(calibration_file):
        with open(calibration_file, 'r') as f:
            pitch, roll = f.read().strip().split(',')
            initial_pitch = float(pitch)
            initial_roll = float(roll)


def load_compass_heading():
    global heading_offset
    try:
        if os.path.exists('compass_cal.txt'):
            with open('compass_cal.txt', 'r') as f:
                heading_offset = float(f.read())
    except:
        heading_offset = 0
        logging.warning("Compass calibration not found. Defaulting heading offset to 0°.")

# --- Hardware Availability Check ---
def verify_hardware():
    try:
        _ = sense.get_temperature()
    except Exception as e:
        logging.critical("Sense HAT error: %s", e)
        safe_screen("Sense HAT not found.\n\nPlease check connection and reboot.")

# --- Launch Initialization ---
show_splash_screen()
verify_hardware()
load_calibration()
load_compass_heading()
if not os.path.exists(calibration_file) or not os.path.exists('compass_cal.txt'):
    show_first_launch()

# Proceed to Part 3 for UI layout and ballistics field rendering
# PART 3/7 — GUI Field Layout, Ballistics Info, Bullet Selector, Wind Inputs

# --- GUI Field + Widget Setup ---
bullet_choice = ttk.Combobox(main, values=list(BULLETS.keys()))
bullet_choice.set(".308_M80")
bullet_choice.grid(row=0, column=1, sticky='w')

field_labels = {}
info_labels = {}
field_order = ['bullet', 'wind_speed', 'wind_direction']

# --- Bullet Field ---
ttk.Label(main, text="Bullet Type:").grid(row=0, column=0, sticky='e')
field_labels['bullet'] = ttk.Label(main, text=bullet_choice.get())
field_labels['bullet'].grid(row=0, column=2, sticky='w')

# --- Wind Speed Field ---
ttk.Label(main, text="Wind Speed (m/s):").grid(row=1, column=0, sticky='e')
field_labels['wind_speed'] = ttk.Label(main, text=f"{wind_speed:.2f}")
field_labels['wind_speed'].grid(row=1, column=1, sticky='w')

# --- Wind Direction Field ---
ttk.Label(main, text="Wind Angle (deg):").grid(row=2, column=0, sticky='e')
field_labels['wind_direction'] = ttk.Label(main, text=f"{wind_angle:.2f}")
field_labels['wind_direction'].grid(row=2, column=1, sticky='w')

# --- Ballistics Output Labels ---
output_keys = ['time_of_flight', 'drop', 'mil_adjustment', 'wind_drift', 'range', 'atmospheric']
for i, key in enumerate(output_keys, start=3):
    ttk.Label(main, text=key.replace('_', ' ').title() + ":").grid(row=i, column=0, sticky='e')
    info_labels[key] = ttk.Label(main, text="...")
    info_labels[key].grid(row=i, column=1, columnspan=2, sticky='w')

# --- Progress Label for status messages ---
progress_label = ttk.Label(main, text="")
progress_label.grid(row=10, column=0, columnspan=3, pady=(10, 0))

# --- Function to Refresh Output Info Display ---
def update_display():
    field_labels['wind_speed'].config(text=f"{wind_speed:.2f}")
    field_labels['wind_direction'].config(text=f"{wind_angle:.2f}")
    field_labels['bullet'].config(text=bullet_choice.get())
    for key in info_labels:
        info_labels[key].config(text=current_ballistics_info.get(key, '...'))

# --- Callback for Bullet Change ---
def on_bullet_select(event=None):
    global selected_bullet
    label = bullet_choice.get()
    if label in BULLETS:
        selected_bullet = BULLETS[label]
        update_display()
        save_settings()

bullet_choice.bind("<<ComboboxSelected>>", on_bullet_select)

# --- Proceed to Part 4: Joystick Navigation, Highlighting, and Field Modification ---

# PATCHED PART 4 — Joystick Navigation + Modify Mode Control

# --- Highlight Logic ---
modify_mode = False

def highlight_field():
    for i, key in enumerate(field_order):
        label = field_labels[key] if key != 'bullet' else bullet_choice
        style = ("Helvetica", 12, "bold") if i == highlighted_field_index else ("Helvetica", 10)
        fg = "blue" if i == highlighted_field_index and modify_mode else "green" if i == highlighted_field_index else "black"
        label.config(font=style, foreground=fg)

# --- Joystick Pins ---
joystick_up_pin = 5
joystick_down_pin = 6
joystick_left_pin = 13
joystick_right_pin = 19

GPIO.setup(joystick_up_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(joystick_down_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(joystick_left_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(joystick_right_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# --- Joystick Navigation Logic ---
def joystick_up(channel=None):
    if modify_mode:
        modify_selected_field("up")
    else:
        global highlighted_field_index
        highlighted_field_index = (highlighted_field_index - 1) % len(field_order)
        highlight_field()
update_display()
root.mainloop()
GPIO.cleanup()

def joystick_down(channel=None):
    if modify_mode:
        modify_selected_field("down")
    else:
        global highlighted_field_index
        highlighted_field_index = (highlighted_field_index + 1) % len(field_order)
        highlight_field()

def joystick_left(channel=None):
    if modify_mode:
        modify_selected_field("left")

def joystick_right(channel=None):
    if modify_mode:
        modify_selected_field("right")
      
joystick_pins = [joystick_up_pin, joystick_down_pin, joystick_left_pin, joystick_right_pin]

def debounce_wrapper(func):
    def wrapper(channel):
        GPIO.remove_event_detect(channel)
        func(channel)
        GPIO.add_event_detect(channel, GPIO.FALLING, callback=wrapper, bouncetime=200)
    return wrapper

GPIO.add_event_detect(joystick_up_pin, GPIO.FALLING, callback=debounce_wrapper(joystick_up), bouncetime=200)
GPIO.add_event_detect(joystick_down_pin, GPIO.FALLING, callback=debounce_wrapper(joystick_down), bouncetime=200)
GPIO.add_event_detect(joystick_left_pin, GPIO.FALLING, callback=debounce_wrapper(joystick_left), bouncetime=200)
GPIO.add_event_detect(joystick_right_pin, GPIO.FALLING, callback=debounce_wrapper(joystick_right), bouncetime=200)


# --- Field Modifier ---
def modify_selected_field(direction):
    global wind_speed, wind_angle
    field = field_order[highlighted_field_index]
    small_step = 0.1
    large_step = 1.0
    small_angle = 5
    large_angle = 15

    if field == 'wind_speed':
        step = large_step if direction in ("up", "down") else small_step
        delta = step if direction in ("right", "down") else -step
        wind_speed = max(0.0, wind_speed + delta)
    elif field == 'wind_direction':
        step = large_angle if direction in ("up", "down") else small_angle
        delta = step if direction in ("right", "down") else -step
        wind_angle = (wind_angle + delta) % 360

    save_settings()
    update_display()

# --- Key Bindings ---
def key1_pressed():
    global modify_mode
    field = field_order[highlighted_field_index]
    if field in ["wind_speed", "wind_direction"]:
        modify_mode = not modify_mode
        highlight_field()
    elif field == "bullet":
        on_bullet_select()

def key2_pressed():
    global modify_mode
    if modify_mode:
        modify_mode = False
        highlight_field()
    else:
        update_display()

def key3_pressed():
    global mode
    mode = 'manual' if mode == 'continuous' else 'continuous'
    update_display()
    if mode == 'continuous':
        schedule_continuous_updates()
      
button1.when_pressed = key1_pressed
button2.when_pressed = key2_pressed
button3.when_pressed = key3_pressed

highlight_field()
# PART 5/7 — Ballistics Calculation, Sensor Sync, Rangefinding, Environmental Averaging

# --- Atmospheric Sensor Sampling (3 reads every 30s) ---
def get_average_atmospheric_conditions():
    global latest_temp, latest_pressure, latest_humidity
    temp_sum = pressure_sum = humidity_sum = 0
    for _ in range(3):
        temp_sum += sense.get_temperature()
        pressure_sum += sense.get_pressure()
        humidity_sum += sense.get_humidity()
        time.sleep(10)
    latest_temp = temp_sum / 3
    latest_pressure = pressure_sum / 3
    latest_humidity = humidity_sum / 3

# --- Rangefinder Reading with UART ---
def get_range():
    try:
        if ser and ser.in_waiting > 0:
            ser.flushInput()
            raw = ser.readline().decode('utf-8').strip()
            if raw.isdigit():
                val = int(raw)
                if 0 < val < 3000:
                    return val
    except Exception as e:
        logging.error("Range read failed: %s", e)
    return 0

# --- Calculate Elevation Using Pitch and Range ---
def calculate_elevations(range_distance):
    pitch = sense.get_orientation_degrees()['pitch'] - initial_pitch
    return range_distance * math.tan(math.radians(pitch)), 0

# --- Calculate Relative Wind Angle ---
def get_relative_wind_angle():
   heading = (sense.get_compass() - heading_offset + 360) % 360
   return (wind_angle - heading + 360) % 360


# --- Ballistics Calculation Trigger ---
def update_ballistics():
    get_average_atmospheric_conditions()
    range_to_target = get_range()
    target_elevation, shooter_elevation = calculate_elevations(range_to_target)
    rel_wind_angle = get_relative_wind_angle()
    try:
        result = simulate_trajectory(
            bullet=selected_bullet,
            range_m=range_to_target,
            wind_speed_mps=wind_speed,
            wind_angle_deg=rel_wind_angle,
            shooter_elev_m=shooter_elevation,
            target_elev_m=target_elevation,
            temp_c=latest_temp,
            pressure_hpa=latest_pressure,
            humidity=latest_humidity,
            pitch_deg=sense.get_orientation_degrees()['pitch'] - initial_pitch,
            roll_deg=sense.get_orientation_degrees()['roll'] - initial_roll
        )
        current_ballistics_info['time_of_flight'] = f"{result['time_of_flight']:.2f} s"
        current_ballistics_info['drop'] = f"{result['drop_m']:.2f} m / {result['drop_in']:.2f} in"
        current_ballistics_info['mil_adjustment'] = f"{result['mil_adjustment']:.2f} mil"
        current_ballistics_info['wind_drift'] = f"{result['wind_drift_mil']:.2f} mil"
        current_ballistics_info['range'] = f"{range_to_target} m"
        current_ballistics_info['atmospheric'] = f"Temp: {latest_temp:.1f}°C | Pressure: {latest_pressure:.1f} hPa | Hum: {latest_humidity:.1f}%"
        update_display()
    except Exception as e:
        logging.error("Ballistics calculation failed: %s", traceback.format_exc())
        current_ballistics_info['time_of_flight'] = "ERROR"
        current_ballistics_info['drop'] = "ERROR"
        current_ballistics_info['mil_adjustment'] = "ERROR"
        current_ballistics_info['wind_drift'] = "ERROR"
        current_ballistics_info['range'] = "0"
        current_ballistics_info['atmospheric'] = "Sensor Error"
        update_display()

# --- Proceed to Part 6: Continuous Mode Threading, Manual Trigger Handling, and Progress Indicators ---
# PART 6/7 — Manual Trigger, Continuous Mode, Progress Feedback, Sync

# --- Manual Trigger Button ---
def handle_manual_trigger(channel=None):
    if mode == "manual":
        logging.info("Manual trigger pressed. Starting calculation...")
        progress_label.config(text="Calculating...")
        root.update_idletasks()
        update_ballistics()
        progress_label.config(text="Update Complete")
        root.after(1500, lambda: progress_label.config(text=""))

GPIO.add_event_detect(man_button, GPIO.FALLING, callback=handle_manual_trigger, bouncetime=300)

# --- Continuous Update Loop ---
def schedule_continuous_updates():
    global continuous_thread

    def loop():
        logging.info("Continuous mode thread started.")
        while mode == "continuous":
            if update_lock.acquire(blocking=False):
                try:
                    progress_label.config(text="Updating...")
                    root.update_idletasks()
                    logging.info("Continuous update triggered.")
                    root.update()
                    update_ballistics()
                    progress_label.config(text="Update Complete")
                    root.after(1500, lambda: progress_label.config(text=""))
                finally:
                    update_lock.release()
            time.sleep(3)

    if continuous_thread is None or not continuous_thread.is_alive():
        continuous_thread = threading.Thread(target=loop, daemon=True)
        continuous_thread.start()

# --- GUI Exit Cleanup ---
def graceful_exit():
    GPIO.cleanup()
    root.destroy()

# --- Final GUI Bindings ---
root.protocol("WM_DELETE_WINDOW", graceful_exit)

# --- Highlight Active Field and Sync Display at Launch ---
highlight_field()
update_display()

# --- Proceed to Part 7: Final Touches, Calibration Mode, Wind Menu, Splash, Logging, and Export ---
# PART 7/7 — Calibration UI, Wind Menu, Final Logging, Wrap-up

# --- Wind Settings Window ---
def open_wind_settings():
    win = tk.Toplevel(root)
    win.title("Wind Settings")

    tk.Label(win, text="Wind Speed (m/s):").grid(row=0, column=0, padx=10, pady=5, sticky='e')
    speed_var = tk.DoubleVar(value=wind_speed)
    speed_entry = tk.Entry(win, textvariable=speed_var)
    speed_entry.grid(row=0, column=1, padx=10, pady=5)

    tk.Label(win, text="Wind Direction (° or cardinal):").grid(row=1, column=0, padx=10, pady=5, sticky='e')
    angle_var = tk.StringVar(value=str(wind_angle))
    angle_entry = tk.Entry(win, textvariable=angle_var)
    angle_entry.grid(row=1, column=1, padx=10, pady=5)

    def save_wind():
        global wind_speed, wind_angle
        try:
            wind_speed = float(speed_var.get())
            try:
                wind_angle = float(angle_var.get())
            except ValueError:
                cardinal = angle_var.get().strip().upper()
                wind_angle = {
                    'N': 0, 'NE': 45, 'E': 90, 'SE': 135, 'S': 180, 'SW': 225, 'W': 270, 'NW': 315
                }.get(cardinal, 0)
            save_settings()
            update_display()
            win.destroy()
        except Exception as e:
            logging.error("Failed to save wind input: %s", e)
            tk.messagebox.showerror("Input Error", str(e))

    tk.Button(win, text="Save", command=save_wind).grid(row=2, column=0, columnspan=2, pady=10)
    win.transient(root)
    win.grab_set()
    win.wait_window()


# --- Calibration GUI ---
def calibrate():
    cal_win = tk.Toplevel(root)
    cal_win.title("Calibrate Sensors")

    tk.Label(cal_win, text="Place device on flat surface and press 'Calibrate'.").pack(pady=10)

    def perform_pitch_roll_cal():
        global initial_pitch, initial_roll
        orientation = sense.get_orientation_degrees()
        initial_pitch = orientation['pitch']
        initial_roll = orientation['roll']
        with open(calibration_file, 'w') as f:
            f.write(f"{initial_pitch},{initial_roll}")
        logging.info("Pitch/Roll calibration saved: pitch=%.2f, roll=%.2f", initial_pitch, initial_roll)
        tk.Label(cal_win, text="Now point the rifle NORTH and press 'Zero Compass'.").pack(pady=10)
        tk.Button(cal_win, text="Zero Compass", command=perform_compass_cal).pack(pady=5)

    def perform_compass_cal():
        heading = sense.get_compass()
        with open("compass_cal.txt", 'w') as f:
            f.write(str(heading))
        logging.info("Compass heading zeroed at: %.2f°", heading)
        tk.Label(cal_win, text="Calibration complete!", fg="green").pack(pady=5)
        tk.Button(cal_win, text="Done", command=cal_win.destroy).pack(pady=10)

    tk.Button(cal_win, text="Calibrate", command=perform_pitch_roll_cal).pack(pady=10)

# --- Final Main Menu Additions (Optional Buttons) ---
ttk.Button(main, text="Wind Settings", command=open_wind_settings).grid(row=11, column=0, columnspan=2, pady=5)
ttk.Button(main, text="Calibrate Device", command=calibrate).grid(row=12, column=0, columnspan=2, pady=5)

# --- Final Launch Setup ---
logging.info("RAT 9 Ballistics GUI fully initialized and ready.")
highlight_field()
update_display()
root.mainloop()
GPIO.cleanup()
