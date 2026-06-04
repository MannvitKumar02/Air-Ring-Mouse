"""
╔══════════════════════════════════════════════════════════════════════════════╗
║              MASTER IMU GESTURE DATA COLLECTOR (OPTIMIZED)                   ║
║  Collects ALL gesture types from a BLE IMU ring into clean labeled CSVs.     ║
║                                                                              ║
║  ALL gestures stop manually — press S to stop recording.                     ║
║                                                                              ║
║  GESTURES:                                                                   ║
║    pointer_move, idle                                                        ║
║    left_click, right_click, double_click                                     ║
║    scroll_up, scroll_down, scroll_left, scroll_right                         ║
║    swipe_left, swipe_right, swipe_up, swipe_down                             ║
║                                                                              ║
║  HOTKEYS:  A = Start   S = Stop                                              ║
║                                                                              ║
║  OPTIMIZED: Smooth counter updates with non-blocking filter computation      ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

import asyncio
import threading
import time
import csv
import os
import math
import tkinter as tk
from tkinter import ttk
from collections import deque
from bleak import BleakClient, BleakScanner

# ─────────────────────────────────────────────
# HARDWARE CONFIG
# ─────────────────────────────────────────────
DEFAULT_BLE_ADDRESS = "E4:B3:23:F8:3B:5A"
CHARACTERISTIC_UUID = "0000FEF4-0000-1000-8000-00805F9B34FB"
ACCEL_SCALE         = 16384.0   # MPU-6050 ±2g
GYRO_SCALE          = 131.0     # MPU-6050 ±250°/s

# ─────────────────────────────────────────────
# COLLECTION CONFIG
# ─────────────────────────────────────────────
FLUSH_EVERY_N_ROWS     = 25
FLUSH_EVERY_SECONDS    = 0.5
WARMUP_SAMPLES_TO_SKIP = 5
ALPHA                  = 0.96
DEADZONE_DEG           = 0.4

# ─────────────────────────────────────────────
# GESTURE DEFINITIONS  (label, mode, hint)
# ─────────────────────────────────────────────
GESTURES = [
    ("pointer_move", "continuous", "Move hand freely in all directions."),
    ("idle",         "continuous", "Hold hand completely still."),
    ("left_click",   "discrete",   "One quick downward tap/flick."),
    ("right_click",  "discrete",   "One slow deliberate tap or inward twist."),
    ("double_click", "discrete",   "Two rapid taps in quick succession."),
    ("scroll_up",    "discrete",   "Tilt wrist upward / roll fingers upward."),
    ("scroll_down",  "discrete",   "Tilt wrist downward / roll fingers downward."),
    ("scroll_left",  "discrete",   "Tilt wrist to the left."),
    ("scroll_right", "discrete",   "Tilt wrist to the right."),
    ("swipe_left",   "discrete",   "Sweep hand quickly to the LEFT."),
    ("swipe_right",  "discrete",   "Sweep hand quickly to the RIGHT."),
    ("swipe_up",     "discrete",   "Sweep hand quickly UPWARD."),
    ("swipe_down",   "discrete",   "Sweep hand quickly DOWNWARD."),
]

GESTURE_LABELS = [g[0] for g in GESTURES]
GESTURE_META   = {g[0]: {"mode": g[1], "hint": g[2]} for g in GESTURES}

# ─────────────────────────────────────────────
# CSV COLUMNS
# ─────────────────────────────────────────────
COLUMNS = [
    "t_ms",
    "Accel_X", "Accel_Y", "Accel_Z",
    "Gyro_X",  "Gyro_Y",  "Gyro_Z",
    "pitch_deg", "roll_deg",
    "delta_pitch", "delta_roll",
    "is_still",
    "label",
    "session_id",
    "collection_mode",
]

# ─────────────────────────────────────────────
# GLOBAL STATE
# ─────────────────────────────────────────────
ble_client  = None
ble_thread  = None
ble_loop    = None
collecting  = False

csv_filename    = ""
buffer_rows     = []
buffer_lock     = threading.Lock()
rows_written    = 0
last_flush_time = 0.0
warmup_left     = 0

_csv_file   = None
_csv_writer = None

device_map  = {}
scanning    = False

# Live display state - OPTIMIZED for smooth updates
latest_values_lock  = threading.Lock()
latest_values       = None
total_rows_captured = 0
recv_times          = deque(maxlen=80)

# Filter state (computed in background thread to not block notifications)
cf_pitch   = 0.0
cf_roll    = 0.0
cf_last_t  = None
prev_pitch = 0.0
prev_roll  = 0.0

# Background processing queue (OPTIMIZATION: process filters async)
filter_queue = deque(maxlen=500)
filter_queue_lock = threading.Lock()
filter_thread = None
filter_running = False


# ═════════════════════════════════════════════
# CSV / BUFFER
# ═════════════════════════════════════════════
def open_csv_for_append(path: str):
    global _csv_file, _csv_writer
    os.makedirs(os.path.dirname(path), exist_ok=True)
    file_exists = os.path.exists(path) and os.path.getsize(path) > 0
    _csv_file   = open(path, "a", newline="", encoding="utf-8")
    _csv_writer = csv.DictWriter(_csv_file, fieldnames=COLUMNS)
    if not file_exists:
        _csv_writer.writeheader()
        _csv_file.flush()


def close_csv():
    global _csv_file, _csv_writer
    try:
        if _csv_file:
            _csv_file.flush()
            _csv_file.close()
    finally:
        _csv_file   = None
        _csv_writer = None


def flush_buffer(force: bool = False):
    global buffer_rows, rows_written, last_flush_time
    if _csv_writer is None:
        return
    now = time.time()
    with buffer_lock:
        if not buffer_rows:
            return
        if (not force
                and len(buffer_rows) < FLUSH_EVERY_N_ROWS
                and (now - last_flush_time) < FLUSH_EVERY_SECONDS):
            return
        rows_to_write = buffer_rows
        buffer_rows   = []
    for r in rows_to_write:
        _csv_writer.writerow(r)
    rows_written    += len(rows_to_write)
    last_flush_time  = now
    _csv_file.flush()


# ═════════════════════════════════════════════
# COMPLEMENTARY FILTER (Non-blocking background processing)
# ═════════════════════════════════════════════
def reset_filter():
    global cf_pitch, cf_roll, cf_last_t, prev_pitch, prev_roll
    cf_pitch = cf_roll = prev_pitch = prev_roll = 0.0
    cf_last_t = None


def compute_filter_values(ax, ay, az, gx, gy, gz, now):
    """Pure function - computes filter values without side effects"""
    global cf_pitch, cf_roll, cf_last_t, prev_pitch, prev_roll

    ax_g  = ax / ACCEL_SCALE
    ay_g  = ay / ACCEL_SCALE
    az_g  = az / ACCEL_SCALE
    gx_ds = gx / GYRO_SCALE
    gy_ds = gy / GYRO_SCALE

    accel_pitch = math.degrees(math.atan2(ay_g, math.sqrt(ax_g**2 + az_g**2)))
    accel_roll  = math.degrees(math.atan2(-ax_g, az_g))

    if cf_last_t is None:
        cf_pitch = accel_pitch
        cf_roll  = accel_roll
        cf_last_t = now
        prev_pitch = cf_pitch
        prev_roll  = cf_roll
        return cf_pitch, cf_roll, 0.0, 0.0, 1

    dt = now - cf_last_t
    cf_last_t = now
    if dt <= 0 or dt > 0.5:
        return cf_pitch, cf_roll, 0.0, 0.0, 1

    cf_pitch = ALPHA * (cf_pitch + gx_ds * dt) + (1 - ALPHA) * accel_pitch
    cf_roll  = ALPHA * (cf_roll  + gy_ds * dt) + (1 - ALPHA) * accel_roll

    dp = cf_pitch - prev_pitch
    dr = cf_roll  - prev_roll
    prev_pitch = cf_pitch
    prev_roll  = cf_roll

    is_still = int(abs(dp) < DEADZONE_DEG and abs(dr) < DEADZONE_DEG)
    return round(cf_pitch, 4), round(cf_roll, 4), round(dp, 4), round(dr, 4), is_still


def filter_processor_thread():
    """Background thread that processes filter calculations without blocking notifications"""
    global filter_running
    
    while filter_running:
        try:
            with filter_queue_lock:
                if not filter_queue:
                    time.sleep(0.001)  # Short sleep when queue empty
                    continue
                
                # Process batch of items
                items_to_process = []
                while filter_queue and len(items_to_process) < 10:
                    items_to_process.append(filter_queue.popleft())
            
            # Process outside lock for better concurrency
            for item in items_to_process:
                row_id, ax, ay, az, gx, gy, gz, now, label, session, mode = item
                
                # Compute filter values
                pitch, roll, dp, dr, still = compute_filter_values(ax, ay, az, gx, gy, gz, now)
                
                # Build complete row
                row = {
                    "t_ms":            int(now * 1000),
                    "Accel_X":         ax,
                    "Accel_Y":         ay,
                    "Accel_Z":         az,
                    "Gyro_X":          gx,
                    "Gyro_Y":          gy,
                    "Gyro_Z":          gz,
                    "pitch_deg":       pitch,
                    "roll_deg":        roll,
                    "delta_pitch":     dp,
                    "delta_roll":      dr,
                    "is_still":        still,
                    "label":           label,
                    "session_id":      session,
                    "collection_mode": mode,
                }
                
                # Add to buffer
                with buffer_lock:
                    buffer_rows.append(row)
                
                # Update latest values for display (only for most recent)
                if item == items_to_process[-1]:
                    with latest_values_lock:
                        latest_values = (ax, ay, az, gx, gy, gz, pitch, roll, dp, dr, still)
                        
        except Exception as e:
            print(f"Filter processor error: {e}")
            time.sleep(0.01)


# ═════════════════════════════════════════════
# THREAD-SAFE UI HELPERS
# ═════════════════════════════════════════════
def set_status(msg: str):
    root.after(0, lambda: status_var.set(msg))

def set_scan_button(enabled: bool):
    root.after(0, lambda: scan_btn.config(state="normal" if enabled else "disabled"))

def set_record_buttons(is_recording: bool):
    def _u():
        start_btn.config(state="disabled" if is_recording else "normal")
        stop_btn.config(state="normal"    if is_recording else "disabled")
    root.after(0, _u)

def update_device_dropdown(options):
    def _u():
        device_combo["values"] = options
        if options:
            device_combo.current(0)
    root.after(0, _u)

def get_selected_address():
    sel = device_choice_var.get().strip()
    if sel in device_map:
        return device_map[sel]
    manual = address_var.get().strip()
    return manual if manual else DEFAULT_BLE_ADDRESS


# ═════════════════════════════════════════════
# BLE NOTIFICATION HANDLER (OPTIMIZED - minimal processing)
# ═════════════════════════════════════════════
def notification_handler(sender, data):
    """
    OPTIMIZED: Immediately increment counter, queue heavy processing for background thread.
    This ensures smooth, non-blocking counter updates.
    """
    global warmup_left, total_rows_captured

    try:
        parts = data.decode(errors="ignore").strip().split(",")
        if len(parts) < 6:
            return

        values = list(map(int, parts[:6]))
        
        # Skip warmup samples
        if warmup_left > 0:
            warmup_left -= 1
            return

        # CRITICAL: Capture timestamp and increment counter IMMEDIATELY
        now = time.time()
        ax, ay, az, gx, gy, gz = values
        
        # Increment counter FIRST (ensures smooth updates)
        with latest_values_lock:
            total_rows_captured += 1
            recv_times.append(now)
        
        # Queue processing for background thread (non-blocking)
        label = gesture_var.get().strip()
        session = session_id_var.get().strip()
        mode = GESTURE_META.get(label, {}).get("mode", "discrete")
        
        with filter_queue_lock:
            filter_queue.append((
                total_rows_captured,
                ax, ay, az, gx, gy, gz,
                now, label, session, mode
            ))

    except Exception as e:
        print("Decode error:", e)


# ═════════════════════════════════════════════
# BLE ASYNC TASK
# ═════════════════════════════════════════════
async def ble_task():
    global ble_client, collecting

    address  = get_selected_address()
    notified = False

    try:
        async with BleakClient(address) as client:
            ble_client = client
            set_status(f"Connected: {address}")
            try:
                await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
                notified = True
            except Exception as e:
                set_status(f"Start notify failed: {e}")
                collecting = False
                set_record_buttons(False)
                return

            while collecting:
                flush_buffer(force=False)
                await asyncio.sleep(0.05)

            await asyncio.sleep(0.15)

            if notified:
                try:
                    await client.stop_notify(CHARACTERISTIC_UUID)
                except Exception:
                    pass  # Device already disconnected — safe to ignore
            try:
                await client.disconnect()
            except Exception:
                pass

    except Exception as e:
        set_status(f"BLE Error: {e}")
    finally:
        ble_client = None
        set_record_buttons(False)


def run_ble_loop():
    global ble_loop
    ble_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(ble_loop)
    ble_loop.run_until_complete(ble_task())


# ═════════════════════════════════════════════
# BLE SCAN
# ═════════════════════════════════════════════
def scan_devices():
    global scanning
    if scanning:
        return

    scanning = True
    set_scan_button(False)
    set_status("Scanning for BLE devices... (please wait)")

    def _scan_thread():
        global device_map, scanning
        try:
            devices = asyncio.run(BleakScanner.discover(timeout=6.0))
            temp_map = {}
            options = []

            for d in devices:
                name = (d.name or "Unknown").strip()
                addr = d.address
                label = f"{name} ({addr})"
                temp_map[label] = addr
                options.append(label)

            options.sort(key=lambda x: x.lower())
            device_map = temp_map

            if options:
                update_device_dropdown(options)
                set_status(f"Found {len(options)} device(s). Select one and press 'A' to start.")
            else:
                set_status("No BLE devices found. Make sure device is ON & advertising.")

        except Exception as e:
            set_status(f"Scan failed: {e}")
        finally:
            scanning = False
            set_scan_button(True)

    threading.Thread(target=_scan_thread, daemon=True).start()


# ═════════════════════════════════════════════
# START COLLECTION
# ═════════════════════════════════════════════
def build_save_path(gesture: str, sess: str, file_no: str) -> str:
    desktop_path = os.path.join(os.path.expanduser("~"), "Desktop")
    return os.path.join(
        desktop_path,
        "IMU_Gesture_Data",
        gesture,
        sess,
        f"sample{file_no}.csv"
    )


def start_ble_collection(event=None):
    global collecting, ble_thread, csv_filename, rows_written, last_flush_time, warmup_left
    global total_rows_captured, latest_values, filter_running, filter_thread

    if collecting:
        return "break"

    gesture = gesture_var.get().strip()
    file_no = file_no_var.get().strip()
    sess = session_id_var.get().strip()

    if not gesture:
        set_status("Enter a valid gesture (e.g., up/down/left/right).")
        return "break"
    if not file_no.isdigit():
        set_status("Enter a valid file number (digits).")
        return "break"
    if not sess:
        set_status("Enter a session_id (e.g., day1_me, person2, etc.).")
        return "break"

    address = get_selected_address()
    if not address:
        set_status("No device selected. Click Scan and choose a device.")
        return "break"

    csv_filename = build_save_path(gesture, sess, file_no)

    # Reset all state
    rows_written = 0
    last_flush_time = 0.0
    warmup_left = WARMUP_SAMPLES_TO_SKIP
    reset_filter()

    with latest_values_lock:
        total_rows_captured = 0
        latest_values = None
        recv_times.clear()

    with filter_queue_lock:
        filter_queue.clear()

    close_csv()
    open_csv_for_append(csv_filename)

    # Start background filter processor
    filter_running = True
    filter_thread = threading.Thread(target=filter_processor_thread, daemon=True)
    filter_thread.start()

    # Start BLE collection
    collecting = True
    set_record_buttons(True)
    set_status(f"● RECORDING (S to stop) | '{gesture}' | session '{sess}' | sample {file_no}\nSaved to: {csv_filename}")

    ble_thread = threading.Thread(target=run_ble_loop, daemon=True)
    ble_thread.start()

    return "break"


# ═════════════════════════════════════════════
# STOP COLLECTION
# ═════════════════════════════════════════════
def stop_ble_collection(event=None):
    global collecting, filter_running

    if not collecting:
        return "break"

    collecting = False

    # Step 1: wait 100ms for in-flight BLE packets to land
    def _wait_for_packets():
        global filter_running
        filter_running = False          # signal background thread to stop
        root.after(300, _finish_stop)   # Step 2: wait 300ms for filter queue to drain

    def _finish_stop():
        flush_buffer(force=True)
        close_csv()

        with latest_values_lock:
            final_count = total_rows_captured

        samples_var.set(str(final_count))

        file_no = file_no_var.get().strip()
        if file_no.isdigit():
            file_no_var.set(str(int(file_no) + 1))

        set_status(
            f"✓ Saved: {csv_filename}\n"
            f"Total samples: {final_count} | File number incremented → ready for next sample."
        )
        set_record_buttons(False)

    root.after(100, _wait_for_packets)
    return "break"


# ═════════════════════════════════════════════
# LIVE PANEL UPDATER
# ═════════════════════════════════════════════
def update_live_panel():
    if collecting:
        with latest_values_lock:
            count = total_rows_captured
            lv    = latest_values
            times = list(recv_times)

        pps = 0.0
        if len(times) >= 2:
            dt = times[-1] - times[0]
            if dt > 0:
                pps = (len(times) - 1) / dt

        samples_var.set(str(count))
        pps_var.set(f"{pps:.1f}")
        rec_var.set("● REC")
        rec_label.config(foreground="red")

        if lv:
            ax, ay, az, gx, gy, gz, pitch, roll, dp, dr, still = lv
            imu_var.set(
                f"Ax:{ax:>6}  Ay:{ay:>6}  Az:{az:>6}   "
                f"Gx:{gx:>6}  Gy:{gy:>6}  Gz:{gz:>6}"
            )
            orient_var.set(
                f"Pitch:{pitch:>7.2f}°   Roll:{roll:>7.2f}°   "
                f"ΔP:{dp:>6.3f}   ΔR:{dr:>6.3f}   Still:{still}"
            )
        else:
            imu_var.set("Waiting for packets...")
            orient_var.set("")
    else:
        rec_var.set("IDLE")
        rec_label.config(foreground="gray")

    root.after(150, update_live_panel)


# ═════════════════════════════════════════════
# GUI
# ═════════════════════════════════════════════
root = tk.Tk()
root.title("IMU Gesture Data Collector - OPTIMIZED")
root.resizable(False, False)

mf = ttk.Frame(root, padding="12")
mf.grid(row=0, column=0, sticky="nsew")

# ── Header ───────────────────────────────────
hdr = ttk.Frame(mf)
hdr.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 6))
ttk.Label(hdr, text="IMU Gesture Data Collector",
          font=("Segoe UI", 12, "bold")).grid(row=0, column=0, sticky="w")
rec_var   = tk.StringVar(value="IDLE")
rec_label = ttk.Label(hdr, textvariable=rec_var, font=("Segoe UI", 11, "bold"), foreground="gray")
rec_label.grid(row=0, column=1, sticky="e", padx=(12, 0))
hdr.columnconfigure(0, weight=1)

ttk.Label(mf, text="A = Start    S = Stop",
          font=("Segoe UI", 8), foreground="gray").grid(
    row=1, column=0, columnspan=3, sticky="w", pady=(0, 6))

# ── BLE Device ───────────────────────────────
ttk.Separator(mf, orient="horizontal").grid(row=2, column=0, columnspan=3, sticky="ew", pady=4)
ttk.Label(mf, text="BLE Device:").grid(row=3, column=0, sticky="w")
device_choice_var = tk.StringVar(value="")
device_combo = ttk.Combobox(mf, width=36, textvariable=device_choice_var, state="readonly")
device_combo.grid(row=3, column=1, sticky="ew", padx=(4, 4))
scan_btn = ttk.Button(mf, text="Scan", command=scan_devices, width=8)
scan_btn.grid(row=3, column=2)

ttk.Label(mf, text="Manual MAC:").grid(row=4, column=0, sticky="w", pady=(2, 0))
address_var = tk.StringVar(value=DEFAULT_BLE_ADDRESS)
ttk.Entry(mf, width=24, textvariable=address_var).grid(row=4, column=1, sticky="w", pady=(2, 0))

# ── Gesture Selector ─────────────────────────
ttk.Separator(mf, orient="horizontal").grid(row=5, column=0, columnspan=3, sticky="ew", pady=6)
ttk.Label(mf, text="Gesture Label:", font=("Segoe UI", 9, "bold")).grid(row=6, column=0, sticky="w")
gesture_var   = tk.StringVar(value=GESTURE_LABELS[0])
gesture_combo = ttk.Combobox(mf, width=22, textvariable=gesture_var,
                              values=GESTURE_LABELS, state="readonly")
gesture_combo.grid(row=6, column=1, sticky="w")
gesture_combo.current(0)

mode_var = tk.StringVar(value="")
hint_var = tk.StringVar(value="")

def on_gesture_change(*_):
    lbl  = gesture_var.get()
    meta = GESTURE_META.get(lbl, {})
    m    = meta.get("mode", "")
    h    = meta.get("hint", "")
    mode_var.set(f"Mode: {m.upper()}  (manual stop)")
    hint_var.set(f"Hint: {h}")

gesture_var.trace_add("write", on_gesture_change)
on_gesture_change()

ttk.Label(mf, textvariable=mode_var, foreground="navy",
          font=("Segoe UI", 8, "italic")).grid(row=7, column=0, columnspan=3, sticky="w", pady=(2, 0))
ttk.Label(mf, textvariable=hint_var, foreground="#555",
          font=("Segoe UI", 8), wraplength=520).grid(row=8, column=0, columnspan=3, sticky="w")

# Quick-select buttons
btn_frame = ttk.LabelFrame(mf, text="Quick Select Gesture", padding="6")
btn_frame.grid(row=9, column=0, columnspan=3, sticky="ew", pady=(6, 2))
for idx, lbl in enumerate(GESTURE_LABELS):
    short = lbl.replace("_", " ").title()
    ttk.Button(btn_frame, text=short, width=14,
               command=lambda l=lbl: gesture_var.set(l)).grid(
        row=idx // 7, column=idx % 7, padx=2, pady=2)

# ── Session / File ────────────────────────────
ttk.Separator(mf, orient="horizontal").grid(row=10, column=0, columnspan=3, sticky="ew", pady=6)
ttk.Label(mf, text="Session ID:").grid(row=11, column=0, sticky="w")
session_id_var = tk.StringVar(value="day1_me")
ttk.Entry(mf, width=22, textvariable=session_id_var).grid(row=11, column=1, sticky="w")

ttk.Label(mf, text="File Number:").grid(row=12, column=0, sticky="w", pady=(2, 0))
file_no_var = tk.StringVar(value="1")
ttk.Entry(mf, width=22, textvariable=file_no_var).grid(row=12, column=1, sticky="w", pady=(2, 0))

# ── Start / Stop ──────────────────────────────
ctrl = ttk.Frame(mf)
ctrl.grid(row=13, column=0, columnspan=3, pady=10)
start_btn = ttk.Button(ctrl, text="▶  Start (A)", command=start_ble_collection, width=18)
start_btn.grid(row=0, column=0, padx=8)
stop_btn  = ttk.Button(ctrl, text="■  Stop  (S)", command=stop_ble_collection,
                        width=18, state="disabled")
stop_btn.grid(row=0, column=1, padx=8)

# ── Live Monitor ──────────────────────────────
live = ttk.LabelFrame(mf, text="Live Monitor", padding="8")
live.grid(row=14, column=0, columnspan=3, sticky="ew", pady=(2, 4))

samples_var = tk.StringVar(value="0")
pps_var     = tk.StringVar(value="0.0")
imu_var     = tk.StringVar(value="Not recording")
orient_var  = tk.StringVar(value="")

ttk.Label(live, text="Samples:").grid(row=0, column=0, sticky="w")
ttk.Label(live, textvariable=samples_var,
          font=("Segoe UI", 10, "bold")).grid(row=0, column=1, sticky="w", padx=6)
ttk.Label(live, text="Pkt/s:").grid(row=0, column=2, sticky="w", padx=(12, 0))
ttk.Label(live, textvariable=pps_var,
          font=("Segoe UI", 10, "bold")).grid(row=0, column=3, sticky="w", padx=6)

ttk.Label(live, text="IMU:").grid(row=1, column=0, sticky="w", pady=(4, 0))
ttk.Label(live, textvariable=imu_var,
          font=("Courier", 9)).grid(row=1, column=1, columnspan=3, sticky="w", pady=(4, 0))

ttk.Label(live, text="Orient:").grid(row=2, column=0, sticky="w", pady=(2, 0))
ttk.Label(live, textvariable=orient_var,
          font=("Courier", 9)).grid(row=2, column=1, columnspan=3, sticky="w", pady=(2, 0))

# ── Status ────────────────────────────────────
status_var = tk.StringVar(value="Ready. Scan → select device → pick gesture → 'A' to start.")
ttk.Label(mf, textvariable=status_var, wraplength=580,
          justify="left", foreground="#333").grid(
    row=15, column=0, columnspan=3, sticky="w", pady=(4, 0))

# ── Hotkeys ───────────────────────────────────
root.bind_all("<a>", start_ble_collection)
root.bind_all("<s>", stop_ble_collection)


def on_close():
    global collecting, filter_running
    collecting = False
    filter_running = False
    try:
        time.sleep(0.2)  # Let threads finish
        flush_buffer(force=True)
        close_csv()
    except Exception:
        pass
    root.destroy()


root.protocol("WM_DELETE_WINDOW", on_close)
root.after(150, update_live_panel)
root.mainloop()