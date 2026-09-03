import serial
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, RadioButtons, TextBox, CheckButtons
import matplotlib.animation as animation
from collections import deque
import time
import csv

# --- CONFIGURATION ---
SERIAL_PORT = 'COM7'  # Update this to your port!
BAUD_RATE = 115200
WINDOW_SIZE = 100
CSV_FILENAME = f"pid_data_{int(time.time())}.csv"

# Buffers for plotting
dist_data = deque([0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)
set_data = deque([0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)
srv_data = deque([0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)
kp_data = deque([0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)
ki_data = deque([0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)
kd_data = deque([0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)

# Initialize Serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2)
    ser.write(b"START\n")
    print(f"Connected! Logging data to: {CSV_FILENAME}")
except Exception as e:
    print(f"Connection Error: {e}")
    exit()

# Open CSV file and write header
csv_file = open(CSV_FILENAME, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(
    ["Timestamp", "Distance_CM", "Setpoint_CM", "Servo_Angle",
     "Kp_eff", "Ki_eff", "Kd_eff"]
)

# Setup Plot Subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 9))
# Extra bottom room so the widgets don't crowd the graphs.
plt.subplots_adjust(bottom=0.42, right=0.8, hspace=0.45)

line_dist, = ax1.plot(dist_data, label="Distance (cm)", color='blue', linewidth=1.5)
line_set, = ax1.plot(set_data, label="Setpoint (cm)", color='red', linestyle='--')
line_srv, = ax2.plot(srv_data, label="Servo Angle", color='green')

# Effective (scheduled) PID gains — kd on a secondary axis since it's a
# different magnitude than kp/ki.
line_kp, = ax3.plot(kp_data, label="Kp_eff", color='purple')
line_ki, = ax3.plot(ki_data, label="Ki_eff", color='orange')
ax3b = ax3.twinx()
line_kd, = ax3b.plot(kd_data, label="Kd_eff", color='brown', linestyle='-.')

ax1.set_ylim(0, 40)
ax1.legend(loc='upper right')
# Firmware boots in Mode 1 (Sensor Check), so reflect that here.
ax1.set_title("Live PID Monitor & Tuning Tool  [MODE: Sensor Check]")

# Live numeric readout beside the distance plot
dist_readout = ax1.text(
    1.02, 0.5, "", transform=ax1.transAxes, va='center', ha='left',
    fontsize=11, family='monospace',
    bbox=dict(boxstyle='round', facecolor='lightyellow', edgecolor='gray')
)
ax2.set_ylim(0, 180)
ax2.set_ylabel("Degrees")
ax2.legend(loc='upper right')
ax3.set_ylabel("Kp / Ki")
ax3b.set_ylabel("Kd")
ax3.set_ylim(0, 5)
ax3b.set_ylim(0, 5)
ax3.set_title("Effective PID Gains (gain scheduling)")
# Combined legend for the twin axes
lines_gain = [line_kp, line_ki, line_kd]
ax3.legend(lines_gain, [l.get_label() for l in lines_gain], loc='upper right')

# =========================================================================
# WIDGETS SETUP (Sliders, Radio Buttons, Text Boxes, Check Buttons)
# =========================================================================

# 1. Setpoint Slider
# Sits well below the plots (see subplots_adjust bottom) so it isn't crowded.
ax_slider = plt.axes([0.2, 0.27, 0.5, 0.03])
s_setpoint = Slider(ax_slider, 'Setpoint (cm)', 15, 32, valinit=15)


def update_setpoint(val):
    ser.write(f"SET {val:.1f}\n".encode())


s_setpoint.on_changed(update_setpoint)

# 2. Mode Selector (Radio Buttons)
# active=1 selects 'Sensor Check' by default to match the firmware's Mode 1 boot.
ax_radio = plt.axes([0.83, 0.5, 0.13, 0.15])
radio_mode = RadioButtons(ax_radio, ('PID Control', 'Sensor Check'), active=1)


def update_mode(label):
    if label == 'PID Control':
        ser.write(b"MODE 0\n")
        ax1.set_title("Live PID Monitor & Tuning Tool  [MODE: PID Control]")
        print("Switched to PID Control Mode")
    elif label == 'Sensor Check':
        ser.write(b"MODE 1\n")
        ax1.set_title("Live PID Monitor & Tuning Tool  [MODE: Sensor Check]")
        print("Switched to Sensor Sanity Check Mode")
    fig.canvas.draw_idle()


radio_mode.on_clicked(update_mode)

# 3. Gain Scheduling Toggle (Check Button)
ax_sched = plt.axes([0.83, 0.35, 0.13, 0.08])
chk_sched = CheckButtons(ax_sched, ['Gain Sched'], [True])


def toggle_sched(label):
    state = chk_sched.get_status()[0]
    ser.write(b"SCHED ON\n" if state else b"SCHED OFF\n")
    print(f"Gain Scheduling: {'ON' if state else 'OFF'}")


chk_sched.on_clicked(toggle_sched)

# 4. PID Parameter Tuning Text Boxes
# IMPORTANT: These match the NEW direct-to-degrees gain scale in the .ino.
# Do NOT use the old values (8.0 / 0.05 / 2500.0) — those were for the
# removed map(-150,150,40,125) scaling and will cause violent oscillation.
ax_kp = plt.axes([0.2, 0.1, 0.1, 0.04])
ax_ki = plt.axes([0.4, 0.1, 0.1, 0.04])
ax_kd = plt.axes([0.6, 0.1, 0.1, 0.04])

txt_kp = TextBox(ax_kp, 'Kp ', initial='6')
txt_ki = TextBox(ax_ki, 'Ki ', initial='1.1')
txt_kd = TextBox(ax_kd, 'Kd ', initial='4')


def submit_kp(text):
    try:
        val = float(text)
        ser.write(f"KP {val}\n".encode())
        print(f"Sent: KP {val}")
    except ValueError:
        pass


def submit_ki(text):
    try:
        val = float(text)
        ser.write(f"KI {val}\n".encode())
        print(f"Sent: KI {val}")
    except ValueError:
        pass


def submit_kd(text):
    try:
        val = float(text)
        ser.write(f"KD {val}\n".encode())
        print(f"Sent: KD {val}")
    except ValueError:
        pass


txt_kp.on_submit(submit_kp)
txt_ki.on_submit(submit_ki)
txt_kd.on_submit(submit_kd)

# =========================================================================
# ANIMATION LOOP
# =========================================================================


def animate(i):
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8').strip()
            if "," in line:
                parts = [float(x) for x in line.split(",")]
                # Expect 6 columns: dist, setpoint, servo, Kp, Ki, Kd.
                # Fall back gracefully if firmware still sends only 3.
                d, s, a = parts[0], parts[1], parts[2]
                kp_eff = parts[3] if len(parts) > 3 else float('nan')
                ki_eff = parts[4] if len(parts) > 4 else float('nan')
                kd_eff = parts[5] if len(parts) > 5 else float('nan')

                # Update Plot Buffers
                dist_data.append(d)
                set_data.append(s)
                srv_data.append(a)
                kp_data.append(kp_eff)
                ki_data.append(ki_eff)
                kd_data.append(kd_eff)

                # Save to CSV
                current_time = time.strftime("%H:%M:%S", time.localtime())
                csv_writer.writerow(
                    [current_time, d, s, a, kp_eff, ki_eff, kd_eff]
                )
                csv_file.flush()
        except (UnicodeDecodeError, ValueError):
            # Skip malformed lines (e.g. Arduino status messages) but do
            # NOT swallow every exception blindly.
            continue

    line_dist.set_ydata(dist_data)
    line_set.set_ydata(set_data)
    line_srv.set_ydata(srv_data)
    line_kp.set_ydata(kp_data)
    line_ki.set_ydata(ki_data)
    line_kd.set_ydata(kd_data)

    # Live numeric readout (latest values)
    d_now = dist_data[-1]
    s_now = set_data[-1]
    a_now = srv_data[-1]
    dist_readout.set_text(
        f"Distance: {d_now:6.1f} cm\n"
        f"Setpoint: {s_now:6.1f} cm\n"
        f"Error:    {s_now - d_now:+6.1f} cm\n"
        f"Servo:    {a_now:6.0f} deg"
    )

    return (line_dist, line_set, line_srv,
            line_kp, line_ki, line_kd, dist_readout)


try:
    # blit=False so the numeric readout (placed outside the axes) refreshes
    # live every frame, not only when the slider forces a full redraw.
    # cache_frame_data=False avoids the unbounded-cache warning for a live stream.
    ani = animation.FuncAnimation(
        fig, animate, interval=20, blit=False, cache_frame_data=False
    )
    plt.show()
finally:
    print("Closing interface... Saving CSV.")
    ser.write(b"STOP\n")
    ser.close()
    csv_file.close()
