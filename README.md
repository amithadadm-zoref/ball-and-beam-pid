# Ball & Beam PID Controller (ESP32)

A closed-loop **ball-and-beam** balancing system. An ESP32 reads the ball's
position from a Sharp IR distance sensor and tilts the beam with a servo to hold
the ball at a commanded distance, using a PID controller with **gain
scheduling**, sensor/derivative filtering, and anti-windup. A Python/matplotlib
dashboard provides live plotting, CSV logging, and on-the-fly tuning.

![Ball and Beam demo](docs/demo.gif)
<!-- Replace the line above with your run video/GIF. See the "Demo Video" section. -->

---

## The System

![Ball and Beam system](PID_Ball_And_Beam.jpeg)

The rig is a lightweight **cardboard channel beam** that pivots at its left end
on a 3D-printed bracket. A green ball rolls freely along the channel. At the
**left end**, the Sharp IR sensor points down the track and measures the ball's
distance. At the **right end**, the MG-996R servo drives a pushrod linkage that
raises and lowers the beam to tilt it. The **ESP32 and breadboard** sit in the
middle wiring it all together, and the **laptop** (left) runs the Python
dashboard for live plotting and tuning. Tilting the beam changes the ball's
acceleration; the PID loop continuously adjusts the tilt to park the ball at the
commanded distance.

---

## Hardware

| Part | Notes |
|------|-------|
| ESP32 dev board | Any variant with ADC on GPIO 27 |
| Sharp **GP2Y0A21YK0F** IR distance sensor | ~10–80 cm range; signal → GPIO 27 |
| **MG-996R** servo | Powered from a **separate 4.8–6 V supply** (not the ESP32 pin) |
| Beam + ball rig | Sensor mounted at one end, pointing down the track |

**Wiring**

| From | Wire | To |
|------|------|----|
| ESP32 `GND` | black | breadboard ground rail |
| IR sensor V+ | red | ESP32 `Vin` (~5 V from USB) |
| IR sensor signal | yellow | ESP32 `D27` |
| IR sensor GND | black | ESP32 `GND` |
| Servo signal | yellow | ESP32 `D26` |
| Servo GND | brown | ESP32 `GND` (shared ground) |
| Servo V+ | red | breadboard power rail |
| Battery + | red | breadboard power rail (same line as servo red) |
| Battery − | black | breadboard ground (same line as ESP32 GND) |

The servo is powered from the **battery rail**, not the ESP32 — but all grounds
are **common** (ESP32 GND ↔ servo GND ↔ battery −), which is essential for the
servo signal to be referenced correctly.

> ⚠️ **Power warning:** the photo/build uses a **9 V battery** for the servo.
> The MG-996R is rated **4.8–7.2 V**, so 9 V is **over-voltage** and a 9 V PP3
> can't supply the servo's stall current (>2 A) — expect brownouts and possible
> servo damage. Use a **4×AA pack (6 V)** or a **5–6 V UBEC/buck converter**
> instead, and add a **1000 µF capacitor** across the servo power rails.

---

## Repository Layout

```
BallAndBeam_Arduino/
├── BallAndBeam_PID/
│   └── BallAndBeam_PID.ino      # ESP32 firmware (PID + gain scheduling)
├── BallAndBeam_Dashboard.py     # Python live dashboard / tuner / logger
├── BallAndBeam_PID_Explained.md # Full algorithm & control-flow documentation
└── README.md
```

---

## Firmware — `BallAndBeam_PID.ino`

Runs a 25 Hz control loop. Key features:

- **Time-correct PID** using real `dt` (sample-rate independent).
- **Gain scheduling** — gains adapt with distance-from-setpoint (aggressive far,
  gentle near) to reduce overshoot.
- **Filtering** — EMA on the sensor and a low-pass on the derivative.
- **Anti-windup** integral clamp and a **servo slew-rate limiter**.
- **Two modes** — `0` full PID, `1` sensor sanity check (servo held level).

Flash it with the Arduino IDE (ESP32 board package + `ESP32Servo` library).

Full walkthrough of every algorithm: **[BallAndBeam_PID_Explained.md](BallAndBeam_PID_Explained.md)**.

---

## Dashboard — `BallAndBeam_Dashboard.py`

Live plots of distance, servo angle, and the **effective (scheduled) gains**,
plus a numeric readout and interactive tuning.

### Setup
```bash
pip install pyserial matplotlib numpy
```

### Run
1. Set `SERIAL_PORT` at the top of the script to your ESP32's port (e.g. `COM7`).
2. Flash the firmware, then:
   ```bash
   python BallAndBeam_Dashboard.py
   ```

### Controls
| Widget | Action |
|--------|--------|
| Setpoint slider | Target distance (cm) |
| Mode radio | PID Control / Sensor Check |
| Gain Sched checkbox | Enable/disable gain scheduling |
| Kp / Ki / Kd text boxes | Live-tune the base gains |

Each run is logged to `pid_data_<timestamp>.csv`
(`Timestamp, Distance, Setpoint, Servo, Kp_eff, Ki_eff, Kd_eff`).

---

## Serial Protocol

Commands (dashboard → ESP32): `START`, `STOP`, `SET <cm>`, `KP <v>`, `KI <v>`,
`KD <v>`, `SCHED ON`, `SCHED OFF`, `MODE 0`, `MODE 1`.

Telemetry (ESP32 → dashboard): CSV line
`Distance,Setpoint,ServoAngle,Kp_eff,Ki_eff,Kd_eff` every control cycle.

---

## Tuning Notes

Well-behaved baseline: `kp = 6`, `ki = 1.1`, `kd = 4` (direct-to-degree scale).

- Oscillating → lower `kp` or raise `kd`.
- Sluggish → raise `kp`.
- Twitchy servo → lower `ALPHA_DERIVATIVE` for more derivative smoothing.
- Keep setpoints within the beam's **physical reach** (~15–28 cm here);
  unreachable targets just wind the integral against a wall.

---

## Demo Video

GitHub can display a short clip. Options:

1. **Drag-and-drop (easiest):** edit this README on GitHub's web UI and drag your
   `.mp4` into the editor — GitHub hosts it and inserts a player link. No LFS needed.
2. **Commit a GIF:** convert the clip to `docs/demo.gif` (kept small) and it
   renders inline via the image tag at the top of this file.
3. **Git LFS:** if you want the raw `.mp4` in the repo,
   `git lfs track "*.mp4"` first (raw videos are git-ignored by default here).

---

## License

Personal hobby project. Add a license of your choice (e.g. MIT) if you want
others to reuse it.
