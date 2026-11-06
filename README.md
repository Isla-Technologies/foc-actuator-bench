# ODrive Actuator Bench

Bench-test an actuator driven by an **ODrive V3.6** or a **SimpleFOC board** (such as the ESP32 FOC
driver boards). Measure its torque live, while it works, and with
automated **stall (peak torque)** and **holding torque** tests. Then measure everything a **robot
simulator** needs to know about it: friction, rotor inertia, no-load and corner speed, backlash,
gearbox stiffness and continuous current, exported in one file. Keep a profile per actuator, each on
its own controller, and export one model each. Runs on **macOS** (Apple Silicon and Intel) and
**Windows**. Includes a simulator of both controllers, so you can try everything without hardware.

Formerly *ODrive Torque Meter*: settings and data from it carry over on the first start.

![Measure screen during a stall test](docs/images/measure.png)

## What it does

- **Live torque**: big output-torque readout with peak, mean and RMS, plus motor current, speed,
  angle, bus voltage, power and temperature, with live charts.
- **Stall test (peak torque)**: press the arm against a fixed object; the app ramps the torque up to
  your target, holds it briefly and releases. It stops by itself if the arm moves.
- **Hold test (holding torque)**: the motor holds its position while you apply a load.
- **Operational torque**: watch and record torque while the actuator runs (motor off, or in torque,
  speed or position control).
- **Weight calibration**: hang known weights on a lever arm to measure the real torque per amp
  (corrects gearbox losses and an inexact motor constant).
- **Simulation model**: tests that measure what a robot simulator needs, each set up and stopped
  safely by the app, and an export of every value with where it came from. See
  [Measuring an actuator for robot simulation](docs/simulation-model.md).
- **Recording**: every sample to CSV (opens in Excel/Numbers), plus a results folder per test.
- **Safety**: STOP button (Esc / Space), limits you set first, the controller's watchdog, motor off on
  exit.

## How the torque is measured

The controller has no torque sensor. It measures the motor's torque-producing current (*Iq*), and
torque follows from it:

```
motor torque  = Kt × Iq            with Kt ≈ 8.27 / KV   [N·m per amp]
output torque = motor torque × gear ratio × gearbox efficiency
```

After a weight calibration the app uses the measured value instead:
`output torque = K_cal × Iq`.

How accurate is it?
- Without calibration, expect roughly **±10–20 %** (KV from a datasheet is approximate, and gearbox
  losses are unknown).
- With the weight calibration, a few percent in steady conditions.
- At very high currents motors saturate magnetically, so the real peak torque can be somewhat lower
  than the current-based number. Only an external load cell would show that.
- While the actuator accelerates, part of the torque goes into accelerating the motor and arm.

## Before you start

With an **ODrive**:

- ODrive **V3.6** with firmware **0.5.x** (0.5.6 is the latest and recommended).
- The ODrive is already **set up for your motor** (motor and encoder calibrated; it works in
  odrivetool). The app can re-run the motor calibration, but it doesn't configure a new motor.
- The ODrive's **DC power supply is on**. USB alone does not power the ODrive.
- A **USB data cable** from the ODrive to the computer (some cables only charge).
- The actuator is **clamped down** so it can't move around.

With a **SimpleFOC board** (ESP32 FOC drivers and others):

- The board runs the **bench firmware** from `firmware/simplefoc_bench` (recommended: it adds a
  watchdog, supply voltage, temperature and an overspeed trip), or your own SimpleFOC sketch with
  Commander and current control.
- Its USB port, or a **USB-serial adapter** on its TX, RX and GND, is plugged into the computer.
- The setup, step by step: **[Measuring an actuator on a SimpleFOC board](docs/simplefoc.md)**.

## Install and start

### macOS

1. **Install Python** 3.12 or 3.13: download the macOS installer from
   [python.org/downloads](https://www.python.org/downloads/) and run it.
2. **Get the app**: on this GitHub page click **Code → Download ZIP** and unzip it (or
   `git clone` the repository).
3. **Start it**: open **Terminal**, type `bash ` (with a space), drag the file `start_mac.command`
   from the app folder into the Terminal window, and press Enter.

The first start installs everything into a private folder (`.venv` inside the app folder). This
needs internet and takes a few minutes. After that, the same command starts the app straight away.

> Double-clicking `start_mac.command` also works if the file is executable. If macOS says it
> "cannot be opened because it is from an unidentified developer", use the Terminal method above.

### Windows

1. **Install Python** 3.12 or 3.13 from [python.org/downloads](https://www.python.org/downloads/).
   During setup, tick **"Add python.exe to PATH"**.
2. **Get the app**: **Code → Download ZIP**, then unzip it.
3. **Double-click `start_windows.bat`**. The first start installs everything (a few minutes).

No USB driver is needed on either system: the ODrive's built-in USB serial port works out of the
box, as do most USB-serial adapters for SimpleFOC boards (if one doesn't show up, install its maker's
driver: Silicon Labs for CP210x, WCH for CH340).

### Try it without hardware

On the Connect page choose **Simulator (no hardware)**. "Arm against a fixed object" is set up for
the stall test, and "Arm with a hanging weight" for the calibration. It simulates whichever controller
the profile is on. You can also start straight into it with `python -m actuator_bench --sim`.

## Using the app

The app guides you through three steps: **1 Setup → 2 Connect → 3 Measure**.

### 1 · Setup: enter your actuator's values

![Setup step](docs/images/setup.png)

| Field | What to enter |
|---|---|
| Motor driven by | ODrive V3.6 or SimpleFOC board. |
| Motor connected to (ODrive) | M0 or M1 on the ODrive. |
| Motor command letter (SimpleFOC) | The letter the sketch registers the motor under: `M` for the bench firmware. |
| Motor KV *or* torque constant | KV from the motor's datasheet (rpm per volt), or Kt in N·m/A if you know it. |
| Pole pairs, phase resistance and inductance | Optional: the No-load speed test uses them when the controller doesn't know them (a SimpleFOC board usually doesn't). |
| Gear ratio | Motor turns per output turn, e.g. `9` for a 9:1 planetary. `1` for direct drive. |
| Gearbox efficiency | Leave 100 % if unknown; the weight calibration measures the real value. |
| Max motor current | The highest current any test may use (the controller's own limit still applies). |
| Max output speed | Speed limit at the output for manual control and tests. |
| Torque unit | N·m, kgf·cm or lbf·in. Files always store N·m. |

Values are checked as you type. **Next** unlocks when everything is valid. Save each actuator as its
own profile (**Save as new profile…**): its calibration and simulation-model results stay with it.

### 2 · Connect

![Connect step](docs/images/connect.png)

For an ODrive, choose **Auto** (recommended) and press **Connect**. Auto tries the ODrive's USB
serial port first, then the official ODrive Python library if it's installed. For a SimpleFOC board,
choose **USB serial (Commander)**, its port and the sketch's baud rate. The checklist then shows:

- firmware version
- whether the motor and encoder are calibrated
- any errors, in plain English
- whether the ODrive's torque constant matches your values
- the controller's current limit and bus voltage
- for a SimpleFOC board: whether it controls current, and what the bench firmware adds

Buttons, where the controller supports them: **Clear errors**, **Run motor calibration** (the output
must be free to move) and **Write my Kt to the ODrive** (optional: the app already corrects for a
mismatch).

### 3 · Measure

- **Live readouts and charts.** Hover over the charts to read exact values. **Pause** freezes them,
  and **Window** sets the time span. **Peak / Mean / RMS** run since the last **Reset**.
- **Manual control.** With the motor off you are only monitoring. **Switch motor on** in Torque,
  Speed or Position (hold) mode, then use the slider or the number field. Operational torque is
  whatever the dashboard shows while the actuator does its job; record it with **Record**.
- **Stall test (peak torque).**
  1. Press the arm firmly against something solid (bench, clamp, end stop) in the direction you test.
  2. Enter the target torque (the maximum allowed by your current limit is shown) and the times.
     Keep the hold short: a stalled motor heats up quickly.
  3. Press **Start stall test** and keep your hands clear.

  You get the peak torque, the current at the peak, the average while holding, how far the arm
  moved and the temperature rise. If the arm moves more than the allowed angle, the test stops: it
  wasn't a real stall.
- **Hold test.** The motor holds its position while you push the arm, hang a weight or run your rig.
  You get the peak and the steady holding torque. The torque limit decides when the actuator gives
  way.
- **Calibration.** You need a rigid arm of known length on the output and two or three known weights
  (e.g. 0, 0.5 and 1 kg).
  1. Enter the arm length.
  2. Make the arm level (by hand with the motor off, or press **Hold the arm** and jog it). Check
     with a spirit level, then press **Arm is level: set horizontal**.
  3. For each weight, enter it and press **Measure this weight**. The arm sweeps slowly ±10°
     around horizontal; averaging the up and down movement cancels friction.
  4. Press **Compute calibration**, then **Save to profile**. From then on, torque readings use the
     calibrated value.
  5. Before **Release the arm**, support the arm or remove the weight: it drops when the motor
     switches off.

![Calibration with the simulator](docs/images/calibration.png)

### Simulation model

The **Simulation model** tab measures what a robot simulator needs to know about the actuator, one
test per tab:

| Tab | Measures | The output must be |
|---|---|---|
| Friction | Coulomb friction and damping | free to turn through the travel |
| Inertia | the inertia the rotor and gearbox add to the joint | free to swing ±20° |
| No-load speed | no-load and corner speed at your battery voltage, motor back-EMF | free to spin |
| Backlash | gearbox play and stiffness | clamped both ways |
| Thermal | continuous current | blocked, as for the stall test |
| Gear ratio | the real gear ratio | turned by hand, motor off |

Each tab explains its setup, and each test stops by itself if the output does what it shouldn't
(moves when it should be clamped, sticks when it should turn). Results are kept on the profile.
**Overview and export** lists every value with where it came from (measured, calculated, from
Setup, or not measured yet) and exports them as one JSON file for the simulator. For several
actuators, switch profile in Setup and export each one: one file per actuator for Isla Studio.

![Simulation model overview](docs/images/model-overview.png)

The full guide, with the order to run the tests, what each result means, how accurate it is and its
limitations: **[Measuring an actuator for robot simulation](docs/simulation-model.md)**.

### Where your data goes

Recordings and test results go to **Documents/ODrive Actuator Bench** (change it under
**File → Change data folder**). Each test creates a folder with `summary.json`, `data.csv` and a
chart image. CSV files start with `#` lines listing the settings used (KV, gear ratio,
calibration, controller and its serial number…).

## Safety

- Clamp the actuator down and keep hands, cables and people out of the arm's path.
- **STOP** (top right), **Esc** or **Space** switch the motor off immediately.
- The motor never switches on by itself: the Switch motor on and Start buttons do it, within the
  limits from Setup.
- While the motor is on, the controller's **watchdog** is active. If the app freezes or the USB
  cable is pulled, the controller switches the motor off within about a second. A SimpleFOC board has
  one only with the bench firmware: with a plain sketch the motor keeps its last command.
- In **torque mode with the output free**, the actuator accelerates up to the speed limit. The ODrive
  holds it there; a SimpleFOC board is switched off above 1.2 × the limit (by the bench firmware at
  once, otherwise by the app a sample later).
- Once started, the **Simulation model tests move the output by themselves**: the friction, inertia
  and no-load speed tests turn it, the backlash and thermal tests push it against its clamp.
- **Stalled motors heat up fast.** Keep stall holds to a few seconds and watch the temperature.
- Settings the app changes on the controller (limits, control mode, watchdog) are put back when you
  disconnect. Nothing is saved permanently to it.

## Troubleshooting

| Problem | What to do |
|---|---|
| "No ODrive serial port found" | Switch the ODrive's DC power on (USB doesn't power it). Try another cable or USB port. Run `python -m actuator_bench ports` (inside `.venv`) to list ports. |
| "didn't answer like an ODrive" | The ODrive's USB port may be set to another protocol. Use the **Official ODrive library** connection, or in odrivetool run `odrv0.config.usb_cdc_protocol = 3` then `odrv0.save_configuration()`. |
| Official library: "could not claim interface" | Only one program can use that connection. Close odrivetool (the USB serial connection doesn't have this limit). |
| Motor won't switch on | Look at the Connect checklist: clear errors, and make sure the motor is calibrated and the encoder is ready. |
| Torque mode gives less torque than set at standstill | The ODrive caps torque in torque mode to `vel_limit × vel_gain`. Raise the speed limit, or use the Stall test, which handles this automatically. |
| `WATCHDOG_TIMER_EXPIRED` | The app stopped talking to the ODrive (cable, sleep, freeze). Clear errors and continue. |
| Firmware 0.5.2 / 0.5.3 | These have ASCII-protocol bugs. Use the Official library connection or update to 0.5.6. |
| SimpleFOC board problems | See the troubleshooting table in [the SimpleFOC guide](docs/simplefoc.md#troubleshooting). |
| Anything else | **Help → Save debug report…** and send the file to whoever helps you. It contains versions, controller settings and recent log messages. |

## Command-line tools

These never move the motor:

```
python -m actuator_bench ports                         # list serial ports, ODrives and adapters marked
python -m actuator_bench info                          # board, firmware, state, errors, configuration
python -m actuator_bench monitor --kv 100 --gear 9     # print live torque (Ctrl+C to stop)
python -m actuator_bench monitor --csv run.csv --seconds 30
python -m actuator_bench info --controller simplefoc --port COM7 --baud 115200
python -m actuator_bench monitor --controller simplefoc --kv 380
```

Run them with the app's Python: `.venv/bin/python` on macOS/Linux, `.venv\Scripts\python` on
Windows. Add `--connection simulator` to try them without hardware.

## For developers

```
actuator_bench/
  control.py           control modes, motor states, faults: the same words for every controller
  device.py            MotorController: what the app needs from any controller
  controllers/         odrive (an ODrive axis), simplefoc (a SimpleFOC motor), commander (its protocol)
  odrive_api.py        ODrive firmware 0.5.x property paths, enums, error codes with explanations
  backends/            serial_backend (ASCII over USB), native_backend (official odrive library),
                       simulator (virtual ODrive + actuator), sim_serial (fake serial port),
                       sim_simplefoc (fake SimpleFOC board on the same physics)
  acquisition.py       background worker: polling, torque, procedures, commands
  procedures.py        stall test, hold test, calibration sweep
  calibration.py       known-weight calibration math
  model_tests.py       friction, inertia, no-load speed, backlash and thermal tests
  identification.py    the fits behind them (friction, inertia, motor model, backlash, heating)
  model_export.py      every measured value, with its source, as one simulation model
  torque.py            Kt, gear ratio, units, filters, statistics
  recorder.py, settings.py, diagnostics.py, cli.py
  ui/                  PySide6 interface (setup, connect and measure pages, live charts,
                       the Simulation model tab and its result charts)
tests/                 unit, protocol, end-to-end and GUI tests (all against the simulators)
firmware/simplefoc_bench/   the bench firmware for SimpleFOC boards (Arduino, ESP32)
```

```
python -m venv .venv && .venv/bin/pip install -r requirements.txt pytest
.venv/bin/python -m pytest          # runs headless; set QT_QPA_PLATFORM=offscreen on CI
.venv/bin/python -m actuator_bench --sim
```

GitHub Actions runs the tests on macOS, Windows and Ubuntu (Python 3.10 and 3.13), and runs the
launch scripts on a fresh macOS and Windows machine.

Built on the ODrive 0.5.6 documentation: [getting started](https://docs.odriverobotics.com/v/0.5.6/getting-started.html),
[ASCII protocol](https://docs.odriverobotics.com/v/0.5.6/ascii-protocol.html),
[API reference](https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html);
and on SimpleFOC's [Commander](https://docs.simplefoc.com/commander_interface) (Arduino-FOC 2.4.0 source).
To add another controller, implement `MotorController` (device.py) in `controllers/`.

## License

MIT. See [LICENSE](LICENSE). ODrive is a trademark of ODrive Robotics; this project isn't affiliated
with them, nor with the SimpleFOC project.
