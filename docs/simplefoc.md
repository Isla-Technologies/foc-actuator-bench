# Measuring an actuator on a SimpleFOC board

The app drives any board that runs a [SimpleFOC](https://docs.simplefoc.com) sketch with
**Commander**, SimpleFOC's text command interface, such as the ESP32 FOC driver boards (ESP32-WROOM-32E,
three half bridges, inline current sensing on two phases). Every measurement works as it does on the
ODrive: live torque, stall, hold and calibration, and the simulation-model tests.

There are two ways to run the board:

| | Bench firmware (recommended) | Your own SimpleFOC sketch |
|---|---|---|
| What to flash | `firmware/simplefoc_bench` from this folder | any sketch with `command.add('M', doMotor, "motor")` |
| Watchdog (motor off if the app or cable stops) | yes | **no** |
| Motor off above the speed limit, in the board's control loop | yes | only by the app, at its sample rate |
| Supply voltage (needed for the No-load speed test) | yes (measured, or the value you set) | no |
| Temperature (needed for the Thermal test) | if a sensor is fitted | no |
| Re-run the sensor alignment from the app | yes | reset the board instead |

Either way the sketch must control **current** (`motor.torque_controller = TorqueControlType::foc_current`,
with its current sensors linked): the app measures torque from the current, and refuses to switch on a
motor that is controlled by voltage.

## 1. Wire it

1. **Motor** phases to the board's A, B and C terminals.
2. **Position sensor** (magnetic encoder on the motor) to the board's sensor header: an AS5600 (I2C) or
   an AS5047P (SPI).
3. **Computer**: if the board has no USB socket (most ESP32 FOC driver boards don't), use a 3.3 V
   USB-serial adapter (CP2102, CH340 or FTDI) on the board's programming header: adapter **TX to the
   board's RX**, adapter **RX to the board's TX**, and **GND to GND**. Check the header's labels on the
   board or in its documentation. Leave the adapter's 3.3 V/5 V pin unconnected when the board has its
   own supply.
4. **Power supply** to the DC input (within the board's rating, e.g. 12–35 V). Use a current-limited
   bench supply for the first run.
5. **Clamp the actuator** to the bench, and leave the output free to turn for the first start: the
   board turns the motor a little to align its sensor every time it starts.

## 2. Flash the bench firmware

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) (or arduino-cli), the **esp32** board
   package (Boards Manager, by Espressif) and the **Simple FOC** library 2.3 or newer (Library Manager).
2. Open `firmware/simplefoc_bench/simplefoc_bench.ino`.
3. Fill in the **CONFIGURE** block at the top:
   - **Pins**: the PWM, enable and current-sense pins are specific to the board. Copy them from the
     board's schematic, or from the example sketch that came with it: it has lines like
     `BLDCDriver3PWM driver = BLDCDriver3PWM(…)` and `InlineCurrentSense current_sense = InlineCurrentSense(…)`.
     The current-sense pins must be on the ESP32's ADC1 (GPIO 32–39).
   - **Current sensing**: the shunt resistance per phase and the amplifier's gain. "R020" printed on a
     shunt means 0.020 Ω; two in parallel make 0.010 Ω. INA240A2 amplifiers have a gain of 50.
   - **Motor**: pole pairs (a BE4108 has 22 magnets: 11 pole pairs) and the alignment voltage (about
     phase resistance × 3 A; 1 V is a safe start for a 4108-size motor).
   - **Sensor**: uncomment the one you have and set its pins.
   - **Supply voltage**, and the optional supply-voltage divider and temperature sensors.
   - Then set `BOARD_PINS_CHECKED` to 1. Until then the sketch refuses to compile, so it can't run
     with wrong pins.
4. Choose the board **ESP32 Dev Module**, the adapter's port, and upload. To flash, most ESP32 boards
   need GPIO0 held low while they reset (a BOOT button, a jumper, or an adapter that does it through
   DTR/RTS): follow the board's own instructions.
5. Open the Serial Monitor at **115200** baud and press the board's reset button. With the output free,
   the motor twitches while it aligns, then `MOT: Ready.` appears. If it says `MOT: Init FOC failed`,
   check the sensor wiring, the pole pairs and the alignment voltage. **Close the Serial Monitor** before
   connecting the app: only one program can use the port.

## 3. Set up the app

In **Setup**, make a **new profile for this actuator** (Save as new profile…), so its calibration and
simulation model stay separate from your other actuators:

| Field | Example: BE4108 380 KV, direct drive |
|---|---|
| Motor driven by | SimpleFOC board |
| Motor command letter | M (the bench firmware's) |
| Motor KV | 380 |
| Pole pairs | 11 |
| Phase resistance / inductance | from the datasheet if you have them (the No-load speed test uses them) |
| Gear ratio | your actuator's (1 for direct drive) |
| Max motor current | at most the firmware's limit, shown on the Connect page |
| Max output speed | for the No-load speed test, about 1.5 × the speed you expect |

On **Connect**, choose **USB serial (Commander)**, the adapter's port (marked CP210x, CH340, FTDI…) and
the baud rate (115200 for the bench firmware), then **Connect**. The checklist shows what the board
can do, its current limit and any fault. Then measure as on the ODrive:
[Measuring an actuator for robot simulation](simulation-model.md).

## Several actuators

Make one profile per actuator, each with its own controller. A profile keeps its calibration and its
simulation-model results, so you can measure an ODrive actuator one day and a SimpleFOC one the next.
On **Overview and export**, export each profile's model (one JSON file each) for Isla Studio.

## What differs from the ODrive

- **Speed in torque mode.** SimpleFOC doesn't limit the speed in torque mode. The bench firmware
  switches the motor off above 1.2 × the speed limit; with a plain sketch only the app does, a sample at
  a time, so an unloaded motor can overshoot. Keep the output loaded or blocked in torque mode.
- **Top speed.** SimpleFOC applies at most its `voltage_limit` per phase, and space-vector modulation
  gives at most 1/√3 of the supply (sine modulation 1/2). The No-load speed test records both, and the
  export works the speeds out for your robot's battery voltage from them.
- **Motor constants.** The ODrive measures phase resistance and inductance in its motor calibration;
  SimpleFOC only knows them if the sketch sets them. Enter them in Setup otherwise.
- **Current range.** The current sensors read about ±1.65 V ÷ (shunt × gain): ±3.3 A with 0.01 Ω and a
  gain of 50. The bench firmware never lets the app ask for more than 90 % of that, whatever the board's
  power rating.

## Troubleshooting

| Problem | What to do |
|---|---|
| No port appears | Try another cable or adapter. If the adapter still doesn't show up, install its maker's driver (Silicon Labs for CP210x, WCH for CH340). |
| "no answer like a SimpleFOC sketch" | Close the Serial Monitor. Check the baud rate, that TX and RX are crossed over, and that the sketch calls `command.run()` in `loop()`. |
| "has no Commander command 'M'" | The sketch registers the motor under another letter: enter that letter in Setup. |
| "controls the motor by voltage" | Use the bench firmware, or set `foc_current` and link the current sensors in your sketch. |
| Motor start-up failed | Free the output, check the sensor and phase wiring, then **Run motor calibration** on the Connect page (bench firmware) or reset the board. |
| Torque reads far too high or low | Check the shunt value and amplifier gain in the firmware, then do the weight calibration. |
| `WATCHDOG_EXPIRED` | The app stopped talking to the board (cable, sleep, freeze). Clear errors and continue. |
| `OVERSPEED` | The output turned faster than 1.2 × the speed limit: lower the torque or raise Max output speed. |
