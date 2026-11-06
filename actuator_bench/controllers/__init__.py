"""The motor controllers the app can drive, each behind the MotorController interface (device.py).

* ``odrive``: ODrive V3.6 with firmware 0.5.x (controllers/odrive.py)
* ``simplefoc``: any board running a SimpleFOC sketch with Commander, such as the ESP32 FOC drivers
  (controllers/simplefoc.py); the bench firmware in firmware/simplefoc_bench adds a watchdog,
  supply voltage and temperature.
"""

CONTROLLERS = {
    "odrive": "ODrive V3.6 (firmware 0.5.x)",
    "simplefoc": "SimpleFOC board (ESP32 FOC driver and others)",
}

# How messages and pages name each kind: "Connect to the ODrive", "The SimpleFOC board switched…"
CONTROLLER_NAMES = {"odrive": "ODrive", "simplefoc": "SimpleFOC board"}
