// SimpleFOC bench firmware for ODrive Actuator Bench.
//
// A SimpleFOC sketch for an ESP32 FOC driver board (three half bridges with inline current
// sensing, e.g. the "ESP32 FOC 20A" boards with IR2104 gate drivers and INA240 amplifiers). The app
// drives the motor through SimpleFOC's standard Commander motor command ('M'). The 'B' command adds
// what a bench needs and plain Commander lacks:
//
//   BI          info: fw, board, pole pairs, current limits, sensors fitted
//   BF          fast values: iq;iq_setpoint;velocity;angle   (A, A, rad/s, rad at the motor)
//   BS          status: vbus;driver_temp;motor_temp;motor_status;faults;enabled
//   BW<s>       watchdog timeout in seconds (0 = off): the motor switches off when the app goes quiet
//   BK          feed the watchdog
//   BO<rad/s>   overspeed trip (0 = off): SimpleFOC doesn't limit the speed in torque mode
//   BC          clear the faults (replies with those that remain)
//   BA          re-run the sensor alignment (the motor turns: keep the output free)
//
// Needs the Arduino "Simple FOC" library 2.3.0 or newer (Library Manager) and the ESP32 board
// package. Setup guide: docs/simplefoc.md in the app's folder.

#include <SimpleFOC.h>

// ================================================================================================
// CONFIGURE FOR YOUR BOARD, MOTOR AND SENSOR
//
// The pin numbers below are placeholders. Take the real ones from your board's schematic or the
// example sketch that came with it (look for "BLDCDriver3PWM driver(...)" and
// "InlineCurrentSense current_sense(...)"), then set BOARD_PINS_CHECKED to 1.
// ================================================================================================
#define BOARD_PINS_CHECKED 0

#define BOARD_NAME "ESP32 FOC driver"
#define SERIAL_BAUD 115200          // the app's Connect page must use the same rate

// --- Motor -------------------------------------------------------------------------------------
#define POLE_PAIRS 11               // BE4108: 22 magnets = 11 pole pairs
#define PHASE_RESISTANCE NOT_SET    // ohm, if you know it (else NOT_SET)
#define MOTOR_KV NOT_SET            // rpm/V, if you know it (else NOT_SET)
#define ALIGN_VOLTAGE 1.0           // V for the sensor alignment at start-up. About phase resistance x 3 A.

// --- Power stage -------------------------------------------------------------------------------
#define PIN_PWM_A 32                // CHANGE: phase A PWM
#define PIN_PWM_B 33                // CHANGE: phase B PWM
#define PIN_PWM_C 25                // CHANGE: phase C PWM
#define PIN_ENABLE_A NOT_SET        // CHANGE: gate driver enable (IR2104 SD), or NOT_SET
#define PIN_ENABLE_B NOT_SET        //         per phase if the board has one each
#define PIN_ENABLE_C NOT_SET
#define SUPPLY_VOLTAGE 24.0         // V: your power supply (also reported when PIN_VBUS isn't fitted)

// --- Current sensing (inline, phases A and B) --------------------------------------------------
#define SHUNT_OHM 0.01              // CHANGE: the shunt per phase (two R020 in parallel = 0.01)
#define AMP_GAIN 50.0               // INA240A1 = 20, A2 = 50, A3 = 100, A4 = 200
#define PIN_CS_A 39                 // CHANGE: ADC pin of phase A's amplifier (use ADC1: GPIO 32-39)
#define PIN_CS_B 36                 // CHANGE: ADC pin of phase B's amplifier
#define HARD_CURRENT_LIMIT 10.0     // A: the most the app may ask for (also capped by what the sensors read)

// --- Position sensor: uncomment one ------------------------------------------------------------
#define SENSOR_AS5600               // I2C magnetic encoder
// #define SENSOR_AS5047            // SPI magnetic encoder
#define PIN_SDA 19                  // CHANGE (AS5600)
#define PIN_SCL 18                  // CHANGE (AS5600)
#define PIN_SENSOR_CS 5             // CHANGE (AS5047)

// --- Optional sensors (-1 = not fitted) --------------------------------------------------------
#define PIN_VBUS -1                 // ADC pin on a divider from the supply
#define VBUS_DIVIDER 11.0           // supply volts per volt at the pin, e.g. 100k/10k = 11
#define VBUS_MIN 10.0               // V: below this the motor is switched off (only if PIN_VBUS is fitted)
#define VBUS_MAX 38.0               // V: above this too
#define PIN_DRIVER_NTC -1           // ADC pin of an NTC on the power stage (10k NTC to GND, 10k to 3.3 V)
#define PIN_MOTOR_NTC -1            // ADC pin of an NTC in the motor (same wiring)
#define NTC_BETA 3950.0
#define TEMP_LIMIT 90.0             // degC: above this the motor is switched off
// ================================================================================================

#if !BOARD_PINS_CHECKED
#error "Set your board's pins in the CONFIGURE block (see docs/simplefoc.md), then set BOARD_PINS_CHECKED to 1."
#endif

#define FIRMWARE_VERSION "1.0"

// Faults reported by BS (same bits as the app's controllers/simplefoc.py).
#define FAULT_WATCHDOG 0x01
#define FAULT_OVER_TEMPERATURE 0x02
#define FAULT_UNDER_VOLTAGE 0x04
#define FAULT_OVER_VOLTAGE 0x08
#define FAULT_OVER_CURRENT 0x10
#define FAULT_START_UP 0x20
#define FAULT_OVERSPEED 0x40

BLDCMotor motor = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, MOTOR_KV);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_ENABLE_A, PIN_ENABLE_B, PIN_ENABLE_C);
InlineCurrentSense current_sense = InlineCurrentSense(SHUNT_OHM, AMP_GAIN, PIN_CS_A, PIN_CS_B, _NC);

#if defined(SENSOR_AS5600)
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
#elif defined(SENSOR_AS5047)
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, PIN_SENSOR_CS);
#else
#error "Choose a position sensor in the CONFIGURE block."
#endif

Commander command = Commander(Serial);

// The amplifier's output swings about half the ADC range either way.
const float SENSE_RANGE_A = 1.65f / (SHUNT_OHM * AMP_GAIN);
const float MAX_CURRENT = min((float)HARD_CURRENT_LIMIT, 0.9f * SENSE_RANGE_A);

uint8_t faults = 0;
uint32_t watchdog_ms = 0;          // 0 = off
uint32_t last_feed_ms = 0;
float overspeed_rad_s = 0.0f;      // 0 = off
uint16_t over_current_loops = 0;
float vbus = SUPPLY_VOLTAGE;
float driver_temp = NAN;
float motor_temp = NAN;
uint32_t last_slow_ms = 0;

// --- output helpers ----------------------------------------------------------------------------
bool talking() { return command.verbose != VerboseMode::nothing; }

void number(float value) { Serial.print(value, (int)command.decimal_places); }

void reply_start(char sub) {
  // Commander has printed 'B' already in machine-readable mode; the sub-command follows.
  if (command.verbose == VerboseMode::machine_readable) Serial.print(sub);
}

bool is_get(const char* value) { return value[0] == '\n' || value[0] == '\r' || value[0] == '\0'; }

// --- sensors -----------------------------------------------------------------------------------
float read_ntc(int pin) {
  float mv = analogReadMilliVolts(pin);
  if (mv <= 1.0f || mv >= 3299.0f) return NAN;   // open or shorted
  float r = 10000.0f * mv / (3300.0f - mv);       // NTC to GND, 10k to 3.3 V
  return 1.0f / (logf(r / 10000.0f) / NTC_BETA + 1.0f / 298.15f) - 273.15f;
}

void read_slow_sensors() {
  if (PIN_VBUS >= 0) vbus = analogReadMilliVolts(PIN_VBUS) / 1000.0f * VBUS_DIVIDER;
  if (PIN_DRIVER_NTC >= 0) driver_temp = read_ntc(PIN_DRIVER_NTC);
  if (PIN_MOTOR_NTC >= 0) motor_temp = read_ntc(PIN_MOTOR_NTC);
}

// --- safety ------------------------------------------------------------------------------------
void trip(uint8_t fault) {
  faults |= fault;
  motor.disable();
}

uint8_t check_conditions() {
  uint8_t found = 0;
  if (PIN_VBUS >= 0 && vbus < VBUS_MIN) found |= FAULT_UNDER_VOLTAGE;
  if (PIN_VBUS >= 0 && vbus > VBUS_MAX) found |= FAULT_OVER_VOLTAGE;
  if (driver_temp > TEMP_LIMIT || motor_temp > TEMP_LIMIT) found |= FAULT_OVER_TEMPERATURE;
  return found;
}

void safety() {
  uint32_t now = millis();
  if (now - last_slow_ms >= 50) {
    last_slow_ms = now;
    read_slow_sensors();
    uint8_t found = check_conditions();
    if (found && motor.enabled) trip(found);
  }
  if (!motor.enabled) {
    over_current_loops = 0;
    return;
  }
  if (watchdog_ms && now - last_feed_ms > watchdog_ms) trip(FAULT_WATCHDOG);
  if (overspeed_rad_s > 0 && fabsf(motor.shaft_velocity) > overspeed_rad_s) trip(FAULT_OVERSPEED);
  over_current_loops = fabsf(motor.current.q) > 1.3f * MAX_CURRENT ? over_current_loops + 1 : 0;
  if (over_current_loops > 20) trip(FAULT_OVER_CURRENT);
  if (faults) motor.disable();
}

// --- commands ----------------------------------------------------------------------------------
void do_motor(char* cmd) {
  // Never switch on with faults, and never above the current the sensors can measure.
  if (faults && cmd[0] == 'E' && !is_get(cmd + 1) && atof(cmd + 1) != 0) {
    char off[] = "E0\n";
    command.motor(&motor, off);
    return;
  }
  if (cmd[0] == 'L' && cmd[1] == 'C' && !is_get(cmd + 2) && atof(cmd + 2) > MAX_CURRENT) {
    char limited[20];
    snprintf(limited, sizeof(limited), "LC%.3f\n", MAX_CURRENT);
    command.motor(&motor, limited);
    return;
  }
  if (cmd[0] == 'E' && !is_get(cmd + 1) && atof(cmd + 1) != 0) last_feed_ms = millis();
  command.motor(&motor, cmd);
}

void do_bench(char* cmd) {
  char sub = cmd[0];
  const char* value = cmd + 1;
  if (!talking()) return;
  reply_start(sub);
  switch (sub) {
    case 'I':
      Serial.print("fw=" FIRMWARE_VERSION ";board=" BOARD_NAME ";pp=");
      Serial.print(POLE_PAIRS);
      Serial.print(";imax=");
      number(MAX_CURRENT);
      Serial.print(";isense=");
      number(SENSE_RANGE_A);
      Serial.print(PIN_VBUS >= 0 ? ";vbus=measured" : ";vbus=set");
      Serial.print(PIN_DRIVER_NTC >= 0 ? ";temp=1" : ";temp=0");
      Serial.print(PIN_MOTOR_NTC >= 0 ? ";mtemp=1" : ";mtemp=0");
#if defined(ESP32)
      Serial.print(";id=");
      Serial.print((uint32_t)(ESP.getEfuseMac() >> 32), HEX);
      Serial.print((uint32_t)ESP.getEfuseMac(), HEX);
#endif
      Serial.println();
      break;
    case 'F':
      number(motor.current.q); Serial.print(';');
      number(motor.current_sp); Serial.print(';');
      number(motor.shaft_velocity); Serial.print(';');
      number(motor.shaft_angle); Serial.println();
      break;
    case 'S':
      number(vbus); Serial.print(';');
      number(driver_temp); Serial.print(';');
      number(motor_temp); Serial.print(';');
      Serial.print((int)motor.motor_status); Serial.print(';');
      Serial.print((int)faults); Serial.print(';');
      Serial.println((int)motor.enabled);
      break;
    case 'W':
      if (!is_get(value)) {
        float seconds = atof(value);
        watchdog_ms = seconds > 0 ? (uint32_t)(seconds * 1000.0f) : 0;
        last_feed_ms = millis();
      }
      number(watchdog_ms / 1000.0f); Serial.println();
      break;
    case 'K':
      last_feed_ms = millis();
      Serial.println(1);
      break;
    case 'O':
      if (!is_get(value)) overspeed_rad_s = max(0.0f, (float)atof(value));
      number(overspeed_rad_s); Serial.println();
      break;
    case 'C':
      read_slow_sensors();
      faults = check_conditions();
      if (motor.motor_status != FOCMotorStatus::motor_ready) faults |= FAULT_START_UP;
      Serial.println((int)faults);
      break;
    case 'A': {
      motor.disable();
      watchdog_ms = 0;
      motor.sensor_direction = Direction::UNKNOWN;
      motor.zero_electric_angle = NOT_SET;
      int ok = motor.initFOC();
      motor.disable();
      if (ok) faults &= ~FAULT_START_UP; else faults |= FAULT_START_UP;
      Serial.println(ok ? 1 : 0);
      break;
    }
    default:
      Serial.println("err");
  }
}

void on_motor(char* cmd) { do_motor(cmd); }
void on_bench(char* cmd) { do_bench(cmd); }

// --- setup and loop ----------------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  SimpleFOCDebug::enable(&Serial);  // start-up messages; the app skips them

#if defined(SENSOR_AS5600)
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  sensor.init(&Wire);
#else
  sensor.init();
#endif
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = SUPPLY_VOLTAGE;
  driver.init();
  motor.linkDriver(&driver);
  current_sense.linkDriver(&driver);

  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.voltage_limit = SUPPLY_VOLTAGE / _SQRT3;  // all that space-vector PWM can give
  motor.voltage_sensor_align = ALIGN_VOLTAGE;
  motor.current_limit = min(1.0f, MAX_CURRENT);   // the app sets its own when it switches the motor on
  motor.velocity_limit = 20.0f;
  motor.LPF_velocity.Tf = 0.01f;

  motor.init();
  current_sense.init();
  motor.linkCurrentSense(&current_sense);
  if (!motor.initFOC()) faults |= FAULT_START_UP;
  motor.target = 0;
  motor.disable();  // the app switches it on

  command.add('M', on_motor, "motor");
  command.add('B', on_bench, "bench");
  read_slow_sensors();
  last_feed_ms = millis();
}

void loop() {
  motor.loopFOC();
  motor.move();     // no argument: the target is what Commander set
  safety();
  command.run();
}
