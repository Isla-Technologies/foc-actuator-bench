"""User settings: actuator profiles (the values entered in the Setup step) and app preferences."""

from __future__ import annotations

import json
import logging
import os
import sys
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path

from .controllers import CONTROLLERS
from .samples import Limits
from .torque import TORQUE_UNITS, TorqueModel, kt_from_kv

log = logging.getLogger(__name__)

APP_DIR_NAME = "FOC Actuator Bench"
# The app's earlier names, newest first: "ODrive Actuator Bench" (0.2), "ODrive Torque Meter" (0.1).
LEGACY_DIR_NAMES = ("ODrive Actuator Bench", "ODrive Torque Meter")


def config_dir() -> Path:
    if sys.platform == "win32":
        base = Path(os.environ.get("APPDATA") or Path.home() / "AppData" / "Roaming")
    elif sys.platform == "darwin":
        base = Path.home() / "Library" / "Application Support"
    else:
        base = Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config")
    return base / APP_DIR_NAME


def default_data_dir() -> Path:
    return Path.home() / "Documents" / APP_DIR_NAME


def legacy_settings_path() -> Path | None:
    """The settings file of the newest earlier name that has one."""
    for name in LEGACY_DIR_NAMES:
        path = config_dir().parent / name / "settings.json"
        if path.exists():
            return path
    return None


@dataclass
class ActuatorProfile:
    name: str = "My actuator"
    kt_source: str = "kv"             # "kv": derive Kt from KV, "kt": Kt entered directly
    kv: float = 100.0                 # motor KV [rpm/V]
    kt: float = 0.0827                # motor torque constant [N*m/A]
    gear_ratio: float = 1.0           # output turns once per gear_ratio motor turns
    efficiency_pct: float = 100.0     # gearbox efficiency
    # The motor's other values, for the Speed test and the simulation model when the controller
    # doesn't know them (0 = unknown).
    pole_pairs: int = 0
    phase_resistance_ohm: float = 0.0
    phase_inductance_uh: float = 0.0
    controller: str = "odrive"        # odrive | simplefoc (see controllers/__init__.py)
    axis: int = 0                     # ODrive: 0 = M0, 1 = M1
    motor_letter: str = "M"           # SimpleFOC: the motor's Commander command letter
    max_current_a: float = 10.0       # safety limit for every test
    max_speed_rpm: float = 60.0       # output side
    unit: str = "N·m"
    calibration_k_out: float | None = None   # output N*m per amp from the weight calibration
    calibration_date: str = ""
    calibration_details: dict = field(default_factory=dict)
    use_calibration: bool = True
    # The simulation model: the latest result of each test, and the robot it is for.
    model_results: dict = field(default_factory=dict)   # test name -> result summary (with "date")
    battery_voltage: float = 24.0     # V the robot's drivers run on
    peak_current_a: float = 0.0       # A the robot's drivers may give for a moment (0 = max motor current)
    temp_limit_c: float = 80.0        # °C the motor may reach (sets the continuous current)

    @property
    def kt_motor(self) -> float:
        return kt_from_kv(self.kv) if self.kt_source == "kv" else self.kt

    @property
    def has_calibration(self) -> bool:
        return bool(self.calibration_k_out)

    @property
    def robot_peak_current(self) -> float:
        return self.peak_current_a if self.peak_current_a > 0 else self.max_current_a

    def validate(self) -> dict[str, str]:
        """Problems keyed by field name; empty when everything is valid."""
        problems: dict[str, str] = {}
        if not self.name.strip():
            problems["name"] = "Give the profile a name."
        if self.kt_source == "kv" and not self.kv > 0:
            problems["kv"] = "Motor KV must be greater than 0."
        if self.kt_source == "kt" and not self.kt > 0:
            problems["kt"] = "Torque constant must be greater than 0."
        if self.kt_source not in ("kv", "kt"):
            problems["kt_source"] = "Choose KV or torque constant."
        if not self.gear_ratio > 0:
            problems["gear_ratio"] = "Gear ratio must be greater than 0 (use 1 for direct drive)."
        if not 0 < self.efficiency_pct <= 100:
            problems["efficiency_pct"] = "Efficiency must be between 1 and 100 %."
        if self.controller not in CONTROLLERS:
            problems["controller"] = "Choose the controller."
        if self.controller == "odrive" and self.axis not in (0, 1):
            problems["axis"] = "Choose M0 or M1."
        if self.controller == "simplefoc" and not (len(self.motor_letter) == 1 and self.motor_letter.isalpha()):
            problems["motor_letter"] = "One letter, as in the sketch's command.add('M', …) (usually M)."
        if not (isinstance(self.pole_pairs, int) and 0 <= self.pole_pairs <= 200):
            problems["pole_pairs"] = "Pole pairs: a whole number from 1 to 200, or 0 if unknown."
        if not 0 <= self.phase_resistance_ohm < 100:
            problems["phase_resistance_ohm"] = "Phase resistance can't be negative."
        if not 0 <= self.phase_inductance_uh < 1e6:
            problems["phase_inductance_uh"] = "Phase inductance can't be negative."
        if not self.max_current_a > 0:
            problems["max_current_a"] = "Current limit must be greater than 0."
        if not self.max_speed_rpm > 0:
            problems["max_speed_rpm"] = "Speed limit must be greater than 0."
        if self.unit not in TORQUE_UNITS:
            problems["unit"] = "Unknown torque unit."
        return problems

    def torque_model(self) -> TorqueModel:
        return TorqueModel(
            kt_motor=self.kt_motor,
            gear_ratio=self.gear_ratio,
            efficiency=self.efficiency_pct / 100.0,
            calibrated_k_out=self.calibration_k_out if self.use_calibration else None,
        )

    def limits(self) -> Limits:
        return Limits(max_current=self.max_current_a, max_motor_speed=self.max_speed_rpm / 60.0 * self.gear_ratio)

    @classmethod
    def from_dict(cls, data: dict) -> ActuatorProfile:
        known = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in data.items() if k in known})


@dataclass
class AppSettings:
    profiles: list[ActuatorProfile] = field(default_factory=lambda: [ActuatorProfile()])
    current_profile: str = "My actuator"
    connection_mode: str = "auto"
    serial_port: str = ""
    serial_baud: int = 115200         # SimpleFOC boards: the sketch's Serial.begin() rate
    sample_rate_hz: float = 100.0
    display_filter_ms: float = 30.0
    watchdog_enabled: bool = True
    watchdog_timeout_s: float = 1.0
    plot_window_s: float = 10.0
    data_dir: str = ""
    sim_scenario: str = "wall"
    sim_mass_kg: float = 0.0

    @property
    def data_path(self) -> Path:
        return Path(self.data_dir) if self.data_dir else default_data_dir()

    def profile(self, name: str | None = None) -> ActuatorProfile:
        wanted = name or self.current_profile
        for profile in self.profiles:
            if profile.name == wanted:
                return profile
        if not self.profiles:
            self.profiles.append(ActuatorProfile())
        return self.profiles[0]

    def upsert_profile(self, profile: ActuatorProfile) -> None:
        for i, existing in enumerate(self.profiles):
            if existing.name == profile.name:
                self.profiles[i] = profile
                break
        else:
            self.profiles.append(profile)
        self.current_profile = profile.name

    def delete_profile(self, name: str) -> None:
        self.profiles = [p for p in self.profiles if p.name != name]
        if not self.profiles:
            self.profiles.append(ActuatorProfile())
        if self.current_profile == name:
            self.current_profile = self.profiles[0].name

    def to_dict(self) -> dict:
        data = asdict(self)
        data["profiles"] = [asdict(p) for p in self.profiles]
        return data

    @classmethod
    def from_dict(cls, data: dict) -> AppSettings:
        known = {f.name for f in fields(cls)}
        values = {k: v for k, v in data.items() if k in known and k != "profiles"}
        settings = cls(**values)
        profiles = [ActuatorProfile.from_dict(p) for p in data.get("profiles", []) if isinstance(p, dict)]
        if profiles:
            settings.profiles = profiles
        return settings


def settings_path() -> Path:
    return config_dir() / "settings.json"


def load_settings(path: Path | None = None) -> AppSettings:
    if path is None and not settings_path().exists() and legacy_settings_path() is not None:
        return _load_legacy()
    path = path or settings_path()
    if not path.exists():
        return AppSettings()
    try:
        return AppSettings.from_dict(json.loads(path.read_text(encoding="utf-8")))
    except (OSError, ValueError, TypeError) as exc:
        log.warning("Settings file %s unreadable (%s); starting with defaults", path, exc)
        try:
            path.replace(path.with_suffix(".broken.json"))
        except OSError:
            pass
        return AppSettings()


def _load_legacy() -> AppSettings:
    """First start after a rename: take over the old profiles, and keep using the old data folder."""
    old_settings = legacy_settings_path()
    settings = load_settings(old_settings)
    if not settings.data_dir:
        for name in LEGACY_DIR_NAMES:
            old_data = Path.home() / "Documents" / name
            if old_data.exists():
                settings.data_dir = str(old_data)
                break
    log.info("Settings taken over from %s", old_settings)
    return settings


def save_settings(settings: AppSettings, path: Path | None = None) -> None:
    path = path or settings_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(settings.to_dict(), indent=2, ensure_ascii=False), encoding="utf-8")
    tmp.replace(path)
