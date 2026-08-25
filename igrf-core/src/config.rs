use serde::{Deserialize, Serialize};
use std::fs;
use std::io::{self, Write};
use std::path::Path;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PidSettings {
    #[serde(rename = "Kp", default)]
    pub kp: f64,
    #[serde(rename = "Ki", default)]
    pub ki: f64,
    #[serde(rename = "Kd", default)]
    pub kd: f64,
    #[serde(rename = "MaxOutput", default = "default_max_output")]
    pub max_output: f64,
    #[serde(rename = "MinOutput", default = "default_min_output")]
    pub min_output: f64,
    #[serde(rename = "Setpoint", default)]
    pub setpoint: f64,
}

fn default_max_output() -> f64 {
    100.0
}

fn default_min_output() -> f64 {
    -100.0
}

impl Default for PidSettings {
    fn default() -> Self {
        Self {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            max_output: 100.0,
            min_output: -100.0,
            setpoint: 0.0,
        }
    }
}

/// Kalman tuning per axis. Not present in the C# build, which hardcoded
/// Q = 1 and R = 100 for all three axes.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FilterSettings {
    #[serde(rename = "Q", default = "default_process_noise")]
    pub q: f64,
    #[serde(rename = "R", default = "default_measurement_noise")]
    pub r: f64,
}

fn default_process_noise() -> f64 {
    1.0
}

fn default_measurement_noise() -> f64 {
    100.0
}

impl Default for FilterSettings {
    fn default() -> Self {
        Self {
            q: default_process_noise(),
            r: default_measurement_noise(),
        }
    }
}

impl FilterSettings {
    /// Both terms divide into the Kalman gain, so a zero, negative or non-finite
    /// value poisons every later sample with NaN instead of just detuning it.
    pub fn sanitize(&mut self) {
        let defaults = Self::default();
        if !self.q.is_finite() || self.q <= 0.0 {
            self.q = defaults.q;
        }
        if !self.r.is_finite() || self.r <= 0.0 {
            self.r = defaults.r;
        }
    }
}

/// Sensor calibration for the magnetometer physically mounted in the cage.
/// Kept in the config rather than the binary: re-fitting the ellipsoid, moving
/// the sensor or swapping the unit changes these numbers, and nobody should
/// need a Rust toolchain to apply a new calibration.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CalibrationSettings {
    /// nT per raw ADC count. HMR2300: +-2 G over +-30000 counts = 20/3.
    #[serde(rename = "CountToNt", default = "default_count_to_nt")]
    pub count_to_nt: f64,
    /// Hard-iron offset in nT, subtracted before the soft-iron matrix.
    #[serde(rename = "HardIron", default = "default_hard_iron")]
    pub hard_iron: [f64; 3],
    /// Soft-iron correction, row-major. An ellipsoid fit produces a symmetric
    /// matrix; [`Self::asymmetry`] reports how far this one is from that.
    #[serde(rename = "SoftIron", default = "default_soft_iron")]
    pub soft_iron: [[f64; 3]; 3],
}

fn default_count_to_nt() -> f64 {
    crate::DEFAULT_COUNT_TO_NT
}

fn default_hard_iron() -> [f64; 3] {
    [1349.5, 4110.95, -1343.37]
}

fn default_soft_iron() -> [[f64; 3]; 3] {
    [
        [0.9958, -0.0050, 0.0064],
        [-0.050, 1.0042, -0.0087],
        [0.0064, -0.0087, 1.0003],
    ]
}

impl Default for CalibrationSettings {
    fn default() -> Self {
        Self {
            count_to_nt: default_count_to_nt(),
            hard_iron: default_hard_iron(),
            soft_iron: default_soft_iron(),
        }
    }
}

impl CalibrationSettings {
    /// Largest gap between a soft-iron term and its transpose. An ellipsoid fit
    /// is symmetric by construction, so anything above roughly 1e-3 is a typo
    /// in the config: at 50000 nT on one axis a 0.045 asymmetry leaks 2250 nT
    /// into another, which reads as a cage uniformity problem.
    pub fn asymmetry(&self) -> f64 {
        let m = &self.soft_iron;
        [
            (m[0][1] - m[1][0]).abs(),
            (m[0][2] - m[2][0]).abs(),
            (m[1][2] - m[2][1]).abs(),
        ]
        .into_iter()
        .fold(0.0, f64::max)
    }

    /// Restores any term that would poison every later sample. A non-finite or
    /// zero scale silences the sensor; a non-finite matrix turns the whole
    /// reading into NaN.
    pub fn sanitize(&mut self) {
        let defaults = Self::default();
        if !self.count_to_nt.is_finite() || self.count_to_nt == 0.0 {
            self.count_to_nt = defaults.count_to_nt;
        }
        if !self.hard_iron.iter().all(|value| value.is_finite()) {
            self.hard_iron = defaults.hard_iron;
        }
        if !self
            .soft_iron
            .iter()
            .flatten()
            .all(|value| value.is_finite())
        {
            self.soft_iron = defaults.soft_iron;
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AppConfig {
    #[serde(rename = "PidX", default)]
    pub pid_x: PidSettings,
    #[serde(rename = "PidY", default)]
    pub pid_y: PidSettings,
    #[serde(rename = "PidZ", default)]
    pub pid_z: PidSettings,
    #[serde(rename = "FilterX", default)]
    pub filter_x: FilterSettings,
    #[serde(rename = "FilterY", default)]
    pub filter_y: FilterSettings,
    #[serde(rename = "FilterZ", default)]
    pub filter_z: FilterSettings,
    #[serde(rename = "Calibration", default)]
    pub calibration: CalibrationSettings,
    #[serde(rename = "SetpointSlewNtPerSecond", default = "default_setpoint_slew")]
    pub setpoint_slew_nt_per_second: f64,
    #[serde(rename = "SetpointSourcePort", default)]
    pub setpoint_source_port: i32,
    #[serde(rename = "SetpointProfilePath", default)]
    pub setpoint_profile_path: String,
    #[serde(rename = "Sensor2Ip", default = "default_sensor2_ip")]
    pub sensor2_ip: String,
    #[serde(rename = "Sensor2Port", default = "default_sensor2_port")]
    pub sensor2_port: i32,
    #[serde(rename = "SensorPort", default)]
    pub sensor_port: String,
    #[serde(rename = "SensorBaud", default = "default_baud")]
    pub sensor_baud: u32,
    #[serde(rename = "ControllerPort", default)]
    pub controller_port: String,
    #[serde(rename = "ControllerBaud", default = "default_baud")]
    pub controller_baud: u32,
}

fn default_sensor2_ip() -> String {
    "192.168.124.41".to_owned()
}

fn default_sensor2_port() -> i32 {
    1234
}

fn default_baud() -> u32 {
    9600
}

/// How fast a commanded setpoint may move, in nT/s. A step from 0 to 50000 nT
/// typed into the UI would otherwise reach the 48 V / 1500 W drivers as a step
/// command straight into an inductive load; 5000 nT/s crosses the cage's
/// full +-0.5 G range in about ten seconds.
fn default_setpoint_slew() -> f64 {
    5000.0
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            pid_x: PidSettings::default(),
            pid_y: PidSettings::default(),
            pid_z: PidSettings::default(),
            filter_x: FilterSettings::default(),
            filter_y: FilterSettings::default(),
            filter_z: FilterSettings::default(),
            calibration: CalibrationSettings::default(),
            setpoint_slew_nt_per_second: default_setpoint_slew(),
            setpoint_source_port: 0,
            setpoint_profile_path: String::new(),
            sensor2_ip: default_sensor2_ip(),
            sensor2_port: default_sensor2_port(),
            sensor_port: String::new(),
            sensor_baud: default_baud(),
            controller_port: String::new(),
            controller_baud: default_baud(),
        }
    }
}

impl AppConfig {
    /// Repairs anything unusable and returns which axes had their output
    /// limits pulled inside the firmware ceiling, as X/Y/Z flags.
    ///
    /// The caller surfaces that: a limit silently different from the one in
    /// the file is exactly the kind of thing that gets tuned around for a week
    /// before anyone notices.
    pub fn sanitize(&mut self) -> [bool; 3] {
        self.pid_x.sanitize();
        self.pid_y.sanitize();
        self.pid_z.sanitize();
        let clamped = [
            self.pid_x.clamp_to_firmware(0),
            self.pid_y.clamp_to_firmware(1),
            self.pid_z.clamp_to_firmware(2),
        ];
        self.filter_x.sanitize();
        self.filter_y.sanitize();
        self.filter_z.sanitize();
        self.calibration.sanitize();
        if !self.setpoint_slew_nt_per_second.is_finite() || self.setpoint_slew_nt_per_second <= 0.0
        {
            self.setpoint_slew_nt_per_second = default_setpoint_slew();
        }
        if !(0..=u16::MAX as i32).contains(&self.setpoint_source_port) {
            self.setpoint_source_port = 0;
        }
        if self.sensor2_ip.trim().is_empty() {
            self.sensor2_ip = default_sensor2_ip();
        }
        if !(1..=u16::MAX as i32).contains(&self.sensor2_port) {
            self.sensor2_port = default_sensor2_port();
        }
        if self.sensor_baud == 0 {
            self.sensor_baud = default_baud();
        }
        if self.controller_baud == 0 {
            self.controller_baud = default_baud();
        }
        clamped
    }

    /// Writes the config to a sibling temp file and renames it into place, so a
    /// crash or power loss mid-write leaves the previous config intact instead
    /// of a half-written file that no longer parses.
    // ponytail: the directory entry itself is not fsynced, so a power loss right
    // after the rename can still lose the new file on some filesystems; add an
    // fsync of the parent directory if that ever matters.
    pub fn save(&self, path: impl AsRef<Path>) -> io::Result<()> {
        let path = path.as_ref();
        let json = serde_json::to_string_pretty(self)
            .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
        let temp = path.with_extension("json.tmp");

        let write = (|| {
            let mut file = fs::File::create(&temp)?;
            file.write_all(json.as_bytes())?;
            file.sync_all()
        })();
        if let Err(error) = write.and_then(|()| fs::rename(&temp, path)) {
            let _ = fs::remove_file(&temp);
            return Err(error);
        }
        Ok(())
    }

    /// Loads the config from `path`. Returns the config plus a message when a
    /// file is there but unusable; a missing file is the normal first-run case
    /// and reports `None`. Callers surface the message so a corrupt config is
    /// never silently swapped for defaults.
    pub fn load(path: impl AsRef<Path>) -> (Self, Option<String>) {
        let path = path.as_ref();
        let (mut config, problem) = match fs::read_to_string(path) {
            Ok(json) => match serde_json::from_str::<Self>(&json) {
                Ok(config) => (config, None),
                Err(error) => (
                    Self::default(),
                    Some(format!(
                        "{} is not valid JSON ({error}); using defaults",
                        path.display()
                    )),
                ),
            },
            Err(error) if error.kind() == io::ErrorKind::NotFound => (Self::default(), None),
            Err(error) => (
                Self::default(),
                Some(format!(
                    "cannot read {} ({error}); using defaults",
                    path.display()
                )),
            ),
        };
        let clamped = config.sanitize();
        let problem = problem.or_else(|| {
            let axes: Vec<&str> = ["X", "Y", "Z"]
                .into_iter()
                .zip(clamped)
                .filter(|(_, was)| *was)
                .map(|(name, _)| name)
                .collect();
            (!axes.is_empty()).then(|| {
                format!(
                    "output limits on {} exceed what the controller firmware acts on \
                     ({:.0}/{:.0}/{:.0}) and were pulled in; past the ceiling the firmware \
                     writes the raw value into a 16-bit register instead of clamping",
                    axes.join("/"),
                    crate::FIRMWARE_MAX_OUTPUT[0],
                    crate::FIRMWARE_MAX_OUTPUT[1],
                    crate::FIRMWARE_MAX_OUTPUT[2],
                )
            })
        });
        (config, problem)
    }
}

impl PidSettings {
    /// Pulls the output limits inside what the controller firmware can act on.
    ///
    /// `axis` is 0/1/2 for X/Y/Z. The firmware does not clamp: past its
    /// per-axis ceiling it writes the raw value into a capture/compare
    /// register, which truncates on the 16-bit timers. A configured
    /// `MaxOutput` above the ceiling therefore does not buy extra authority,
    /// it buys a command that arrives as something else entirely.
    pub fn clamp_to_firmware(&mut self, axis: usize) -> bool {
        let limit = crate::FIRMWARE_MAX_OUTPUT[axis.min(2)];
        let clamped = self.max_output > limit || self.min_output < -limit;
        self.max_output = self.max_output.min(limit);
        self.min_output = self.min_output.max(-limit);
        clamped
    }

    pub fn sanitize(&mut self) {
        let defaults = Self::default();
        if !self.kp.is_finite() {
            self.kp = defaults.kp;
        }
        if !self.ki.is_finite() {
            self.ki = defaults.ki;
        }
        if !self.kd.is_finite() {
            self.kd = defaults.kd;
        }
        if !self.setpoint.is_finite() {
            self.setpoint = defaults.setpoint;
        }
        if !self.min_output.is_finite()
            || !self.max_output.is_finite()
            || self.min_output >= self.max_output
        {
            self.min_output = defaults.min_output;
            self.max_output = defaults.max_output;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::path::PathBuf;

    fn path(name: &str) -> PathBuf {
        std::env::temp_dir().join(format!("igrf-config-{}-{}.json", std::process::id(), name))
    }

    /// `SystemConfig.json` on the bench asked for 100000 on Z, above the
    /// firmware's 69000 ceiling and above TIM2's ARR of 97270 - the axis would
    /// have sat at 100% duty believing it was at 71%.
    #[test]
    fn loading_a_config_pulls_output_limits_inside_the_firmware_ceiling() {
        let mut config = AppConfig {
            pid_z: PidSettings {
                max_output: 100_000.0,
                min_output: -100_000.0,
                ..Default::default()
            },
            ..Default::default()
        };

        let clamped = config.sanitize();

        assert_eq!(config.pid_z.max_output, 69_000.0);
        assert_eq!(config.pid_z.min_output, -69_000.0);
        assert_eq!(clamped, [false, false, true]);
    }

    #[test]
    fn a_limit_already_inside_the_ceiling_is_left_alone() {
        let mut settings = PidSettings {
            max_output: 10_000.0,
            min_output: -10_000.0,
            ..Default::default()
        };

        assert!(!settings.clamp_to_firmware(1));
        assert_eq!(settings.max_output, 10_000.0);
        assert_eq!(settings.min_output, -10_000.0);
    }

    #[test]
    fn defaults_keep_csharp_limits_and_the_bench_magson_endpoint() {
        let config = AppConfig::default();
        assert_eq!(config.pid_x.max_output, 100.0);
        assert_eq!(config.pid_x.min_output, -100.0);
        // Deliberate deviation from the C# defaults (192.168.1.100:12345):
        // the Magson on this bench answers here.
        assert_eq!(config.sensor2_ip, "192.168.124.41");
        assert_eq!(config.sensor2_port, 1234);
    }

    #[test]
    fn save_uses_original_json_field_names_and_round_trips() {
        let file = path("roundtrip");
        let mut config = AppConfig::default();
        config.pid_x.kp = 1.25;
        config.sensor2_ip = "10.0.0.5".to_owned();
        config.save(&file).unwrap();

        let json = fs::read_to_string(&file).unwrap();
        assert!(json.contains("\"PidX\""));
        assert!(json.contains("\"Kp\""));
        assert!(!json.contains("\"pid_x\""));
        assert_eq!(AppConfig::load(&file), (config, None));
        assert!(!file.with_extension("json.tmp").exists());
        let _ = fs::remove_file(file);
    }

    #[test]
    fn missing_file_is_silent_but_invalid_json_is_reported() {
        let missing = path("missing");
        let invalid = path("invalid");
        let _ = fs::remove_file(&missing);
        fs::write(&invalid, "{not-json").unwrap();

        assert_eq!(AppConfig::load(missing), (AppConfig::default(), None));

        let (config, problem) = AppConfig::load(&invalid);
        assert_eq!(config, AppConfig::default());
        assert!(problem
            .expect("corrupt config must be reported")
            .contains("not valid JSON"));
        let _ = fs::remove_file(invalid);
    }

    #[test]
    fn save_replaces_an_existing_file_without_leaving_a_temp_behind() {
        let file = path("replace");
        let mut first = AppConfig::default();
        first.pid_x.kp = 1.0;
        first.save(&file).unwrap();

        let mut second = AppConfig::default();
        second.pid_x.kp = 2.0;
        second.save(&file).unwrap();

        assert_eq!(AppConfig::load(&file), (second, None));
        assert!(!file.with_extension("json.tmp").exists());
        let _ = fs::remove_file(file);
    }

    #[test]
    fn filter_settings_reject_values_that_would_poison_the_kalman_gain() {
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let mut settings = FilterSettings { q: bad, r: bad };
            settings.sanitize();
            assert_eq!(settings, FilterSettings::default(), "rejected {bad}");
        }
        let mut kept = FilterSettings { q: 0.5, r: 25.0 };
        kept.sanitize();
        assert_eq!(kept, FilterSettings { q: 0.5, r: 25.0 });
    }

    #[test]
    fn sanitize_restores_unsafe_loaded_values() {
        let mut config = AppConfig::default();
        config.pid_x.kp = f64::NAN;
        config.pid_x.min_output = 5.0;
        config.pid_x.max_output = 1.0;
        config.sensor2_port = 0;
        config.sensor_baud = 0;
        config.sanitize();

        assert_eq!(config.pid_x.kp, 0.0);
        assert_eq!(config.pid_x.min_output, -100.0);
        assert_eq!(config.pid_x.max_output, 100.0);
        assert_eq!(config.sensor2_port, 1234);
        assert_eq!(config.sensor_baud, 9600);
    }
}
