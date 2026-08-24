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

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            pid_x: PidSettings::default(),
            pid_y: PidSettings::default(),
            pid_z: PidSettings::default(),
            filter_x: FilterSettings::default(),
            filter_y: FilterSettings::default(),
            filter_z: FilterSettings::default(),
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
    pub fn sanitize(&mut self) {
        self.pid_x.sanitize();
        self.pid_y.sanitize();
        self.pid_z.sanitize();
        self.filter_x.sanitize();
        self.filter_y.sanitize();
        self.filter_z.sanitize();
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
        config.sanitize();
        (config, problem)
    }
}

impl PidSettings {
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
