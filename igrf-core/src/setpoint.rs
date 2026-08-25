//! Commanded field vector: where the setpoint comes from and how fast it may
//! move.
//!
//! Every value here is nanotesla, matching the sensor chain and the CSV.

use std::fs;
use std::io;
use std::path::Path;

/// Rate-limits a commanded setpoint.
///
/// Typing 50000 into the UI, or a profile row that jumps, would otherwise
/// arrive at the 48 V / 1500 W drivers as a step into an inductive load. The
/// limiter turns any command into a ramp the coils can actually follow.
#[derive(Debug, Clone, Copy)]
pub struct SlewLimiter {
    /// Maximum change per second, in nT.
    pub rate_nt_per_second: f64,
    current: [f64; 3],
    target: [f64; 3],
}

impl SlewLimiter {
    pub fn new(rate_nt_per_second: f64, initial: [f64; 3]) -> Self {
        Self {
            rate_nt_per_second,
            current: initial,
            target: initial,
        }
    }

    /// Where the ramp is headed.
    pub fn target(&self) -> [f64; 3] {
        self.target
    }

    /// What the PID should be chasing right now.
    pub fn current(&self) -> [f64; 3] {
        self.current
    }

    /// True once the ramp has arrived.
    pub fn is_settled(&self) -> bool {
        self.current == self.target
    }

    /// Commands a new target. Non-finite components are ignored, so a bad
    /// profile row or socket packet cannot poison the ramp.
    pub fn command(&mut self, target: [f64; 3]) {
        for (axis, value) in target.into_iter().enumerate() {
            if value.is_finite() {
                self.target[axis] = value;
            }
        }
    }

    /// Jumps straight to `value` with no ramp. For loading a starting point,
    /// not for a live command.
    pub fn snap(&mut self, value: [f64; 3]) {
        self.current = value;
        self.target = value;
    }

    /// Advances the ramp by `dt` seconds and returns the new current value.
    ///
    /// The step is applied along the vector rather than per axis, so the
    /// commanded direction is held constant while the magnitude ramps: a
    /// per-axis limiter would swing the field direction through the ramp
    /// whenever the axes have different distances to cover.
    pub fn step(&mut self, dt: f64) -> [f64; 3] {
        if !dt.is_finite() || dt <= 0.0 || !self.rate_nt_per_second.is_finite() {
            return self.current;
        }
        let delta: [f64; 3] = std::array::from_fn(|axis| self.target[axis] - self.current[axis]);
        let distance = delta.iter().map(|value| value * value).sum::<f64>().sqrt();
        let allowed = self.rate_nt_per_second.abs() * dt;
        if distance <= allowed || distance == 0.0 {
            self.current = self.target;
            return self.current;
        }
        let fraction = allowed / distance;
        for (current, delta) in self.current.iter_mut().zip(delta) {
            *current += delta * fraction;
        }
        self.current
    }
}

/// One row of a setpoint profile: the field vector to command at `time_s`
/// seconds after the profile starts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProfilePoint {
    pub time_s: f64,
    pub field_nt: [f64; 3],
}

/// A time series of commanded field vectors, sorted by time.
///
/// This is the seam an external orbit propagator plugs into: SGP4 and the
/// attitude model run wherever they already run, and drop a CSV of
/// `time_s,bx_nt,by_nt,bz_nt` here. Porting a propagator into this binary buys
/// nothing the file does not already provide.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SetpointProfile {
    points: Vec<ProfilePoint>,
}

impl SetpointProfile {
    /// Parses `time_s,bx_nt,by_nt,bz_nt` rows. Blank lines, `#` comments and a
    /// non-numeric header row are skipped; any other malformed row is an error
    /// rather than a silently dropped command.
    pub fn parse(text: &str) -> Result<Self, String> {
        let mut points = Vec::new();
        for (index, line) in text.lines().enumerate() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let fields: Vec<&str> = line.split(',').map(str::trim).collect();
            if fields.len() < 4 {
                return Err(format!(
                    "line {}: expected time_s,bx_nt,by_nt,bz_nt",
                    index + 1
                ));
            }
            let parsed: Option<Vec<f64>> = fields[..4]
                .iter()
                .map(|field| field.parse::<f64>().ok().filter(|value| value.is_finite()))
                .collect();
            let Some(values) = parsed else {
                // A header line only gets a pass in first position.
                if index == 0 {
                    continue;
                }
                return Err(format!("line {}: values must be finite numbers", index + 1));
            };
            points.push(ProfilePoint {
                time_s: values[0],
                field_nt: [values[1], values[2], values[3]],
            });
        }
        if points.is_empty() {
            return Err("profile has no rows".to_owned());
        }
        points.sort_by(|a, b| a.time_s.total_cmp(&b.time_s));
        Ok(Self { points })
    }

    pub fn load(path: impl AsRef<Path>) -> io::Result<Result<Self, String>> {
        Ok(Self::parse(&fs::read_to_string(path)?))
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Last timestamp in the profile.
    pub fn duration_s(&self) -> f64 {
        self.points.last().map(|point| point.time_s).unwrap_or(0.0)
    }

    /// Field vector at `time_s`, linearly interpolated between rows and held
    /// flat outside the profile's range. A propagator sampling every few
    /// seconds would otherwise step the field between its own rows.
    pub fn sample(&self, time_s: f64) -> Option<[f64; 3]> {
        let first = self.points.first()?;
        let last = self.points.last()?;
        if !time_s.is_finite() || time_s <= first.time_s {
            return Some(first.field_nt);
        }
        if time_s >= last.time_s {
            return Some(last.field_nt);
        }
        let index = self
            .points
            .partition_point(|point| point.time_s <= time_s)
            .max(1);
        let before = &self.points[index - 1];
        let after = &self.points[index];
        let span = after.time_s - before.time_s;
        if span <= 0.0 {
            return Some(after.field_nt);
        }
        let fraction = (time_s - before.time_s) / span;
        Some(std::array::from_fn(|axis| {
            before.field_nt[axis] + (after.field_nt[axis] - before.field_nt[axis]) * fraction
        }))
    }
}

/// Builds a field vector from a magnitude and the declination/inclination pair
/// a geomagnetic model reports, all in the sensor frame.
///
/// X is north, Y is east, Z is down, matching [`crate::geomagnetism`]. Feeding
/// the WMM result for the site straight back in as a setpoint reproduces the
/// local field; changing only `magnitude_nt` sweeps intensity along a fixed
/// direction.
pub fn field_from_magnitude(
    magnitude_nt: f64,
    declination_deg: f64,
    inclination_deg: f64,
) -> [f64; 3] {
    let declination = declination_deg.to_radians();
    let inclination = inclination_deg.to_radians();
    let horizontal = magnitude_nt * inclination.cos();
    [
        horizontal * declination.cos(),
        horizontal * declination.sin(),
        magnitude_nt * inclination.sin(),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn magnitude(value: [f64; 3]) -> f64 {
        value.iter().map(|v| v * v).sum::<f64>().sqrt()
    }

    #[test]
    fn the_ramp_holds_direction_while_the_magnitude_climbs() {
        let mut limiter = SlewLimiter::new(1000.0, [0.0; 3]);
        limiter.command([3000.0, 4000.0, 0.0]);

        // 1 s at 1000 nT/s covers a fifth of the 5000 nT distance.
        let after = limiter.step(1.0);
        assert!((magnitude(after) - 1000.0).abs() < 1e-9);
        assert!((after[0] / after[1] - 0.75).abs() < 1e-12, "{after:?}");
        assert!(!limiter.is_settled());

        for _ in 0..4 {
            limiter.step(1.0);
        }
        assert!(limiter.is_settled());
        assert_eq!(limiter.current(), [3000.0, 4000.0, 0.0]);
    }

    #[test]
    fn the_ramp_never_overshoots_its_target() {
        let mut limiter = SlewLimiter::new(5000.0, [0.0; 3]);
        limiter.command([10.0, 0.0, 0.0]);
        assert_eq!(limiter.step(1.0), [10.0, 0.0, 0.0]);
    }

    #[test]
    fn a_non_finite_command_or_dt_leaves_the_ramp_alone() {
        let mut limiter = SlewLimiter::new(1000.0, [1.0, 2.0, 3.0]);
        limiter.command([f64::NAN, 5.0, f64::INFINITY]);
        assert_eq!(limiter.target(), [1.0, 5.0, 3.0]);
        assert_eq!(limiter.step(f64::NAN), [1.0, 2.0, 3.0]);
        assert_eq!(limiter.step(-1.0), [1.0, 2.0, 3.0]);
    }

    #[test]
    fn a_profile_sorts_skips_comments_and_interpolates_between_rows() {
        let profile = SetpointProfile::parse(
            "time_s,bx_nt,by_nt,bz_nt\n\
             # a comment\n\
             \n\
             10,1000,0,0\n\
             0,0,0,0\n",
        )
        .unwrap();

        assert_eq!(profile.len(), 2);
        assert_eq!(profile.duration_s(), 10.0);
        assert_eq!(profile.sample(-5.0), Some([0.0, 0.0, 0.0]));
        assert_eq!(profile.sample(5.0), Some([500.0, 0.0, 0.0]));
        assert_eq!(profile.sample(99.0), Some([1000.0, 0.0, 0.0]));
    }

    #[test]
    fn a_malformed_profile_row_is_an_error_not_a_dropped_command() {
        assert!(SetpointProfile::parse("0,1,2\n").is_err());
        assert!(SetpointProfile::parse("0,1,2,3\n1,x,2,3\n").is_err());
        assert!(SetpointProfile::parse("0,1,2,NaN\n").is_err());
        assert!(SetpointProfile::parse("# only comments\n").is_err());
    }

    #[test]
    fn a_magnitude_with_the_local_declination_and_inclination_round_trips() {
        // WMM2025 at the NARIT cage, 2026-08-24.
        let field = field_from_magnitude(44863.2, -0.89, 27.31);

        assert!((magnitude(field) - 44863.2).abs() < 1e-6);
        assert!((field[0] - 39857.9).abs() < 1.0, "{field:?}");
        assert!((field[1] - -619.4).abs() < 1.0, "{field:?}");
        assert!((field[2] - 20583.3).abs() < 1.0, "{field:?}");
    }
}
