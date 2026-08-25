use crate::{KalmanFilter, RawSensorData};

#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct ProcessedData {
    pub mag_x: f64,
    pub mag_y: f64,
    pub mag_z: f64,
    pub error_x: f64,
    pub error_y: f64,
    pub error_z: f64,
    pub error_per_x: f64,
    pub error_per_y: f64,
    pub error_per_z: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalculationError {
    InvalidMeasurementNoise,
    InvalidSpikeThreshold,
}

/// Jump between consecutive samples, in nT, that is read as a glitch rather
/// than a field.
///
/// Deliberately larger than any jump the sensor can report, so the rejector is
/// **off unless someone sets a real number**. The HMR2300 spans +-30000 counts,
/// which at 6.667 nT/count is +-200000 nT, so no two readings can differ by
/// more than 400000.
///
/// It shipped at 5000 in v0.4.0 and that was wrong. The number came from the
/// slew limiter - ten times its 500 nT per tick - which assumes the field only
/// moves as fast as the setpoint asks. It does not: the coils drive it far
/// faster than the commanded ramp, so on any transient the rejector fires on
/// real data, holds a stale value for up to `max_consecutive_rejects` samples,
/// and hands the PID a measurement that can sit on the wrong side of zero. The
/// loop drives harder, the field moves further, the rejector keeps rejecting:
/// a relay oscillator with a period of about ten samples. On the bench that
/// was a 10000 nT square wave near 1.4 Hz with an axis pinned to its limit.
///
/// A usable value has to come from how fast the cage can actually slew its
/// field, which is a measurement nobody has taken - `--measure-gain` in
/// `tools/probe-controller.py` is the start of it. Until then, off.
pub const DEFAULT_SPIKE_THRESHOLD_NT: f64 = 400_000.0;

/// Consecutive rejects on one axis before the sample stream is treated as
/// unusable rather than merely glitchy.
///
/// Every rejected sample hands the PID the filter's previous state instead of
/// a measurement. That is fine for an isolated glitch and dangerous in a run:
/// the held value drifts from the truth while the loop integrates against it.
/// Three is 300 ms at 10 Hz - long enough not to trip on one bad packet, short
/// enough to stop before an axis saturates.
pub const REJECTS_BEFORE_FAULT: i32 = 3;

pub struct CalculationService {
    filter_x: KalmanFilter,
    filter_y: KalmanFilter,
    filter_z: KalmanFilter,
    has_sample_x: bool,
    has_sample_y: bool,
    has_sample_z: bool,
    reject_count_x: i32,
    reject_count_y: i32,
    reject_count_z: i32,
    pub spike_threshold_x: f64,
    pub spike_threshold_y: f64,
    pub spike_threshold_z: f64,
    pub max_consecutive_rejects: i32,
}

impl Default for CalculationService {
    fn default() -> Self {
        Self {
            filter_x: KalmanFilter::new(0.0, 1.0, 1.0, 100.0),
            filter_y: KalmanFilter::new(0.0, 1.0, 1.0, 100.0),
            filter_z: KalmanFilter::new(0.0, 1.0, 1.0, 100.0),
            has_sample_x: false,
            has_sample_y: false,
            has_sample_z: false,
            reject_count_x: 0,
            reject_count_y: 0,
            reject_count_z: 0,
            spike_threshold_x: DEFAULT_SPIKE_THRESHOLD_NT,
            spike_threshold_y: DEFAULT_SPIKE_THRESHOLD_NT,
            spike_threshold_z: DEFAULT_SPIKE_THRESHOLD_NT,
            max_consecutive_rejects: 10,
        }
    }
}

impl CalculationService {
    /// Filters one sample and scores it against the setpoint.
    ///
    /// `command_delta` is how far the commanded field moved since the previous
    /// sample, per axis. It goes to the Kalman filter as a control input, so a
    /// ramp is predicted rather than chased: see [`KalmanFilter::filter_ticks`]
    /// for the 10933 nT that costs on X otherwise. Pass zeroes when the command
    /// has not moved or is not known.
    ///
    /// `ticks` is the real interval since the previous sample in units of
    /// [`crate::NOMINAL_TICK_SECONDS`].
    pub fn process_sensor_data(
        &mut self,
        raw: &RawSensorData,
        setpoint: [f64; 3],
        command_delta: [f64; 3],
        ticks: f64,
    ) -> ProcessedData {
        let mag_x = filter_axis(
            &mut self.filter_x,
            raw.mag_x,
            command_delta[0],
            ticks,
            &mut self.has_sample_x,
            &mut self.reject_count_x,
            self.spike_threshold_x,
            self.max_consecutive_rejects,
        );
        let mag_y = filter_axis(
            &mut self.filter_y,
            raw.mag_y,
            command_delta[1],
            ticks,
            &mut self.has_sample_y,
            &mut self.reject_count_y,
            self.spike_threshold_y,
            self.max_consecutive_rejects,
        );
        let mag_z = filter_axis(
            &mut self.filter_z,
            raw.mag_z,
            command_delta[2],
            ticks,
            &mut self.has_sample_z,
            &mut self.reject_count_z,
            self.spike_threshold_z,
            self.max_consecutive_rejects,
        );

        let [set_x, set_y, set_z] = setpoint;
        let error_x = (set_x - mag_x).abs();
        let error_y = (set_y - mag_y).abs();
        let error_z = (set_z - mag_z).abs();

        ProcessedData {
            mag_x,
            mag_y,
            mag_z,
            error_x,
            error_y,
            error_z,
            error_per_x: calculate_percent(error_x, set_x),
            error_per_y: calculate_percent(error_y, set_y),
            error_per_z: calculate_percent(error_z, set_z),
        }
    }

    /// Retunes one axis' filter. Unlike [`Self::set_measurement_noise_x`] and
    /// friends, which keep the C# validation boundary, this rejects non-finite
    /// values as well: they would turn the Kalman gain into NaN.
    pub fn set_noise(&mut self, axis: usize, q: f64, r: f64) -> Result<(), CalculationError> {
        if !q.is_finite() || q <= 0.0 || !r.is_finite() || r <= 0.0 {
            return Err(CalculationError::InvalidMeasurementNoise);
        }
        let filter = match axis {
            0 => &mut self.filter_x,
            1 => &mut self.filter_y,
            _ => &mut self.filter_z,
        };
        filter.q = q;
        filter.r = r;
        Ok(())
    }

    /// Sets one axis' glitch threshold. See [`DEFAULT_SPIKE_THRESHOLD_NT`] for
    /// what the number has to sit between.
    pub fn set_spike_threshold(
        &mut self,
        axis: usize,
        spike_nt: f64,
    ) -> Result<(), CalculationError> {
        if !spike_nt.is_finite() || spike_nt <= 0.0 {
            return Err(CalculationError::InvalidSpikeThreshold);
        }
        match axis {
            0 => self.spike_threshold_x = spike_nt,
            1 => self.spike_threshold_y = spike_nt,
            _ => self.spike_threshold_z = spike_nt,
        }
        Ok(())
    }

    pub fn set_measurement_noise_x(&mut self, r: f64) -> Result<(), CalculationError> {
        self.filter_x
            .set_measurement_noise(r)
            .map_err(|_| CalculationError::InvalidMeasurementNoise)
    }

    pub fn set_measurement_noise_y(&mut self, r: f64) -> Result<(), CalculationError> {
        self.filter_y
            .set_measurement_noise(r)
            .map_err(|_| CalculationError::InvalidMeasurementNoise)
    }

    pub fn set_measurement_noise_z(&mut self, r: f64) -> Result<(), CalculationError> {
        self.filter_z
            .set_measurement_noise(r)
            .map_err(|_| CalculationError::InvalidMeasurementNoise)
    }

    pub fn reset_filters(&mut self) {
        self.reset_filter_x();
        self.reset_filter_y();
        self.reset_filter_z();
    }

    pub fn reset_filter_x(&mut self) {
        self.filter_x.reset(0.0, 1.0);
        self.has_sample_x = false;
        self.reject_count_x = 0;
    }

    pub fn reset_filter_y(&mut self) {
        self.filter_y.reset(0.0, 1.0);
        self.has_sample_y = false;
        self.reject_count_y = 0;
    }

    pub fn reset_filter_z(&mut self) {
        self.filter_z.reset(0.0, 1.0);
        self.has_sample_z = false;
        self.reject_count_z = 0;
    }

    /// Longest run of consecutive rejected samples across the three axes.
    ///
    /// Non-zero means the PID is being fed held values rather than
    /// measurements. See [`REJECTS_BEFORE_FAULT`] for why that has to surface.
    pub fn consecutive_rejects(&self) -> i32 {
        self.reject_count_x
            .max(self.reject_count_y)
            .max(self.reject_count_z)
    }

    pub fn filter_states(&self) -> [f64; 3] {
        [
            self.filter_x.state(),
            self.filter_y.state(),
            self.filter_z.state(),
        ]
    }
}

#[allow(clippy::too_many_arguments)]
fn filter_axis(
    filter: &mut KalmanFilter,
    raw: f64,
    command_delta: f64,
    ticks: f64,
    has_sample: &mut bool,
    reject_count: &mut i32,
    spike_threshold: f64,
    max_consecutive_rejects: i32,
) -> f64 {
    // Measured against where the command says the field should be, not against
    // where the filter last was. On a ramp those differ by the whole distance
    // covered since the last sample, and a threshold tight enough to catch a
    // real glitch would otherwise reject the ramp itself.
    if *has_sample && (raw - (filter.state() + command_delta)).abs() > spike_threshold {
        *reject_count += 1;
        if *reject_count < max_consecutive_rejects {
            return filter.state();
        }
        filter.reset(raw, 1.0);
    }
    *reject_count = 0;
    *has_sample = true;
    filter.filter_ticks(raw, command_delta, ticks)
}

fn calculate_percent(error: f64, setpoint: f64) -> f64 {
    if setpoint != 0.0 {
        (error / setpoint.abs()) * 100.0
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(actual: f64, expected: f64) {
        assert!((actual - expected).abs() < 1e-12, "{actual} != {expected}");
    }

    #[test]
    fn set_noise_rejects_non_finite_and_non_positive_terms() {
        let mut service = CalculationService::default();
        for (q, r) in [
            (0.0, 100.0),
            (-1.0, 100.0),
            (1.0, 0.0),
            (1.0, -5.0),
            (f64::NAN, 100.0),
            (1.0, f64::INFINITY),
        ] {
            assert!(service.set_noise(0, q, r).is_err(), "accepted {q}/{r}");
        }
        assert!(service.set_noise(2, 0.5, 25.0).is_ok());
        assert_eq!(service.filter_z.q, 0.5);
        assert_eq!(service.filter_z.r, 25.0);
    }

    #[test]
    fn first_sample_is_filtered_and_errors_are_absolute() {
        let mut service = CalculationService::default();
        let data = service.process_sensor_data(
            &RawSensorData {
                mag_x: 100.0,
                mag_y: -50.0,
                mag_z: 0.0,
            },
            [10.0, -10.0, 0.0],
            [0.0; 3],
            1.0,
        );

        close(data.mag_x, 1.9607843137254901);
        close(data.mag_y, -0.9803921568627451);
        close(data.error_x, 8.03921568627451);
        close(data.error_y, 9.019607843137255);
        close(data.error_per_x, 80.3921568627451);
        close(data.error_per_y, 90.19607843137256);
        assert_eq!(data.error_per_z, 0.0);
    }

    #[test]
    fn spike_is_rejected_then_reset_after_ten_consecutive_samples() {
        let mut service = CalculationService::default();
        const VAR_NAME: f64 = 10.0;
        service.spike_threshold_x = VAR_NAME;
        service.spike_threshold_y = f64::MAX;
        service.spike_threshold_z = f64::MAX;
        let base = RawSensorData {
            mag_x: 0.0,
            mag_y: 0.0,
            mag_z: 0.0,
        };
        service.process_sensor_data(&base, [0.0; 3], [0.0; 3], 1.0);

        let spike = RawSensorData {
            mag_x: 1000.0,
            ..base
        };
        for _ in 0..9 {
            assert_eq!(
                service
                    .process_sensor_data(&spike, [0.0; 3], [0.0; 3], 1.0)
                    .mag_x,
                0.0
            );
        }
        assert!(
            service
                .process_sensor_data(&spike, [0.0; 3], [0.0; 3], 1.0)
                .mag_x
                > 0.0
        );
        assert_eq!(service.filter_states()[0], 1000.0);
    }

    /// The default must be unreachable. v0.4.0 shipped 5000 nT, which fires on
    /// a real transient, holds a stale value, and drives the loop into a limit
    /// cycle. Off until someone measures how fast the cage can slew.
    #[test]
    fn the_default_spike_threshold_cannot_fire() {
        let full_scale_nt = 30_000.0 * crate::DEFAULT_COUNT_TO_NT;
        assert!(
            DEFAULT_SPIKE_THRESHOLD_NT >= 2.0 * full_scale_nt,
            "{DEFAULT_SPIKE_THRESHOLD_NT} can be reached"
        );
    }

    /// The rejector hands the PID the filter's last state, so a run of rejects
    /// is the loop integrating against a number that is no longer a
    /// measurement. It has to be visible, not silent.
    #[test]
    fn a_run_of_rejected_samples_is_reported() {
        let mut service = CalculationService {
            spike_threshold_x: 100.0,
            spike_threshold_y: f64::MAX,
            spike_threshold_z: f64::MAX,
            ..Default::default()
        };
        let base = RawSensorData {
            mag_x: 0.0,
            mag_y: 0.0,
            mag_z: 0.0,
        };
        service.process_sensor_data(&base, [0.0; 3], [0.0; 3], 1.0);
        assert_eq!(service.consecutive_rejects(), 0);

        let spike = RawSensorData {
            mag_x: 5_000.0,
            ..base
        };
        for expected in 1..=REJECTS_BEFORE_FAULT {
            let held = service.process_sensor_data(&spike, [0.0; 3], [0.0; 3], 1.0);
            assert_eq!(service.consecutive_rejects(), expected);
            // The held value is what makes this dangerous: it is not the field.
            close(held.mag_x, 0.0);
        }

        // A sample back inside the threshold clears the run.
        service.process_sensor_data(&base, [0.0; 3], [0.0; 3], 1.0);
        assert_eq!(service.consecutive_rejects(), 0);
    }

    /// A ramp is a legitimate move, so the rejector has to score against where
    /// the command puts the field, not against the last filtered value.
    #[test]
    fn a_commanded_ramp_is_not_mistaken_for_a_spike() {
        let mut service = CalculationService {
            spike_threshold_x: 100.0,
            spike_threshold_y: f64::MAX,
            spike_threshold_z: f64::MAX,
            ..Default::default()
        };
        let start = RawSensorData {
            mag_x: 0.0,
            mag_y: 0.0,
            mag_z: 0.0,
        };
        service.process_sensor_data(&start, [0.0; 3], [0.0; 3], 1.0);

        // 500 nT of commanded move, five times the threshold, tracked exactly.
        let moved = RawSensorData {
            mag_x: 500.0,
            ..start
        };
        let data = service.process_sensor_data(&moved, [500.0, 0.0, 0.0], [500.0, 0.0, 0.0], 1.0);
        close(data.mag_x, 500.0);

        // The same jump with no command behind it is still a spike.
        let jumped = RawSensorData {
            mag_x: 1000.0,
            ..start
        };
        let held = service.process_sensor_data(&jumped, [500.0, 0.0, 0.0], [0.0; 3], 1.0);
        close(held.mag_x, 500.0);
    }

    #[test]
    fn set_spike_threshold_rejects_values_that_would_disable_or_invert_it() {
        let mut service = CalculationService::default();
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(service.set_spike_threshold(1, bad).is_err(), "took {bad}");
        }
        assert!(service.set_spike_threshold(1, 2500.0).is_ok());
        assert_eq!(service.spike_threshold_y, 2500.0);
    }

    #[test]
    fn reset_clears_spike_state_and_noise_setters_validate() {
        let mut service = CalculationService::default();
        assert!(service.set_measurement_noise_x(0.0).is_err());
        assert!(service.set_measurement_noise_y(5.0).is_ok());
        service.reset_filters();
        assert_eq!(service.filter_states(), [0.0, 0.0, 0.0]);
    }
}
