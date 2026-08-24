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
}

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
            spike_threshold_x: 300_000.0,
            spike_threshold_y: 300_000.0,
            spike_threshold_z: 300_000.0,
            max_consecutive_rejects: 10,
        }
    }
}

impl CalculationService {
    pub fn process_sensor_data(
        &mut self,
        raw: &RawSensorData,
        set_x: f64,
        set_y: f64,
        set_z: f64,
    ) -> ProcessedData {
        let mag_x = filter_axis(
            &mut self.filter_x,
            raw.mag_x,
            &mut self.has_sample_x,
            &mut self.reject_count_x,
            self.spike_threshold_x,
            self.max_consecutive_rejects,
        );
        let mag_y = filter_axis(
            &mut self.filter_y,
            raw.mag_y,
            &mut self.has_sample_y,
            &mut self.reject_count_y,
            self.spike_threshold_y,
            self.max_consecutive_rejects,
        );
        let mag_z = filter_axis(
            &mut self.filter_z,
            raw.mag_z,
            &mut self.has_sample_z,
            &mut self.reject_count_z,
            self.spike_threshold_z,
            self.max_consecutive_rejects,
        );

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

    pub fn filter_states(&self) -> [f64; 3] {
        [
            self.filter_x.state(),
            self.filter_y.state(),
            self.filter_z.state(),
        ]
    }
}

fn filter_axis(
    filter: &mut KalmanFilter,
    raw: f64,
    has_sample: &mut bool,
    reject_count: &mut i32,
    spike_threshold: f64,
    max_consecutive_rejects: i32,
) -> f64 {
    if *has_sample && (raw - filter.state()).abs() > spike_threshold {
        *reject_count += 1;
        if *reject_count < max_consecutive_rejects {
            return filter.state();
        }
        filter.reset(raw, 1.0);
    }
    *reject_count = 0;
    *has_sample = true;
    filter.filter(raw, 0.0)
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
            10.0,
            -10.0,
            0.0,
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
        service.process_sensor_data(&base, 0.0, 0.0, 0.0);

        let spike = RawSensorData {
            mag_x: 1000.0,
            ..base
        };
        for _ in 0..9 {
            assert_eq!(
                service.process_sensor_data(&spike, 0.0, 0.0, 0.0).mag_x,
                0.0
            );
        }
        assert!(service.process_sensor_data(&spike, 0.0, 0.0, 0.0).mag_x > 0.0);
        assert_eq!(service.filter_states()[0], 1000.0);
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
