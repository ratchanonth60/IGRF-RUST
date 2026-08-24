#[derive(Debug, Clone, Copy)]
pub struct KalmanFilter {
    pub a: f64,
    pub h: f64,
    pub q: f64,
    pub r: f64,
    state: f64,
    covariance: f64,
}

impl KalmanFilter {
    pub fn new(initial_state: f64, initial_covariance: f64, q: f64, r: f64) -> Self {
        Self {
            a: 1.0,
            h: 1.0,
            q,
            r,
            state: initial_state,
            covariance: initial_covariance,
        }
    }

    pub fn state(&self) -> f64 {
        self.state
    }

    pub fn covariance(&self) -> f64 {
        self.covariance
    }

    pub fn set_measurement_noise(&mut self, value: f64) -> Result<(), &'static str> {
        if value <= 0.0 {
            return Err("measurement noise R must be greater than zero");
        }
        self.r = value;
        Ok(())
    }

    pub fn filter(&mut self, measurement: f64, control_input: f64) -> f64 {
        let x_pred = (self.a * self.state) + control_input;
        let p_pred = (self.a * self.covariance * self.a) + self.q;
        let gain = (p_pred * self.h) / ((self.h * p_pred * self.h) + self.r);

        self.state = x_pred + gain * (measurement - (self.h * x_pred));
        self.covariance = (1.0 - (gain * self.h)) * p_pred;
        self.state
    }

    pub fn reset(&mut self, initial_state: f64, initial_covariance: f64) {
        self.state = initial_state;
        self.covariance = initial_covariance;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(actual: f64, expected: f64) {
        assert!((actual - expected).abs() < 1e-12, "{actual} != {expected}");
    }

    #[test]
    fn filter_matches_predict_update_equations() {
        let mut filter = KalmanFilter::new(0.0, 1.0, 1.0, 100.0);

        close(filter.filter(100.0, 0.0), 1.9607843137254901);
        close(filter.state(), 1.9607843137254901);
        close(filter.covariance(), 1.9607843137254903);
    }

    #[test]
    fn measurement_noise_matches_csharp_validation_boundary() {
        let mut filter = KalmanFilter::new(0.0, 1.0, 1.0, 100.0);
        assert!(filter.set_measurement_noise(0.0).is_err());
        assert!(filter.set_measurement_noise(-1.0).is_err());
        assert!(filter.set_measurement_noise(f64::INFINITY).is_ok());
        assert!(filter.set_measurement_noise(f64::NAN).is_ok());
        assert!(filter.r.is_nan());
        assert!(filter.set_measurement_noise(5.0).is_ok());
        assert_eq!(filter.r, 5.0);
    }

    #[test]
    fn reset_replaces_state_but_not_configuration() {
        let mut filter = KalmanFilter::new(0.0, 1.0, 2.0, 3.0);
        filter.a = 0.5;
        filter.reset(10.0, 4.0);

        assert_eq!(filter.state(), 10.0);
        assert_eq!(filter.covariance(), 4.0);
        assert_eq!(filter.a, 0.5);
        assert_eq!(filter.q, 2.0);
        assert_eq!(filter.r, 3.0);
    }
}
