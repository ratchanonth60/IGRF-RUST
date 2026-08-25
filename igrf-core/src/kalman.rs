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

    /// One update over a nominal tick. Equivalent to
    /// `filter_ticks(measurement, control_input, 1.0)`.
    pub fn filter(&mut self, measurement: f64, control_input: f64) -> f64 {
        self.filter_ticks(measurement, control_input, 1.0)
    }

    /// One update over `ticks` nominal intervals.
    ///
    /// `control_input` is how far the true quantity is known to have moved
    /// since the last update, in the same units as the measurement. Leaving it
    /// at zero states that the field only random-walks, which is what `q`
    /// describes - and a commanded ramp is not a random walk. At q = 1, r = 500
    /// the steady-state gain is 0.0437, so a 5000 nT/s ramp settles 10933 nT
    /// behind the truth: the loop sees an error that is not there and the CSV
    /// records a field the cage never held. Feeding the commanded step in here
    /// removes that lag outright.
    ///
    /// `q` is scaled by `ticks` because it is a variance per interval. Without
    /// it a loop driven off a 60 Hz repaint sees ticks quantised to
    /// 100/117/133 ms and the gain moves with the display, not the physics.
    pub fn filter_ticks(&mut self, measurement: f64, control_input: f64, ticks: f64) -> f64 {
        // A stalled or backwards clock must not drive the process noise to zero
        // (gain 0, filter deaf) or to infinity (gain 1, filter off).
        let ticks = ticks.clamp(0.01, 10.0);
        let x_pred = (self.a * self.state) + control_input;
        let p_pred = (self.a * self.covariance * self.a) + self.q * ticks;
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

    /// The whole point of the control input: a known move must not be
    /// re-derived from the measurement, because that is what costs the lag.
    #[test]
    fn a_known_step_is_tracked_without_the_gain_having_to_find_it() {
        let mut blind = KalmanFilter::new(0.0, 1.0, 1.0, 500.0);
        let mut informed = blind;

        // The truth steps 500 nT and the measurement lands on it exactly.
        blind.filter(500.0, 0.0);
        informed.filter(500.0, 500.0);

        assert!(blind.state() < 100.0, "the blind filter has to crawl there");
        close(informed.state(), 500.0);
    }

    #[test]
    fn process_noise_scales_with_the_interval() {
        let mut once = KalmanFilter::new(0.0, 1.0, 1.0, 100.0);
        let mut twice = once;

        once.filter_ticks(0.0, 0.0, 2.0);
        twice.filter(0.0, 0.0);
        twice.filter(0.0, 0.0);

        // Not equal - two updates also apply the gain twice - but a double-length
        // tick must admit more process noise than a single one, not the same.
        assert!(once.covariance() > twice.covariance());
    }

    #[test]
    fn a_zero_or_negative_tick_leaves_the_gain_usable() {
        let mut filter = KalmanFilter::new(0.0, 1.0, 1.0, 100.0);
        for ticks in [0.0, -5.0, f64::INFINITY] {
            let value = filter.filter_ticks(10.0, 0.0, ticks);
            assert!(value.is_finite(), "ticks {ticks} produced {value}");
        }
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
