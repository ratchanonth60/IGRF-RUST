/// Tick the gains are defined against, in seconds. `Ki` is "per tick" and `Kd`
/// is "per tick" as well, exactly as the C# build and the existing
/// `SystemConfig.json` tuning assume.
pub const NOMINAL_TICK_SECONDS: f64 = 0.1;

#[derive(Debug, Clone, Copy)]
pub struct PidController {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub max_output: f64,
    pub min_output: f64,
    prev_error: f64,
    integral: f64,
}

impl Default for PidController {
    fn default() -> Self {
        Self {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            max_output: 100.0,
            min_output: -100.0,
            prev_error: 0.0,
            integral: 0.0,
        }
    }
}

impl PidController {
    /// One step against the nominal tick. Equivalent to
    /// `calculate_dt(setpoint, measurement, NOMINAL_TICK_SECONDS)`.
    pub fn calculate(&mut self, setpoint: f64, measurement: f64) -> f64 {
        self.calculate_dt(setpoint, measurement, NOMINAL_TICK_SECONDS)
    }

    /// One step over an actual elapsed `dt` in seconds.
    ///
    /// The integral and derivative terms are scaled by how many nominal ticks
    /// `dt` really was, so a late tick contributes proportionally more integral
    /// and proportionally less derivative. At `dt == NOMINAL_TICK_SECONDS` this
    /// is bit-for-bit the untimed loop, which keeps every gain in
    /// `SystemConfig.json` meaning what it meant when it was tuned.
    ///
    /// Without this, a loop driven off a 60 Hz repaint sees ticks quantised to
    /// 100/117/133 ms and Ki and Kd swing by roughly 17% between samples.
    pub fn calculate_dt(&mut self, setpoint: f64, measurement: f64, dt: f64) -> f64 {
        // A stalled or backwards clock must not divide the derivative by zero
        // or dump an unbounded step into the integral.
        let ticks = (dt / NOMINAL_TICK_SECONDS).clamp(0.01, 10.0);
        let error = setpoint - measurement;
        let p_out = self.kp * error;

        self.integral += error * ticks;
        let mut i_out = self.ki * self.integral;
        if i_out > self.max_output {
            i_out = self.max_output;
            self.integral = self.max_output / if self.ki != 0.0 { self.ki } else { 1.0 };
        } else if i_out < self.min_output {
            i_out = self.min_output;
            self.integral = self.min_output / if self.ki != 0.0 { self.ki } else { 1.0 };
        }

        let d_out = self.kd * (error - self.prev_error) / ticks;
        let mut output = p_out + i_out + d_out;
        if output > self.max_output {
            output = self.max_output;
        } else if output < self.min_output {
            output = self.min_output;
        }
        self.prev_error = error;
        output
    }

    /// Clears the integral and derivative history. Only for a deliberate stop:
    /// see [`Self::hold`] for the watchdog path.
    pub fn reset(&mut self) {
        self.prev_error = 0.0;
        self.integral = 0.0;
    }

    /// Freezes the loop without discarding what it has learned.
    ///
    /// A watchdog pause is not a stop: the coils are holding a large standing
    /// current that lives almost entirely in the integral (at Ki = 0.068 an
    /// output of -7900 is an integral of about -116000). Clearing it drops the
    /// field, the error jumps to the full ambient 40000 nT, and the rebuilt
    /// integral overshoots into the output limit - a full-scale transient
    /// through the drivers on every reconnect. Only `prev_error` is dropped, so
    /// the first tick back does not see a derivative kick across the gap.
    pub fn hold(&mut self) {
        self.prev_error = 0.0;
    }

    /// Current integral term, in error-units x ticks.
    pub fn integral(&self) -> f64 {
        self.integral
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_csharp_controller() {
        let mut pid = PidController::default();
        assert_eq!(pid.kp, 0.0);
        assert_eq!(pid.ki, 0.0);
        assert_eq!(pid.kd, 0.0);
        assert_eq!(pid.max_output, 100.0);
        assert_eq!(pid.min_output, -100.0);
        assert_eq!(pid.calculate(0.0, 0.0), 0.0);
    }

    #[test]
    fn calculate_uses_p_i_and_d_terms() {
        let mut pid = PidController {
            kp: 2.0,
            ki: 0.5,
            kd: 1.0,
            ..Default::default()
        };
        assert_eq!(pid.calculate(4.0, 1.0), 10.5);
        assert_eq!(pid.calculate(4.0, 2.0), 5.5);
    }

    #[test]
    fn integral_is_clamped_before_final_output() {
        let mut pid = PidController {
            ki: 1.0,
            max_output: 10.0,
            min_output: -10.0,
            ..Default::default()
        };
        assert_eq!(pid.calculate(20.0, 0.0), 10.0);
        assert_eq!(pid.calculate(-5.0, 0.0), 5.0);
    }

    #[test]
    fn final_output_is_clamped() {
        let mut pid = PidController {
            kp: 2.0,
            max_output: 10.0,
            min_output: -10.0,
            ..Default::default()
        };
        assert_eq!(pid.calculate(100.0, 0.0), 10.0);
        assert_eq!(pid.calculate(-100.0, 0.0), -10.0);
    }

    /// The whole point of the dt scaling: a tick that arrives late must not
    /// change the effective gains.
    #[test]
    fn a_late_tick_integrates_proportionally_more_than_an_on_time_one() {
        let mut nominal = PidController {
            ki: 1.0,
            ..Default::default()
        };
        let mut late = nominal;

        nominal.calculate_dt(10.0, 0.0, NOMINAL_TICK_SECONDS);
        nominal.calculate_dt(10.0, 0.0, NOMINAL_TICK_SECONDS);
        late.calculate_dt(10.0, 0.0, NOMINAL_TICK_SECONDS * 2.0);

        assert_eq!(nominal.integral(), late.integral());
    }

    #[test]
    fn the_untimed_call_still_matches_the_original_per_tick_loop() {
        let mut untimed = PidController {
            kp: 2.0,
            ki: 0.5,
            kd: 1.0,
            ..Default::default()
        };
        let mut timed = untimed;

        assert_eq!(
            untimed.calculate(4.0, 1.0),
            timed.calculate_dt(4.0, 1.0, NOMINAL_TICK_SECONDS)
        );
        assert_eq!(
            untimed.calculate(4.0, 2.0),
            timed.calculate_dt(4.0, 2.0, NOMINAL_TICK_SECONDS)
        );
    }

    #[test]
    fn a_zero_or_negative_dt_cannot_divide_the_derivative_by_zero() {
        let mut pid = PidController {
            kd: 1.0,
            ki: 1.0,
            ..Default::default()
        };
        assert!(pid.calculate_dt(10.0, 0.0, 0.0).is_finite());
        assert!(pid.calculate_dt(10.0, 0.0, -5.0).is_finite());
    }

    #[test]
    fn hold_keeps_the_standing_output_but_drops_the_derivative_kick() {
        let mut pid = PidController {
            ki: 1.0,
            kd: 1.0,
            ..Default::default()
        };
        pid.calculate(10.0, 0.0);
        let integral = pid.integral();

        pid.hold();

        assert_eq!(
            pid.integral(),
            integral,
            "the standing current must survive"
        );
        // With prev_error cleared the first tick back sees d = kd * error, not
        // kd * (error - a stale error from before the gap.)
        assert_eq!(pid.calculate(0.0, 0.0), integral);
    }

    #[test]
    fn reset_clears_integral_and_previous_error() {
        let mut pid = PidController {
            ki: 1.0,
            kd: 1.0,
            ..Default::default()
        };
        pid.calculate(10.0, 0.0);
        pid.reset();
        assert_eq!(pid.calculate(0.0, 0.0), 0.0);
    }
}
