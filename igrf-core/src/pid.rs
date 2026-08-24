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
    pub fn calculate(&mut self, setpoint: f64, measurement: f64) -> f64 {
        let error = setpoint - measurement;
        let p_out = self.kp * error;

        self.integral += error;
        let mut i_out = self.ki * self.integral;
        if i_out > self.max_output {
            i_out = self.max_output;
            self.integral = self.max_output / if self.ki != 0.0 { self.ki } else { 1.0 };
        } else if i_out < self.min_output {
            i_out = self.min_output;
            self.integral = self.min_output / if self.ki != 0.0 { self.ki } else { 1.0 };
        }

        let d_out = self.kd * (error - self.prev_error);
        let mut output = p_out + i_out + d_out;
        if output > self.max_output {
            output = self.max_output;
        } else if output < self.min_output {
            output = self.min_output;
        }
        self.prev_error = error;
        output
    }

    pub fn reset(&mut self) {
        self.prev_error = 0.0;
        self.integral = 0.0;
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
