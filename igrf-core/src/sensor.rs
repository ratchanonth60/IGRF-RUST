const SCALE: f64 = 20.0 / 3.0;
const HARD_IRON: [f64; 3] = [1349.5, 4110.95, -1343.37];
const SOFT_IRON: [[f64; 3]; 3] = [
    [0.9958, -0.0050, 0.0064],
    [-0.050, 1.0042, -0.0087],
    [0.0064, -0.0087, 1.0003],
];

#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct RawSensorData {
    pub mag_x: f64,
    pub mag_y: f64,
    pub mag_z: f64,
}

pub struct SensorService {
    pub reference_x: f64,
    pub reference_y: f64,
    pub reference_z: f64,
    last_raw_x: f64,
    last_raw_y: f64,
    last_raw_z: f64,
}

impl Default for SensorService {
    fn default() -> Self {
        Self {
            reference_x: 0.0,
            reference_y: 0.0,
            reference_z: 0.0,
            last_raw_x: 0.0,
            last_raw_y: 0.0,
            last_raw_z: 0.0,
        }
    }
}

impl SensorService {
    pub fn process_data(&mut self, packet: &[u8]) -> RawSensorData {
        if packet.len() < 7 {
            return RawSensorData::default();
        }

        let raw_x = i16::from_be_bytes([packet[0], packet[1]]);
        let raw_y = i16::from_be_bytes([packet[2], packet[3]]);
        let raw_z = i16::from_be_bytes([packet[4], packet[5]]);
        let mag = [
            raw_x as f64 * SCALE,
            raw_y as f64 * SCALE,
            raw_z as f64 * SCALE,
        ];

        self.last_raw_x = mag[0];
        self.last_raw_y = mag[1];
        self.last_raw_z = mag[2];

        let mag_hi = [
            mag[0] - HARD_IRON[0] - self.reference_x,
            mag[1] - HARD_IRON[1] - self.reference_y,
            mag[2] - HARD_IRON[2] - self.reference_z,
        ];
        let mut mag_cal = [0.0; 3];
        for (i, row) in SOFT_IRON.iter().enumerate() {
            mag_cal[i] = row[0] * mag_hi[0] + row[1] * mag_hi[1] + row[2] * mag_hi[2];
        }

        RawSensorData {
            mag_x: mag_cal[0],
            mag_y: mag_cal[1],
            mag_z: mag_cal[2],
        }
    }

    pub fn set_zero(&mut self, current_x: f64, current_y: f64, current_z: f64) {
        self.reference_x += current_x;
        self.reference_y += current_y;
        self.reference_z += current_z;
    }

    pub fn last_raw_x(&self) -> f64 {
        self.last_raw_x
    }

    pub fn last_raw_y(&self) -> f64 {
        self.last_raw_y
    }

    pub fn last_raw_z(&self) -> f64 {
        self.last_raw_z
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(actual: f64, expected: f64) {
        assert!((actual - expected).abs() < 1e-9, "{actual} != {expected}");
    }

    #[test]
    fn processes_signed_big_endian_axes_and_calibration() {
        let mut sensor = SensorService::default();
        sensor.reference_x = 1.0;
        sensor.reference_y = -2.0;
        sensor.reference_z = 3.0;
        let result = sensor.process_data(&[0x12, 0x34, 0xFF, 0xFE, 0x80, 0x00, 0x0D]);

        close(sensor.last_raw_x(), 31066.666666666668);
        close(sensor.last_raw_y(), -13.333333333333334);
        close(sensor.last_raw_z(), -218453.33333333334);
        close(result.mag_x, 28222.447218);
        close(result.mag_y, -3736.522475666666);
        close(result.mag_z, -216952.04989066668);
    }

    #[test]
    fn short_packet_returns_zero_without_changing_last_raw_values() {
        let mut sensor = SensorService::default();
        sensor.process_data(&[0x12, 0x34, 0x56, 0x78, 0x00, 0x01, 0x0D]);
        assert_eq!(sensor.process_data(&[0; 6]), RawSensorData::default());
        assert_eq!(sensor.last_raw_x(), 31066.666666666668);
    }

    #[test]
    fn set_zero_accumulates_references() {
        let mut sensor = SensorService::default();
        sensor.set_zero(1.5, -2.0, 3.0);
        sensor.set_zero(0.5, 1.0, -1.0);

        assert_eq!(sensor.reference_x, 2.0);
        assert_eq!(sensor.reference_y, -1.0);
        assert_eq!(sensor.reference_z, 2.0);
    }
}
