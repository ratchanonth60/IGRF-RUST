pub const CONTROLLER_HEADER: u8 = 0xA0;
pub const CONTROLLER_PACKET_LEN: usize = 15;

/// Largest magnitude the controller firmware handles correctly, per axis
/// (X, Y, Z), read out of the STM32 flash dump.
///
/// These are not soft preferences. The firmware's range check has three arms:
///
/// ```text
/// if (v > 0 && v <=  LIMIT) { CCR = (u32)v;   DIR = 1; }
/// if (v < 0 && v >= -LIMIT) { CCR = (u32)-v;  DIR = 0; }
/// else                      { CCR = (u32)v;            }   // no abs, no clamp
/// ```
///
/// The `else` arm is what an out-of-range value hits, and it writes the raw
/// value into the capture/compare register:
///
/// - X and Y are 16-bit timers (TIM1 ARR 55960, TIM3 ARR 58360), so the store
///   truncates. Commanding 83940 on X lands at CCR 18404, a third of the drive
///   that was asked for rather than more.
/// - `vcvt.u32.f32` of a negative float saturates to 0 on ARM, so a
///   past-the-limit negative command drops the coil to zero output while the
///   direction pin keeps its previous state.
/// - Z is a 32-bit timer (TIM2 ARR 97270) so it does not wrap, but anything
///   above the ARR is simply 100% duty.
///
/// Nothing reports any of this back, so the only defence is never to send a
/// value the firmware mishandles. [`build_controller_packet`] enforces it.
pub const FIRMWARE_MAX_OUTPUT: [f64; 3] = [42000.0, 17700.0, 69000.0];

pub fn calculate_mod_rtu_crc(data: &[u8]) -> [u8; 2] {
    let mut crc = 0xFFFF_u16;
    for &byte in data {
        crc ^= byte as u16;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    crc.to_be_bytes()
}

/// Clamps one axis to what the firmware can act on. See
/// [`FIRMWARE_MAX_OUTPUT`] for why exceeding it is worse than saturating.
pub fn clamp_to_firmware(axis: usize, output: f64) -> f64 {
    let limit = FIRMWARE_MAX_OUTPUT[axis.min(2)];
    output.clamp(-limit, limit)
}

pub fn build_controller_packet(
    output_x: f64,
    output_y: f64,
    output_z: f64,
) -> [u8; CONTROLLER_PACKET_LEN] {
    let mut packet = [0_u8; CONTROLLER_PACKET_LEN];
    packet[0] = CONTROLLER_HEADER;
    // Clamped here rather than at the call sites: this is the one function
    // every path to the coils goes through, and a value the firmware
    // mishandles must not be constructible.
    packet[1..5].copy_from_slice(&(clamp_to_firmware(0, output_x) as f32).to_le_bytes());
    packet[5..9].copy_from_slice(&(clamp_to_firmware(1, output_y) as f32).to_le_bytes());
    packet[9..13].copy_from_slice(&(clamp_to_firmware(2, output_z) as f32).to_le_bytes());
    let crc = calculate_mod_rtu_crc(&packet[..13]);
    packet[13..].copy_from_slice(&crc);
    packet
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_is_high_byte_first() {
        assert_eq!(
            calculate_mod_rtu_crc(&[0x01, 0x03, 0x00, 0x00, 0x00, 0x0A]),
            [0xCD, 0xC5]
        );
    }

    /// The firmware writes an out-of-range value straight into a 16-bit CCR,
    /// so 83940 on X arrives as 18404 - a third of the requested drive. The
    /// packet builder has to be the thing that stops it.
    #[test]
    fn outputs_beyond_what_the_firmware_handles_are_clamped_not_wrapped() {
        let packet = build_controller_packet(100_000.0, -100_000.0, 100_000.0);

        assert_eq!(&packet[1..5], &42_000.0_f32.to_le_bytes());
        assert_eq!(&packet[5..9], &(-17_700.0_f32).to_le_bytes());
        assert_eq!(&packet[9..13], &69_000.0_f32.to_le_bytes());
    }

    #[test]
    fn values_inside_the_firmware_range_pass_through_untouched() {
        let packet = build_controller_packet(-7900.0, 12_000.0, 21_500.0);

        assert_eq!(&packet[1..5], &(-7900.0_f32).to_le_bytes());
        assert_eq!(&packet[5..9], &12_000.0_f32.to_le_bytes());
        assert_eq!(&packet[9..13], &21_500.0_f32.to_le_bytes());
    }

    #[test]
    fn controller_packet_is_15_bytes_with_little_endian_f32_payload() {
        let packet = build_controller_packet(1.0, -2.5, 3.25);
        assert_eq!(packet[0], 0xA0);
        assert_eq!(&packet[1..5], &1.0_f32.to_le_bytes());
        assert_eq!(&packet[5..9], &(-2.5_f32).to_le_bytes());
        assert_eq!(&packet[9..13], &3.25_f32.to_le_bytes());
        assert_eq!(&packet[13..], &calculate_mod_rtu_crc(&packet[..13]));
    }
}
