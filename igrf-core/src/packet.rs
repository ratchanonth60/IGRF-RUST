pub const CONTROLLER_HEADER: u8 = 0xA0;
pub const CONTROLLER_PACKET_LEN: usize = 15;

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

pub fn build_controller_packet(
    output_x: f64,
    output_y: f64,
    output_z: f64,
) -> [u8; CONTROLLER_PACKET_LEN] {
    let mut packet = [0_u8; CONTROLLER_PACKET_LEN];
    packet[0] = CONTROLLER_HEADER;
    packet[1..5].copy_from_slice(&(output_x as f32).to_le_bytes());
    packet[5..9].copy_from_slice(&(output_y as f32).to_le_bytes());
    packet[9..13].copy_from_slice(&(output_z as f32).to_le_bytes());
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
