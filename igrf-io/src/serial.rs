use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::io::{self, Read, Write};
use std::time::Duration;

pub const SENSOR_PACKET_SIZE: usize = 7;
const BUFFER_LIMIT: usize = 1000;
const SERIAL_IO_TIMEOUT: Duration = Duration::from_millis(20);

#[derive(Debug, Default, Clone)]
pub struct SensorFrameParser {
    buffer: Vec<u8>,
    sensor_ready: bool,
}

impl SensorFrameParser {
    pub fn feed(&mut self, bytes: &[u8]) -> Vec<Vec<u8>> {
        self.buffer.extend_from_slice(bytes);

        if self.buffer.len() > BUFFER_LIMIT {
            self.buffer.clear();
        }

        if !self.sensor_ready {
            if let Some(index) = self.buffer.windows(2).position(|window| window == b"OK") {
                self.sensor_ready = true;
                self.buffer.drain(..index + 2);
            }
            // Preserve the C# handler's one-pass behavior: data following the
            // handshake is parsed on the next read callback.
            return Vec::new();
        }

        let mut packets = Vec::new();
        while self.buffer.len() >= SENSOR_PACKET_SIZE {
            let next_boundary_is_valid = self.buffer.len() < SENSOR_PACKET_SIZE * 2
                || self.buffer[SENSOR_PACKET_SIZE * 2 - 1] == 0x0D;
            if self.buffer[SENSOR_PACKET_SIZE - 1] == 0x0D && next_boundary_is_valid {
                packets.push(self.buffer[..SENSOR_PACKET_SIZE].to_vec());
                self.buffer.drain(..SENSOR_PACKET_SIZE);
            } else {
                self.buffer.remove(0);
            }
        }
        packets
    }

    pub fn is_sensor_ready(&self) -> bool {
        self.sensor_ready
    }

    pub fn buffered_len(&self) -> usize {
        self.buffer.len()
    }

    pub fn reset(&mut self) {
        self.buffer.clear();
        self.sensor_ready = false;
    }
}

#[derive(Default)]
pub struct SerialPortManager {
    port: Option<Box<dyn SerialPort>>,
    parser: SensorFrameParser,
}

impl SerialPortManager {
    pub fn connect(&mut self, port_name: &str, baud_rate: u32) -> serialport::Result<()> {
        self.disconnect();
        let mut port = serialport::new(port_name, baud_rate)
            .data_bits(DataBits::Eight)
            .flow_control(FlowControl::None)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .timeout(SERIAL_IO_TIMEOUT)
            .dtr_on_open(true)
            .open()?;
        port.write_data_terminal_ready(true)?;
        self.port = Some(port);
        Ok(())
    }

    pub fn is_open(&self) -> bool {
        self.port.is_some()
    }

    pub fn write(&mut self, data: &[u8]) -> serialport::Result<()> {
        let Some(port) = self.port.as_mut() else {
            return Err(serialport::Error::new(
                serialport::ErrorKind::NoDevice,
                "serial port is not open",
            ));
        };
        port.write_all(data).map_err(serialport::Error::from)
    }

    pub fn read_available(&mut self) -> serialport::Result<Vec<Vec<u8>>> {
        let Some(port) = self.port.as_mut() else {
            return Err(serialport::Error::new(
                serialport::ErrorKind::NoDevice,
                "serial port is not open",
            ));
        };
        let available = port.bytes_to_read()? as usize;
        if available == 0 {
            return Ok(self.parser.feed(&[]));
        }
        let mut bytes = [0_u8; 4096];
        let read_len = available.min(bytes.len());
        let count = match port.read(&mut bytes[..read_len]) {
            Ok(count) => count,
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
                ) =>
            {
                0
            }
            Err(error) => return Err(serialport::Error::from(error)),
        };
        Ok(self.parser.feed(&bytes[..count]))
    }

    pub fn parser(&self) -> &SensorFrameParser {
        &self.parser
    }

    pub fn disconnect(&mut self) {
        self.port = None;
        self.parser.reset();
    }
}

impl Write for SerialPortManager {
    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        SerialPortManager::write(self, data)
            .map(|_| data.len())
            .map_err(|error| io::Error::other(error.to_string()))
    }

    fn flush(&mut self) -> io::Result<()> {
        let Some(port) = self.port.as_mut() else {
            return Err(io::Error::new(
                io::ErrorKind::NotConnected,
                "serial port is not open",
            ));
        };
        port.flush()
    }
}

impl Drop for SerialPortManager {
    fn drop(&mut self) {
        self.disconnect();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PACKET: [u8; SENSOR_PACKET_SIZE] = [1, 2, 3, 4, 5, 6, 0x0D];

    #[test]
    fn handshake_can_be_split_and_packet_requires_cr_framing() {
        let mut parser = SensorFrameParser::default();
        assert!(parser.feed(b"noiseO").is_empty());
        assert!(parser.feed(b"K").is_empty());
        assert!(parser.is_sensor_ready());
        assert!(parser.feed(&PACKET[..3]).is_empty());
        assert_eq!(parser.feed(&PACKET[3..]), vec![PACKET.to_vec()]);
    }

    #[test]
    fn handshake_and_data_in_one_read_are_deferred_one_pass() {
        let mut parser = SensorFrameParser::default();
        let mut input = b"OK".to_vec();
        input.extend_from_slice(&PACKET);

        assert!(parser.feed(&input).is_empty());
        assert_eq!(parser.feed(&[]), vec![PACKET.to_vec()]);
    }

    #[test]
    fn invalid_prefix_resynchronizes_one_byte_at_a_time() {
        let mut parser = SensorFrameParser::default();
        parser.feed(b"OK");
        let mut input = vec![0x99];
        input.extend_from_slice(&PACKET);

        assert_eq!(parser.feed(&input), vec![PACKET.to_vec()]);
    }

    #[test]
    fn overflow_clears_buffer_and_disconnect_resets_handshake() {
        let mut parser = SensorFrameParser::default();
        assert!(parser.feed(&vec![0_u8; BUFFER_LIMIT + 1]).is_empty());
        assert_eq!(parser.buffered_len(), 0);
        parser.feed(b"OK");
        assert!(parser.is_sensor_ready());
        parser.reset();
        assert!(!parser.is_sensor_ready());
        assert_eq!(parser.buffered_len(), 0);
    }
}
