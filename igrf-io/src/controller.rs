use igrf_core::build_controller_packet;
use std::io::{self, Write};

pub fn write_controller_packet<W: Write>(
    writer: &mut W,
    output_x: f64,
    output_y: f64,
    output_z: f64,
) -> io::Result<()> {
    if [output_x, output_y, output_z]
        .into_iter()
        .any(|output| !output.is_finite())
    {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "controller outputs must be finite",
        ));
    }
    writer.write_all(&build_controller_packet(output_x, output_y, output_z))
}

/// The controller's only reply, sent when it rejects a packet.
///
/// From the flash dump (`docs/controller-protocol.md`): a CRC mismatch at
/// `0x0800197c` writes the six bytes at `0x20000000` back to the host and
/// restarts the receive loop. An accepted packet produces no reply at all.
pub const CONTROLLER_ERROR_REPLY: &[u8] = b"Error\r";

/// Counts rejected packets from the controller's return path.
///
/// Nothing else comes back over that link, so the count is the number of
/// commands the coils never acted on. Without it a noisy USB line silently
/// drops setpoints at 10 Hz and the app keeps reporting success, because a
/// write that reached the driver looks identical to one the firmware kept.
#[derive(Debug, Default, Clone)]
pub struct ControllerReplyCounter {
    tail: Vec<u8>,
}

impl ControllerReplyCounter {
    /// Rejections completed by `bytes`, including any that started in an
    /// earlier call - a six-byte reply splits across reads at 10 Hz.
    pub fn feed(&mut self, bytes: &[u8]) -> usize {
        self.tail.extend_from_slice(bytes);
        let mut count = 0;
        while let Some(index) = self
            .tail
            .windows(CONTROLLER_ERROR_REPLY.len())
            .position(|window| window == CONTROLLER_ERROR_REPLY)
        {
            count += 1;
            self.tail.drain(..index + CONTROLLER_ERROR_REPLY.len());
        }
        // Only a partial reply can still grow into a match, so the buffer never
        // needs to outlive one: anything older is line noise.
        let keep = CONTROLLER_ERROR_REPLY.len() - 1;
        if self.tail.len() > keep {
            self.tail.drain(..self.tail.len() - keep);
        }
        count
    }

    pub fn reset(&mut self) {
        self.tail.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct FailingWriter;

    impl Write for FailingWriter {
        fn write(&mut self, _: &[u8]) -> io::Result<usize> {
            Err(io::Error::new(io::ErrorKind::BrokenPipe, "test failure"))
        }

        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn transport_writes_the_same_packet_without_owning_a_timer() {
        let mut bytes = Vec::new();
        write_controller_packet(&mut bytes, 1.0, 2.0, 3.0).unwrap();
        assert_eq!(bytes.len(), 15);
        assert_eq!(bytes[0], 0xA0);
    }

    #[test]
    fn transport_propagates_write_failures() {
        assert_eq!(
            write_controller_packet(&mut FailingWriter, 1.0, 2.0, 3.0)
                .unwrap_err()
                .kind(),
            io::ErrorKind::BrokenPipe
        );
    }

    #[test]
    fn a_reply_split_across_reads_is_still_counted_once() {
        let mut counter = ControllerReplyCounter::default();
        assert_eq!(counter.feed(b"Err"), 0);
        assert_eq!(counter.feed(b"or\r"), 1);
        assert_eq!(counter.feed(b"Error\rError\r"), 2);
    }

    #[test]
    fn noise_between_replies_neither_counts_nor_accumulates() {
        let mut counter = ControllerReplyCounter::default();
        assert_eq!(counter.feed(&[0xFF; 4096]), 0);
        assert_eq!(counter.feed(b"junkError\rjunk"), 1);
        // An incomplete reply stays armed but must not count on its own.
        assert_eq!(counter.feed(b"Error"), 0);
        assert_eq!(counter.feed(b"xError\r"), 1);
    }

    #[test]
    fn transport_rejects_non_finite_outputs_before_writing() {
        let mut bytes = Vec::new();
        assert_eq!(
            write_controller_packet(&mut bytes, f64::NAN, 2.0, 3.0)
                .unwrap_err()
                .kind(),
            io::ErrorKind::InvalidInput
        );
        assert!(bytes.is_empty());
    }
}
