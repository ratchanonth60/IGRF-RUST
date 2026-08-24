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
