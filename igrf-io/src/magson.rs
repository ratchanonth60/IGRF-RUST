use std::io::{self, Read};
use std::net::{Shutdown, TcpStream, ToSocketAddrs};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{mpsc, Arc};
use std::thread::{self, JoinHandle};
use std::time::Duration;

pub const MAGSON_FRAME_SIZE: usize = 72;
const MAGSON_DATA_TYPE: i32 = 1;

/// Frames that fail to parse in a row before the stream is treated as
/// misaligned rather than as carrying frame types this build ignores.
///
/// The link is a fixed-size stream with no delimiter, so one lost byte shifts
/// every boundary after it and nothing recovers on its own. Draining a whole
/// frame per failure - what this did before - keeps that shift forever: every
/// later window reads a data type out of the middle of a payload, fails, and
/// is discarded in silence for the rest of the run.
///
/// Four is a compromise. Frames of other types are legitimate and arrive
/// aligned, so a single failure is not evidence of anything; four consecutive
/// ones, at 288 bytes, is. Hunting one byte at a time can lock onto a
/// `01 00 00 00` inside a payload, but that alignment then fails four more
/// times and the hunt resumes, so a wrong guess costs frames rather than the
/// stream.
const RESYNC_AFTER_FAILURES: u32 = 4;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MagsonSample {
    pub bx: f64,
    pub by: f64,
    pub bz: f64,
}

#[derive(Debug, Default, Clone)]
pub struct MagsonFrameParser {
    buffer: Vec<u8>,
    consecutive_failures: u32,
    dropped: u64,
}

impl MagsonFrameParser {
    pub fn feed(&mut self, bytes: &[u8]) -> Vec<MagsonSample> {
        self.buffer.extend_from_slice(bytes);
        let mut samples = Vec::new();
        while self.buffer.len() >= MAGSON_FRAME_SIZE {
            if let Some(sample) = parse_magson_frame(&self.buffer[..MAGSON_FRAME_SIZE]) {
                samples.push(sample);
                self.buffer.drain(..MAGSON_FRAME_SIZE);
                self.consecutive_failures = 0;
                continue;
            }
            self.dropped = self.dropped.saturating_add(1);
            self.consecutive_failures = self.consecutive_failures.saturating_add(1);
            // Below the threshold the alignment is still assumed good and this
            // is a frame type this build does not read. Above it, the boundary
            // itself is suspect, so step one byte and look again.
            let step = if self.consecutive_failures < RESYNC_AFTER_FAILURES {
                MAGSON_FRAME_SIZE
            } else {
                1
            };
            self.buffer.drain(..step);
        }
        samples
    }

    /// Frames read but not decoded, since the last [`Self::reset`]. Covers both
    /// types this build ignores and every window discarded while resynchronising,
    /// which is deliberate: the two are not distinguishable without the frame
    /// specification, and a count that climbs steadily means neither is benign.
    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    pub fn buffered_len(&self) -> usize {
        self.buffer.len()
    }

    pub fn reset(&mut self) {
        self.buffer.clear();
        self.consecutive_failures = 0;
        self.dropped = 0;
    }
}

pub fn parse_magson_frame(frame: &[u8]) -> Option<MagsonSample> {
    if frame.len() != MAGSON_FRAME_SIZE
        || i32::from_le_bytes(frame[0..4].try_into().ok()?) != MAGSON_DATA_TYPE
    {
        return None;
    }
    Some(MagsonSample {
        bx: f32::from_le_bytes(frame[48..52].try_into().ok()?) as f64,
        by: f32::from_le_bytes(frame[52..56].try_into().ok()?) as f64,
        bz: f32::from_le_bytes(frame[56..60].try_into().ok()?) as f64,
    })
}

pub struct MagsonTcpClient {
    stream: Option<TcpStream>,
    stop: Arc<AtomicBool>,
    open: Arc<AtomicBool>,
    dropped: Arc<AtomicU64>,
    reader_thread: Option<JoinHandle<()>>,
}

impl Default for MagsonTcpClient {
    fn default() -> Self {
        Self {
            stream: None,
            stop: Arc::new(AtomicBool::new(false)),
            open: Arc::new(AtomicBool::new(false)),
            dropped: Arc::new(AtomicU64::new(0)),
            reader_thread: None,
        }
    }
}

impl MagsonTcpClient {
    pub fn connect(&mut self, ip: &str, port: u16) -> io::Result<mpsc::Receiver<MagsonSample>> {
        self.disconnect();
        let address = (ip, port)
            .to_socket_addrs()?
            .next()
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "no TCP address"))?;
        let stream = TcpStream::connect_timeout(&address, Duration::from_secs(5))?;
        stream.set_read_timeout(Some(Duration::from_millis(250)))?;
        let reader = stream.try_clone()?;
        let (sender, receiver) = mpsc::channel();
        self.stop = Arc::new(AtomicBool::new(false));
        self.open = Arc::new(AtomicBool::new(true));
        self.dropped = Arc::new(AtomicU64::new(0));
        let stop = Arc::clone(&self.stop);
        let open = Arc::clone(&self.open);
        let dropped = Arc::clone(&self.dropped);

        self.reader_thread = Some(thread::spawn(move || {
            let mut reader = reader;
            let mut parser = MagsonFrameParser::default();
            let mut bytes = [0_u8; 4096];
            'read_loop: while !stop.load(Ordering::Relaxed) {
                match reader.read(&mut bytes) {
                    Ok(0) => break,
                    Ok(count) => {
                        for sample in parser.feed(&bytes[..count]) {
                            if sender.send(sample).is_err() {
                                break 'read_loop;
                            }
                        }
                        dropped.store(parser.dropped(), Ordering::Relaxed);
                    }
                    Err(error)
                        if matches!(
                            error.kind(),
                            io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
                        ) => {}
                    Err(_) => break,
                }
            }
            open.store(false, Ordering::Relaxed);
        }));
        self.stream = Some(stream);
        Ok(receiver)
    }

    pub fn is_open(&self) -> bool {
        self.open.load(Ordering::Relaxed)
    }

    /// Frames read but not decoded on this connection. See
    /// [`MagsonFrameParser::dropped`]; a count that keeps climbing is the only
    /// sign the app gets that the stream is not being understood.
    pub fn dropped_frames(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    pub fn disconnect(&mut self) {
        self.open.store(false, Ordering::Relaxed);
        self.stop.store(true, Ordering::Relaxed);
        if let Some(stream) = self.stream.take() {
            let _ = stream.shutdown(Shutdown::Both);
        }
        if let Some(thread) = self.reader_thread.take() {
            let _ = thread.join();
        }
    }
}

impl Drop for MagsonTcpClient {
    fn drop(&mut self) {
        self.disconnect();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(data_type: i32, bx: f32, by: f32, bz: f32) -> [u8; MAGSON_FRAME_SIZE] {
        let mut frame = [0_u8; MAGSON_FRAME_SIZE];
        frame[0..4].copy_from_slice(&data_type.to_le_bytes());
        frame[48..52].copy_from_slice(&bx.to_le_bytes());
        frame[52..56].copy_from_slice(&by.to_le_bytes());
        frame[56..60].copy_from_slice(&bz.to_le_bytes());
        frame
    }

    #[test]
    fn parser_handles_fragmented_fixed_frames_and_ignores_other_types() {
        let first = frame(1, 1.25, -2.5, 3.75);
        let second = frame(2, 9.0, 9.0, 9.0);
        let mut parser = MagsonFrameParser::default();
        assert!(parser.feed(&first[..20]).is_empty());
        let mut rest = first[20..].to_vec();
        rest.extend_from_slice(&second);
        assert_eq!(
            parser.feed(&rest),
            vec![MagsonSample {
                bx: 1.25,
                by: -2.5,
                bz: 3.75
            }]
        );
        assert_eq!(parser.buffered_len(), 0);
    }

    /// The failure this exists for: one byte lost shifts every boundary after
    /// it, and draining a whole frame per failure preserves the shift forever.
    #[test]
    fn a_stream_that_loses_one_byte_recovers_instead_of_ending() {
        let mut parser = MagsonFrameParser::default();
        let good = frame(1, 1.0, 2.0, 3.0);
        assert_eq!(parser.feed(&good).len(), 1);

        // Drop the first byte of the next frame, then keep streaming.
        let mut stream = good[1..].to_vec();
        for _ in 0..8 {
            stream.extend_from_slice(&good);
        }
        let samples = parser.feed(&stream);

        assert!(
            samples.len() >= 4,
            "expected the stream to realign, got {} samples",
            samples.len()
        );
        assert!(
            parser.dropped() > 0,
            "the discarded windows must be counted"
        );
        // Realigned means the values are the frame's, not a window of payload.
        assert!(samples.iter().all(|sample| *sample
            == MagsonSample {
                bx: 1.0,
                by: 2.0,
                bz: 3.0
            }));
    }

    /// An aligned stream carrying types this build ignores must not be treated
    /// as misaligned: hunting a byte at a time would find false boundaries in
    /// payloads that are perfectly well framed.
    #[test]
    fn a_few_foreign_frames_do_not_trigger_a_resync() {
        let mut parser = MagsonFrameParser::default();
        let mut stream = Vec::new();
        for _ in 0..3 {
            stream.extend_from_slice(&frame(2, 9.0, 9.0, 9.0));
        }
        stream.extend_from_slice(&frame(1, 4.0, 5.0, 6.0));

        assert_eq!(
            parser.feed(&stream),
            vec![MagsonSample {
                bx: 4.0,
                by: 5.0,
                bz: 6.0
            }]
        );
        assert_eq!(parser.dropped(), 3);
        assert_eq!(parser.buffered_len(), 0, "alignment must have been kept");
    }

    #[test]
    fn standalone_parser_requires_exactly_72_bytes_and_uses_little_endian_offsets() {
        let frame = frame(1, 10.0, 20.0, 30.0);
        let sample = parse_magson_frame(&frame).unwrap();
        let _: f64 = sample.bx;
        assert_eq!(
            sample,
            MagsonSample {
                bx: 10.0,
                by: 20.0,
                bz: 30.0
            }
        );
        assert_eq!(parse_magson_frame(&frame[..71]), None);
        assert_eq!(parse_magson_frame(&[0; 72]), None);
    }
}
