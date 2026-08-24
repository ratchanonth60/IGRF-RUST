use std::io::{self, Read};
use std::net::{Shutdown, TcpStream, ToSocketAddrs};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::thread::{self, JoinHandle};
use std::time::Duration;

pub const MAGSON_FRAME_SIZE: usize = 72;
const MAGSON_DATA_TYPE: i32 = 1;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MagsonSample {
    pub bx: f64,
    pub by: f64,
    pub bz: f64,
}

#[derive(Debug, Default, Clone)]
pub struct MagsonFrameParser {
    buffer: Vec<u8>,
}

impl MagsonFrameParser {
    pub fn feed(&mut self, bytes: &[u8]) -> Vec<MagsonSample> {
        self.buffer.extend_from_slice(bytes);
        let mut samples = Vec::new();
        while self.buffer.len() >= MAGSON_FRAME_SIZE {
            let frame: Vec<u8> = self.buffer.drain(..MAGSON_FRAME_SIZE).collect();
            if let Some(sample) = parse_magson_frame(&frame) {
                samples.push(sample);
            }
        }
        samples
    }

    pub fn buffered_len(&self) -> usize {
        self.buffer.len()
    }

    pub fn reset(&mut self) {
        self.buffer.clear();
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
    reader_thread: Option<JoinHandle<()>>,
}

impl Default for MagsonTcpClient {
    fn default() -> Self {
        Self {
            stream: None,
            stop: Arc::new(AtomicBool::new(false)),
            open: Arc::new(AtomicBool::new(false)),
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
        let stop = Arc::clone(&self.stop);
        let open = Arc::clone(&self.open);

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
