//! UDP listener that lets an external orbit propagator drive the setpoint.
//!
//! One datagram is one command: `bx,by,bz` in nanotesla, ASCII, newline
//! optional. UDP rather than TCP because a setpoint is state, not a stream: if
//! a packet is lost the next one supersedes it, and the propagator never blocks
//! on the cage keeping up. Datagram boundaries also mean no framing to get
//! wrong.
//!
//! ```text
//! echo "39858,-619,20583" | nc -u -w0 127.0.0.1 5005
//! ```
//!
//! The listener binds an address the caller chooses, not `0.0.0.0`. A datagram
//! on this port drives six 48 V coil drivers and carries no authentication of
//! any kind, so reaching past the local machine has to be a decision someone
//! made on purpose - see [`SetpointServer::listen`].

use std::io;
use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::thread::{self, JoinHandle};
use std::time::Duration;

/// Loopback only. An external propagator needs an explicit interface in
/// `SystemConfig.json`, because widening this is a safety decision.
pub const DEFAULT_BIND_ADDRESS: &str = "127.0.0.1";

/// Read timeout, so the thread notices `stop` between datagrams.
const POLL_TIMEOUT: Duration = Duration::from_millis(250);

/// Parses `bx,by,bz` in nT. Extra fields are ignored so a propagator can append
/// its own columns; any non-finite or missing component rejects the datagram.
pub fn parse_setpoint_datagram(bytes: &[u8]) -> Option<[f64; 3]> {
    let text = std::str::from_utf8(bytes).ok()?;
    let mut fields = text.trim().split(',').map(str::trim);
    let mut field = [0.0_f64; 3];
    for value in &mut field {
        *value = fields
            .next()?
            .parse::<f64>()
            .ok()
            .filter(|v| v.is_finite())?;
    }
    Some(field)
}

/// Owns the listener thread. Dropping or disconnecting stops it.
#[derive(Default)]
pub struct SetpointServer {
    stop: Option<Arc<AtomicBool>>,
    thread: Option<JoinHandle<()>>,
    port: Option<u16>,
}

impl SetpointServer {
    /// Binds `address:port` and streams commands down the returned channel.
    /// `port` 0 asks the OS for a free one; read it back with [`Self::port`].
    ///
    /// `address` is the interface to accept commands on. Anything other than a
    /// loopback address exposes the coils to every host that can route to this
    /// machine, with no authentication and no source check: one `nc -u` from
    /// anywhere on the lab network is a valid command. Callers pass
    /// [`DEFAULT_BIND_ADDRESS`] unless someone has decided otherwise.
    pub fn listen(&mut self, address: &str, port: u16) -> io::Result<mpsc::Receiver<[f64; 3]>> {
        self.disconnect();
        let socket = UdpSocket::bind((address, port))?;
        socket.set_read_timeout(Some(POLL_TIMEOUT))?;
        self.port = Some(socket.local_addr()?.port());

        let (sender, receiver) = mpsc::channel();
        let stop = Arc::new(AtomicBool::new(false));
        self.stop = Some(Arc::clone(&stop));
        self.thread = Some(thread::spawn(move || {
            let mut bytes = [0_u8; 512];
            while !stop.load(Ordering::Relaxed) {
                match socket.recv(&mut bytes) {
                    Ok(count) => {
                        // A malformed datagram is dropped rather than fatal: a
                        // stray probe on the port must not stop the run.
                        if let Some(field) = parse_setpoint_datagram(&bytes[..count]) {
                            if sender.send(field).is_err() {
                                break;
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
        }));
        Ok(receiver)
    }

    pub fn is_listening(&self) -> bool {
        self.thread.is_some()
    }

    /// Port actually bound, which differs from the requested one when 0 was
    /// asked for.
    pub fn port(&self) -> Option<u16> {
        self.port
    }

    pub fn disconnect(&mut self) {
        if let Some(stop) = self.stop.take() {
            stop.store(true, Ordering::Relaxed);
        }
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
        self.port = None;
    }
}

impl Drop for SetpointServer {
    fn drop(&mut self) {
        self.disconnect();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_datagram_is_three_finite_nanotesla_components() {
        assert_eq!(
            parse_setpoint_datagram(b"39858,-619,20583\n"),
            Some([39858.0, -619.0, 20583.0])
        );
        assert_eq!(
            parse_setpoint_datagram(b" 1.5 , 2.5 , 3.5 , extra "),
            Some([1.5, 2.5, 3.5])
        );
        assert_eq!(parse_setpoint_datagram(b"1,2"), None);
        assert_eq!(parse_setpoint_datagram(b"1,2,nan"), None);
        assert_eq!(parse_setpoint_datagram(b"hello"), None);
        assert_eq!(parse_setpoint_datagram(&[0xFF, 0xFE]), None);
    }

    #[test]
    fn the_server_delivers_a_command_and_stops_cleanly() {
        let mut server = SetpointServer::default();
        let receiver = server.listen(DEFAULT_BIND_ADDRESS, 0).unwrap();
        let port = server.port().unwrap();

        let client = UdpSocket::bind(("127.0.0.1", 0)).unwrap();
        client
            .send_to(b"1000,-2000,3000", ("127.0.0.1", port))
            .unwrap();

        assert_eq!(
            receiver.recv_timeout(Duration::from_secs(5)).unwrap(),
            [1000.0, -2000.0, 3000.0]
        );

        server.disconnect();
        assert!(!server.is_listening());
    }
}
