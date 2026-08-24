mod cage;
mod netcfg;

use eframe::egui::{self, Color32};
use egui_plot::{Legend, Line, Plot, PlotPoints};
use igrf_core::geomagnetism::{
    Coordinate, GeomagnetismCalculator, GeomagnetismResult, UtcDateTime,
};
use igrf_core::{
    AppConfig, CalculationService, FilterSettings, PidController, PidSettings, ProcessedData,
    SensorService,
};
use igrf_io::{
    write_controller_packet, CsvLogger, MagsonSample, MagsonTcpClient, SerialPortManager,
};
use std::sync::mpsc::{self, Receiver};
use std::thread;
use std::time::{Duration, Instant, SystemTime};

const CONFIG_PATH: &str = "SystemConfig.json";
/// Column-for-column the header the C# build wrote, so existing analysis
/// scripts keep working against logs from either implementation.
const LOG_HEADER: &str = "Timestamp,MagX,MagY,MagZ,MagTotal,SetX,SetY,SetZ,SetTotal,ErrX,ErrY,ErrZ,OutX,OutY,OutZ,KpX,KiX,KdX,KpY,KiY,KdY,KpZ,KiZ,KdZ,Mag2X,Mag2Y,Mag2Z,Mag2Total";
const HANDSHAKE: [u8; 6] = [0x2A, 0x30, 0x30, 0x57, 0x45, 0x0D];
const HISTORY_LIMIT: usize = 500;
const PID_INTERVAL: Duration = Duration::from_millis(100);
const UI_INTERVAL: Duration = Duration::from_millis(50);
/// A running loop is stopped once the newest sensor packet is older than this.
const SENSOR_TIMEOUT: Duration = Duration::from_millis(1000);
/// The C# build reopened the port after this long without a packet, so an
/// unattended run survives a USB hiccup. The watchdog above only stops the
/// coils; it never brings the link back.
const SENSOR_RECONNECT_AFTER: Duration = Duration::from_secs(15);
/// How often a reconnect is retried while the sensor stays silent.
const RECONNECT_INTERVAL: Duration = Duration::from_secs(10);
const AXES: [char; 3] = ['X', 'Y', 'Z'];
const STOP_RED: Color32 = Color32::from_rgb(170, 45, 45);
/// Below this the side-by-side X/Y/Z layout stacks vertically instead.
const MIN_COLUMN_WIDTH: f32 = 190.0;

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1600.0, 1000.0])
            .with_min_inner_size([1100.0, 700.0]),
        ..Default::default()
    };
    eframe::run_native(
        "IGRF control",
        options,
        Box::new(|cc| Ok(Box::new(IgrfApp::new(cc)))),
    )
}

#[derive(Default)]
struct History {
    points: Vec<[f64; 2]>,
}

impl History {
    fn push(&mut self, x: f64, y: f64) {
        if !x.is_finite() || !y.is_finite() {
            return;
        }
        self.points.push([x, y]);
        if self.points.len() > HISTORY_LIMIT {
            let extra = self.points.len() - HISTORY_LIMIT;
            self.points.drain(..extra);
        }
    }

    fn clear(&mut self) {
        self.points.clear();
    }
}

struct PlotHistory {
    sensor_setpoint: [History; 3],
    sensor_measured: [History; 3],
    sensor_magnitude_setpoint: History,
    sensor_magnitude_measured: History,
    magson: [History; 4],
}

impl Default for PlotHistory {
    fn default() -> Self {
        Self {
            sensor_setpoint: [History::default(), History::default(), History::default()],
            sensor_measured: [History::default(), History::default(), History::default()],
            sensor_magnitude_setpoint: History::default(),
            sensor_magnitude_measured: History::default(),
            magson: [
                History::default(),
                History::default(),
                History::default(),
                History::default(),
            ],
        }
    }
}

impl PlotHistory {
    fn clear(&mut self) {
        for history in &mut self.sensor_setpoint {
            history.clear();
        }
        for history in &mut self.sensor_measured {
            history.clear();
        }
        self.sensor_magnitude_setpoint.clear();
        self.sensor_magnitude_measured.clear();
        for history in &mut self.magson {
            history.clear();
        }
    }
}

struct IgrfApp {
    config: AppConfig,
    sensor_port: String,
    sensor_baud: String,
    controller_port: String,
    controller_baud: String,
    magson_ip: String,
    magson_port: String,
    log_path: String,
    available_ports: Vec<String>,
    lan_profiles: Vec<netcfg::LanProfile>,
    lan_selected: usize,
    lan_cidr: String,
    lan_task: Option<Receiver<Result<String, String>>>,

    sensor_manager: SerialPortManager,
    controller_manager: SerialPortManager,
    magson_client: MagsonTcpClient,
    magson_receiver: Option<Receiver<MagsonSample>>,
    sensor_service: SensorService,
    calculation: CalculationService,
    pid_settings: [PidSettings; 3],
    filter_settings: [FilterSettings; 3],
    pids: [PidController; 3],
    pid_running: [bool; 3],

    raw: [f64; 3],
    calibrated: [f64; 3],
    filtered: [f64; 3],
    processed: ProcessedData,
    magson: [f64; 3],
    magson_total: f64,
    outputs: [f64; 3],
    history: PlotHistory,
    follow_plots: bool,
    cage: cage::CageView,
    started_at: Instant,
    last_pid_tick: Instant,
    last_handshake: Option<Instant>,
    last_sensor_packet: Option<Instant>,
    last_sensor_packet_wall: Option<SystemTime>,
    sensor_intended: bool,
    last_reconnect: Option<Instant>,
    resume_after_reconnect: bool,
    paused_by_watchdog: [bool; 3],
    resume_pending: bool,

    logger: Option<CsvLogger>,
    manual_lat: String,
    manual_lon: String,
    manual_altitude: String,
    manual_year: String,
    manual_month: String,
    manual_day: String,
    manual_result: Option<GeomagnetismResult>,
    manual_error: Option<String>,

    status: String,
    error: Option<String>,
}

impl IgrfApp {
    fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let (config, config_problem) = AppConfig::load(CONFIG_PATH);
        let pid_settings = [
            config.pid_x.clone(),
            config.pid_y.clone(),
            config.pid_z.clone(),
        ];
        let filter_settings = [
            config.filter_x.clone(),
            config.filter_y.clone(),
            config.filter_z.clone(),
        ];
        let pids = pid_settings.clone().map(pid_from_settings);
        let (available_ports, status) = match serialport::available_ports() {
            Ok(ports) => (stable_first(ports), "Ready".to_owned()),
            Err(error) => (Vec::new(), format!("Port scan unavailable: {error}")),
        };

        Self {
            sensor_port: config.sensor_port.clone(),
            sensor_baud: config.sensor_baud.to_string(),
            controller_port: config.controller_port.clone(),
            controller_baud: config.controller_baud.to_string(),
            magson_ip: config.sensor2_ip.clone(),
            magson_port: config.sensor2_port.to_string(),
            log_path: "sensor_log.csv".to_owned(),
            available_ports,
            lan_profiles: netcfg::list_wired().unwrap_or_default(),
            lan_selected: 0,
            lan_cidr: String::new(),
            lan_task: None,
            sensor_manager: SerialPortManager::default(),
            controller_manager: SerialPortManager::default(),
            magson_client: MagsonTcpClient::default(),
            magson_receiver: None,
            sensor_service: SensorService::default(),
            calculation: CalculationService::default(),
            pid_settings,
            filter_settings,
            pids,
            pid_running: [false; 3],
            raw: [0.0; 3],
            calibrated: [0.0; 3],
            filtered: [0.0; 3],
            processed: ProcessedData::default(),
            magson: [0.0; 3],
            magson_total: 0.0,
            outputs: [0.0; 3],
            history: PlotHistory::default(),
            follow_plots: true,
            cage: cage::CageView::default(),
            started_at: Instant::now(),
            last_pid_tick: Instant::now(),
            last_handshake: None,
            last_sensor_packet: None,
            last_sensor_packet_wall: None,
            sensor_intended: false,
            last_reconnect: None,
            resume_after_reconnect: false,
            paused_by_watchdog: [false; 3],
            resume_pending: false,
            logger: None,
            manual_lat: "13.7563".to_owned(),
            manual_lon: "100.5018".to_owned(),
            manual_altitude: "0".to_owned(),
            manual_year: "2025".to_owned(),
            manual_month: "1".to_owned(),
            manual_day: "1".to_owned(),
            manual_result: None,
            manual_error: None,
            status,
            error: config_problem,
            config,
        }
    }

    fn set_status(&mut self, message: impl Into<String>) {
        self.status = message.into();
        self.error = None;
    }

    fn set_error(&mut self, message: impl Into<String>) {
        self.error = Some(message.into());
    }

    fn refresh_ports(&mut self) {
        match serialport::available_ports() {
            Ok(ports) => {
                self.available_ports = stable_first(ports);
                self.set_status(format!(
                    "Found {} serial port(s)",
                    self.available_ports.len()
                ));
            }
            Err(error) => self.set_error(format!("Cannot list serial ports: {error}")),
        }
    }

    fn refresh_lan(&mut self) {
        match netcfg::list_wired() {
            Ok(profiles) => {
                self.lan_profiles = profiles;
                self.lan_selected = self
                    .lan_selected
                    .min(self.lan_profiles.len().saturating_sub(1));
                if self.lan_cidr.trim().is_empty() {
                    self.lan_cidr = self
                        .lan_profiles
                        .get(self.lan_selected)
                        .map(|profile| profile.addresses.clone())
                        .unwrap_or_default();
                }
            }
            Err(error) => self.set_error(format!("Cannot list LAN profiles: {error}")),
        }
    }

    /// nmcli takes seconds to bring a profile back up, so the work runs off the
    /// UI thread and the result is picked up in `poll_io`.
    fn spawn_lan_task<F>(&mut self, task: F)
    where
        F: FnOnce() -> Result<String, String> + Send + 'static,
    {
        if self.lan_task.is_some() {
            return;
        }
        let (sender, receiver) = mpsc::channel();
        self.lan_task = Some(receiver);
        thread::spawn(move || {
            let _ = sender.send(task());
        });
        self.set_status("Applying LAN configuration...");
    }

    fn apply_lan_static(&mut self) {
        let Some(target) = self.lan_profiles.get(self.lan_selected).cloned() else {
            self.set_error("No LAN profile selected");
            return;
        };
        let cidr = self.lan_cidr.trim().to_owned();
        if let Err(error) = netcfg::validate_cidr(&cidr) {
            self.set_error(format!("LAN address: {error}"));
            return;
        }
        self.spawn_lan_task(move || netcfg::apply_static(&target, &cidr));
    }

    fn apply_lan_dhcp(&mut self) {
        let Some(target) = self.lan_profiles.get(self.lan_selected).cloned() else {
            self.set_error("No LAN profile selected");
            return;
        };
        self.spawn_lan_task(move || netcfg::apply_dhcp(&target));
    }

    fn poll_lan_task(&mut self) {
        let Some(receiver) = &self.lan_task else {
            return;
        };
        match receiver.try_recv() {
            Ok(result) => {
                self.lan_task = None;
                match result {
                    Ok(message) => self.set_status(message),
                    Err(error) => self.set_error(format!("LAN: {error}")),
                }
                self.refresh_lan();
            }
            Err(mpsc::TryRecvError::Empty) => {}
            Err(mpsc::TryRecvError::Disconnected) => {
                self.lan_task = None;
                self.set_error("LAN task ended without a result");
            }
        }
    }

    fn connect_sensor(&mut self) {
        let baud = match parse_baud(&self.sensor_baud) {
            Ok(value) => value,
            Err(error) => {
                self.set_error(format!("Sensor baud: {error}"));
                return;
            }
        };
        if self.sensor_port.trim().is_empty() {
            self.set_error("Sensor port is empty");
            return;
        }
        match self.sensor_manager.connect(self.sensor_port.trim(), baud) {
            Ok(()) => {
                self.last_handshake = None;
                self.last_sensor_packet = None;
                self.last_sensor_packet_wall = None;
                self.sensor_intended = true;
                self.set_status(format!(
                    "Sensor connected: {} @ {baud}",
                    self.sensor_port.trim()
                ));
            }
            Err(error) => self.set_error(format!("Sensor connect failed: {error}")),
        }
    }

    fn disconnect_sensor(&mut self) {
        self.sensor_manager.disconnect();
        self.last_sensor_packet = None;
        self.last_sensor_packet_wall = None;
        self.sensor_intended = false;
        self.resume_pending = false;
        self.paused_by_watchdog = [false; 3];
        self.set_status("Sensor disconnected");
    }

    fn connect_controller(&mut self) {
        let baud = match parse_baud(&self.controller_baud) {
            Ok(value) => value,
            Err(error) => {
                self.set_error(format!("Controller baud: {error}"));
                return;
            }
        };
        if self.controller_port.trim().is_empty() {
            self.set_error("Controller port is empty");
            return;
        }
        match self
            .controller_manager
            .connect(self.controller_port.trim(), baud)
        {
            Ok(()) => self.set_status(format!(
                "Controller connected: {} @ {baud}",
                self.controller_port.trim()
            )),
            Err(error) => self.set_error(format!("Controller connect failed: {error}")),
        }
    }

    fn disconnect_controller(&mut self) {
        self.controller_manager.disconnect();
        self.set_status("Controller disconnected");
    }

    fn connect_magson(&mut self) {
        let port = match parse_tcp_port(&self.magson_port) {
            Ok(value) => value,
            Err(error) => {
                self.set_error(format!("Magson port: {error}"));
                return;
            }
        };
        if self.magson_ip.trim().is_empty() {
            self.set_error("Magson IP/host is empty");
            return;
        }
        match self.magson_client.connect(self.magson_ip.trim(), port) {
            Ok(receiver) => {
                self.magson_receiver = Some(receiver);
                self.set_status(format!(
                    "Magson connected: {}:{port}",
                    self.magson_ip.trim()
                ));
            }
            Err(error) => self.set_error(format!("Magson connect failed: {error}")),
        }
    }

    fn disconnect_magson(&mut self) {
        self.magson_client.disconnect();
        self.magson_receiver = None;
        self.set_status("Magson disconnected");
    }

    fn apply_filter_settings(&mut self) {
        for axis in 0..3 {
            let settings = self.filter_settings[axis].clone();
            if self
                .calculation
                .set_noise(axis, settings.q, settings.r)
                .is_err()
            {
                self.filter_settings[axis].sanitize();
                self.set_error(format!(
                    "Filter {}: Q and R must be finite and above zero; restored defaults",
                    AXES[axis]
                ));
            }
        }
    }

    /// Reopens the sensor port by itself, matching the C# watchdog: a run left
    /// alone for a month must survive a cable or driver hiccup without someone
    /// there to press Connect.
    fn maybe_reconnect_sensor(&mut self) {
        if !self.sensor_intended
            || !self
                .sensor_age()
                .is_none_or(|age| age > SENSOR_RECONNECT_AFTER)
        {
            return;
        }
        if self
            .last_reconnect
            .is_some_and(|last| last.elapsed() < RECONNECT_INTERVAL)
        {
            return;
        }
        self.last_reconnect = Some(Instant::now());
        self.sensor_manager.disconnect();
        self.connect_sensor();
        if self.sensor_manager.is_open() && self.resume_after_reconnect {
            // Wait for a real packet before driving the coils again; the
            // watchdog would only have to stop them a tick later otherwise.
            self.resume_pending = true;
        }
    }

    fn poll_io(&mut self) {
        self.poll_lan_task();
        self.apply_filter_settings();
        self.maybe_reconnect_sensor();
        if self.sensor_manager.is_open() {
            let now = Instant::now();
            let should_handshake = self
                .last_handshake
                .map(|sent| now.duration_since(sent) >= Duration::from_millis(500))
                .unwrap_or(true);
            if !self.sensor_manager.parser().is_sensor_ready() && should_handshake {
                self.last_handshake = Some(now);
                if let Err(error) = self.sensor_manager.write(&HANDSHAKE) {
                    self.set_error(format!("Sensor handshake failed: {error}"));
                }
            }

            match self.sensor_manager.read_available() {
                Ok(packets) => {
                    for packet in packets {
                        self.handle_sensor_packet(&packet);
                    }
                }
                Err(error) => {
                    self.set_error(format!("Sensor read failed: {error}"));
                    self.sensor_manager.disconnect();
                }
            }
        }

        let samples: Vec<MagsonSample> = self
            .magson_receiver
            .as_ref()
            .map(|receiver| receiver.try_iter().collect())
            .unwrap_or_default();
        for sample in samples {
            self.handle_magson_sample(sample);
        }
        if self.magson_receiver.is_some() && !self.magson_client.is_open() {
            self.magson_receiver = None;
            self.set_error("Magson connection closed");
        }
    }

    fn handle_sensor_packet(&mut self, packet: &[u8]) {
        self.last_sensor_packet = Some(Instant::now());
        self.last_sensor_packet_wall = Some(SystemTime::now());
        let calibrated = self.sensor_service.process_data(packet);
        self.raw = [
            self.sensor_service.last_raw_x(),
            self.sensor_service.last_raw_y(),
            self.sensor_service.last_raw_z(),
        ];
        self.calibrated = [calibrated.mag_x, calibrated.mag_y, calibrated.mag_z];
        self.processed = self.calculation.process_sensor_data(
            &calibrated,
            self.pid_settings[0].setpoint,
            self.pid_settings[1].setpoint,
            self.pid_settings[2].setpoint,
        );
        self.filtered = [
            self.processed.mag_x,
            self.processed.mag_y,
            self.processed.mag_z,
        ];
        let time = self.started_at.elapsed().as_secs_f64();
        for axis in 0..3 {
            self.history.sensor_setpoint[axis].push(time, self.pid_settings[axis].setpoint);
            self.history.sensor_measured[axis].push(time, self.filtered[axis]);
        }
        let measured_magnitude = self
            .filtered
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt();
        let setpoint_magnitude = self
            .pid_settings
            .iter()
            .map(|settings| settings.setpoint * settings.setpoint)
            .sum::<f64>()
            .sqrt();
        self.history
            .sensor_magnitude_setpoint
            .push(time, setpoint_magnitude);
        self.history
            .sensor_magnitude_measured
            .push(time, measured_magnitude);
    }

    fn handle_magson_sample(&mut self, sample: MagsonSample) {
        self.magson = [sample.bx, sample.by, sample.bz];
        self.magson_total = self
            .magson
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt();
        let time = self.started_at.elapsed().as_secs_f64();
        for axis in 0..3 {
            self.history.magson[axis].push(time, self.magson[axis]);
        }
        self.history.magson[3].push(time, self.magson_total);
    }

    /// Age of the newest sensor packet; `None` means none has arrived yet.
    ///
    /// `Instant` is CLOCK_MONOTONIC on Linux and does not advance while the
    /// machine is suspended, so waking from a lid-close would look like no time
    /// passed and the watchdog would never fire. The wall clock sees that gap;
    /// taking the larger of the two also keeps an NTP step backwards from
    /// hiding a real stall.
    fn sensor_age(&self) -> Option<Duration> {
        let monotonic = self.last_sensor_packet?.elapsed();
        let wall = self
            .last_sensor_packet_wall
            .and_then(|at| at.elapsed().ok())
            .unwrap_or(Duration::ZERO);
        Some(monotonic.max(wall))
    }

    fn run_pid(&mut self) {
        let now = Instant::now();
        if now.duration_since(self.last_pid_tick) < PID_INTERVAL {
            return;
        }
        self.last_pid_tick = now;
        if let Some((axis, error)) =
            self.pid_settings
                .iter()
                .enumerate()
                .find_map(|(axis, settings)| {
                    validate_pid_settings(settings)
                        .err()
                        .map(|error| (axis, error))
                })
        {
            self.pid_running = [false; 3];
            self.outputs = [0.0; 3];
            self.pid_settings[axis].sanitize();
            self.set_error(format!(
                "PID {}: {error}; restored safe defaults",
                AXES[axis]
            ));
            return;
        }
        if self.pid_running.iter().any(|state| *state) && sensor_is_stale(self.sensor_age()) {
            let reason = match self.sensor_age() {
                Some(age) => format!("no sensor data for {:.1}s", age.as_secs_f64()),
                None => "no sensor data received yet".to_owned(),
            };
            self.paused_by_watchdog = self.pid_running;
            self.stop_all();
            self.set_error(format!("PID stopped: {reason}"));
            return;
        }

        if self.resume_pending {
            self.resume_pending = false;
            self.pid_running = self.paused_by_watchdog;
            self.paused_by_watchdog = [false; 3];
            if self.pid_running.iter().any(|state| *state) {
                self.set_status("Sensor back; PID resumed");
            }
        }

        for axis in 0..3 {
            apply_pid_settings(&mut self.pids[axis], &self.pid_settings[axis]);
            self.outputs[axis] = if self.pid_running[axis] {
                self.pids[axis].calculate(self.pid_settings[axis].setpoint, self.filtered[axis])
            } else {
                0.0
            };
        }

        if self.controller_manager.is_open() {
            if let Err(error) = write_controller_packet(
                &mut self.controller_manager,
                self.outputs[0],
                self.outputs[1],
                self.outputs[2],
            ) {
                self.set_error(format!("Controller write failed: {error}"));
                self.controller_manager.disconnect();
            }
        }
        self.log_snapshot();
    }

    fn log_snapshot(&mut self) {
        if self.logger.is_none() {
            return;
        }
        // Matches the C# row exactly: filtered field as Mag*, the unsigned error
        // from ProcessedData, F2 everywhere except the F3 gains.
        let timestamp = chrono::Local::now().format("%Y-%m-%d %H:%M:%S%.3f");
        let magnitude =
            |values: [f64; 3]| values.iter().map(|value| value * value).sum::<f64>().sqrt();
        let setpoints = [
            self.pid_settings[0].setpoint,
            self.pid_settings[1].setpoint,
            self.pid_settings[2].setpoint,
        ];
        let errors = [
            self.processed.error_x,
            self.processed.error_y,
            self.processed.error_z,
        ];
        let line = format!(
            "{timestamp},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.2},{:.2},{:.2},{:.2}",
            self.filtered[0],
            self.filtered[1],
            self.filtered[2],
            magnitude(self.filtered),
            setpoints[0],
            setpoints[1],
            setpoints[2],
            magnitude(setpoints),
            errors[0],
            errors[1],
            errors[2],
            self.outputs[0],
            self.outputs[1],
            self.outputs[2],
            self.pid_settings[0].kp,
            self.pid_settings[0].ki,
            self.pid_settings[0].kd,
            self.pid_settings[1].kp,
            self.pid_settings[1].ki,
            self.pid_settings[1].kd,
            self.pid_settings[2].kp,
            self.pid_settings[2].ki,
            self.pid_settings[2].kd,
            self.magson[0],
            self.magson[1],
            self.magson[2],
            self.magson_total,
        );
        let result = self.logger.as_mut().map(|logger| logger.write_line(&line));
        if let Some(Err(error)) = result {
            self.logger = None;
            self.set_error(format!("CSV write failed: {error}"));
        }
    }

    fn start_logging(&mut self) {
        if self.log_path.trim().is_empty() {
            self.set_error("Log path is empty");
            return;
        }
        match CsvLogger::open(self.log_path.trim(), LOG_HEADER) {
            Ok(logger) => {
                self.set_status(format!("Logging to {}", logger.path().display()));
                self.logger = Some(logger);
            }
            Err(error) => self.set_error(format!("Cannot start CSV logging: {error}")),
        }
    }

    fn stop_logging(&mut self) {
        self.logger = None;
        self.set_status("Logging stopped");
    }

    fn save_config(&mut self) {
        let sensor_baud = match parse_baud(&self.sensor_baud) {
            Ok(value) => value,
            Err(error) => {
                self.set_error(format!("Cannot save sensor baud: {error}"));
                return;
            }
        };
        let controller_baud = match parse_baud(&self.controller_baud) {
            Ok(value) => value,
            Err(error) => {
                self.set_error(format!("Cannot save controller baud: {error}"));
                return;
            }
        };
        let magson_port = match parse_tcp_port(&self.magson_port) {
            Ok(value) => value,
            Err(error) => {
                self.set_error(format!("Cannot save Magson port: {error}"));
                return;
            }
        };
        self.config.pid_x = self.pid_settings[0].clone();
        self.config.pid_y = self.pid_settings[1].clone();
        self.config.pid_z = self.pid_settings[2].clone();
        self.config.filter_x = self.filter_settings[0].clone();
        self.config.filter_y = self.filter_settings[1].clone();
        self.config.filter_z = self.filter_settings[2].clone();
        self.config.sensor_port = self.sensor_port.clone();
        self.config.sensor_baud = sensor_baud;
        self.config.controller_port = self.controller_port.clone();
        self.config.controller_baud = controller_baud;
        self.config.sensor2_ip = self.magson_ip.clone();
        self.config.sensor2_port = i32::from(magson_port);
        self.config.sanitize();
        match self.config.save(CONFIG_PATH) {
            Ok(()) => self.set_status(format!("Saved {CONFIG_PATH}")),
            Err(error) => self.set_error(format!("Cannot save {CONFIG_PATH}: {error}")),
        }
    }

    fn load_config(&mut self) {
        let (config, problem) = AppConfig::load(CONFIG_PATH);
        self.sensor_port = config.sensor_port.clone();
        self.sensor_baud = config.sensor_baud.to_string();
        self.controller_port = config.controller_port.clone();
        self.controller_baud = config.controller_baud.to_string();
        self.magson_ip = config.sensor2_ip.clone();
        self.magson_port = config.sensor2_port.to_string();
        self.pid_settings = [
            config.pid_x.clone(),
            config.pid_y.clone(),
            config.pid_z.clone(),
        ];
        self.pids = self.pid_settings.clone().map(pid_from_settings);
        self.filter_settings = [
            config.filter_x.clone(),
            config.filter_y.clone(),
            config.filter_z.clone(),
        ];
        self.config = config;
        match problem {
            Some(message) => self.set_error(message),
            None => self.set_status(format!("Loaded {CONFIG_PATH}")),
        }
    }

    fn reset_axis(&mut self, axis: usize) {
        self.pids[axis].reset();
        match axis {
            0 => self.calculation.reset_filter_x(),
            1 => self.calculation.reset_filter_y(),
            _ => self.calculation.reset_filter_z(),
        }
        self.outputs[axis] = 0.0;
        self.set_status(format!("Reset axis {} PID/filter", ['X', 'Y', 'Z'][axis]));
    }

    fn master_reset(&mut self) {
        self.stop_all();
        for pid in &mut self.pids {
            pid.reset();
        }
        self.calculation.reset_filters();
        self.outputs = [0.0; 3];
        self.filtered = [0.0; 3];
        self.processed = ProcessedData::default();
        self.history.clear();
        if self.error.is_none() {
            self.set_status("Master reset complete");
        }
    }

    fn calculate_manual_wmm(&mut self) {
        let result = (|| {
            let latitude = parse_f64(&self.manual_lat, "latitude")?;
            let longitude = parse_f64(&self.manual_lon, "longitude")?;
            let altitude = parse_f64(&self.manual_altitude, "altitude")?;
            let year = parse_i32(&self.manual_year, "year")?;
            let month = parse_u8(&self.manual_month, "month")?;
            let day = parse_u8(&self.manual_day, "day")?;
            let coordinate =
                Coordinate::new(latitude, longitude).map_err(|error| error.to_string())?;
            let date = UtcDateTime::date(year, month, day).map_err(|error| error.to_string())?;
            GeomagnetismCalculator::new()
                .try_calculate_at_altitude(coordinate, altitude, date)
                .map_err(|error| error.to_string())?
                .ok_or_else(|| "date is outside WMM2025 validity (2025-2030)".to_owned())
        })();
        match result {
            Ok(value) => {
                self.manual_result = Some(value);
                self.manual_error = None;
                self.set_status("Manual WMM2025 calculation complete");
            }
            Err(error) => {
                self.manual_result = None;
                self.manual_error = Some(error);
            }
        }
    }

    fn stop_all(&mut self) {
        self.pid_running = [false; 3];
        self.outputs = [0.0; 3];
        for pid in &mut self.pids {
            pid.reset();
        }
        if self.controller_manager.is_open() {
            if let Err(error) = write_controller_packet(&mut self.controller_manager, 0.0, 0.0, 0.0)
            {
                self.controller_manager.disconnect();
                self.set_error(format!("STOP ALL: controller write failed: {error}"));
                return;
            }
        }
        self.set_status("STOP ALL: every axis paused, outputs zeroed");
    }

    fn show_top_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.heading("IGRF control");
            ui.separator();
            let sensor_age = self.sensor_age();
            status_pill(
                ui,
                "Sensor",
                if !self.sensor_manager.is_open() {
                    LinkState::Off
                } else if self.sensor_manager.parser().is_sensor_ready()
                    && !sensor_is_stale(sensor_age)
                {
                    LinkState::On
                } else {
                    LinkState::Wait
                },
            );
            if self.sensor_manager.is_open() {
                ui.label(
                    egui::RichText::new(match sensor_age {
                        Some(age) => format!("{:.1}s", age.as_secs_f64()),
                        None => "no data".to_owned(),
                    })
                    .small()
                    .weak(),
                );
            }
            status_pill(
                ui,
                "Controller",
                LinkState::from_open(self.controller_manager.is_open()),
            );
            status_pill(
                ui,
                "Magson",
                LinkState::from_open(self.magson_client.is_open()),
            );
            status_pill(ui, "CSV", LinkState::from_open(self.logger.is_some()));
            ui.separator();
            let running = self.pid_running.iter().filter(|state| **state).count();
            ui.label(format!("PID {running}/3 running"));

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let stop = egui::Button::new(
                    egui::RichText::new("STOP ALL")
                        .strong()
                        .color(Color32::WHITE),
                )
                .fill(STOP_RED)
                .min_size(egui::vec2(110.0, 26.0));
                if ui.add(stop).clicked() {
                    self.stop_all();
                }
                if ui.button("Master reset").clicked() {
                    self.master_reset();
                }
            });
        });
    }

    fn show_connection_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            if ui.button("Refresh ports").clicked() {
                self.refresh_ports();
            }
            ui.label(format!("{} detected", self.available_ports.len()));
        });
        ui.separator();
        ui.label("Sensor serial");
        port_selector(
            ui,
            "sensor-port",
            &mut self.sensor_port,
            &self.available_ports,
        );
        ui.horizontal(|ui| {
            ui.label("Baud");
            ui.add(egui::TextEdit::singleline(&mut self.sensor_baud).desired_width(64.0));
            if ui.button("Connect").clicked() {
                self.connect_sensor();
            }
            if ui.button("Disconnect").clicked() {
                self.disconnect_sensor();
            }
        });
        ui.label(if self.sensor_manager.is_open() {
            if self.sensor_manager.parser().is_sensor_ready() {
                "Sensor: connected / ready"
            } else {
                "Sensor: connected / waiting for OK"
            }
        } else {
            "Sensor: disconnected"
        });
        ui.checkbox(
            &mut self.resume_after_reconnect,
            "Resume PID after auto-reconnect",
        )
        .on_hover_text(
            "Off: the loop stays paused until someone starts it. On: it restarts \
             the coils by itself once packets return - only for unattended runs.",
        );

        ui.separator();
        ui.label("Controller serial");
        port_selector(
            ui,
            "controller-port",
            &mut self.controller_port,
            &self.available_ports,
        );
        ui.horizontal(|ui| {
            ui.label("Baud");
            ui.add(egui::TextEdit::singleline(&mut self.controller_baud).desired_width(64.0));
            if ui.button("Connect").clicked() {
                self.connect_controller();
            }
            if ui.button("Disconnect").clicked() {
                self.disconnect_controller();
            }
        });
        ui.label(if self.controller_manager.is_open() {
            "Controller: connected"
        } else {
            "Controller: disconnected"
        });

        ui.separator();
        ui.label("Magson TCP");
        ui.horizontal(|ui| {
            ui.label("IP/host");
            ui.add(egui::TextEdit::singleline(&mut self.magson_ip).desired_width(120.0));
        });
        ui.horizontal(|ui| {
            ui.label("Port");
            ui.add(egui::TextEdit::singleline(&mut self.magson_port).desired_width(64.0));
            if ui.button("Connect").clicked() {
                self.connect_magson();
            }
            if ui.button("Disconnect").clicked() {
                self.disconnect_magson();
            }
        });
        ui.label(if self.magson_client.is_open() {
            "Magson: connected"
        } else {
            "Magson: disconnected"
        });
    }

    fn show_lan_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            if ui.button("Refresh").clicked() {
                self.refresh_lan();
            }
            if self.lan_task.is_some() {
                ui.spinner();
                ui.label("applying...");
            }
        });
        if self.lan_profiles.is_empty() {
            ui.label("No wired NetworkManager profile found");
            return;
        }

        let selected = self.lan_selected.min(self.lan_profiles.len() - 1);
        self.lan_selected = selected;
        let labels: Vec<String> = self
            .lan_profiles
            .iter()
            .map(|profile| profile.label())
            .collect();
        egui::ComboBox::from_id_salt("lan-profile")
            .selected_text(labels[selected].clone())
            .show_ui(ui, |ui| {
                for (index, label) in labels.iter().enumerate() {
                    ui.selectable_value(&mut self.lan_selected, index, label);
                }
            });

        let target = self.lan_profiles[self.lan_selected].clone();
        ui.label(
            egui::RichText::new(format!(
                "now: {} {}",
                target.method,
                if target.addresses.is_empty() {
                    "--"
                } else {
                    &target.addresses
                }
            ))
            .small()
            .weak(),
        );
        if target.carries_default_route {
            ui.colored_label(
                Color32::LIGHT_RED,
                "carries the default route - locked to avoid cutting this machine off",
            );
            return;
        }

        ui.horizontal(|ui| {
            ui.label("Address");
            ui.add(egui::TextEdit::singleline(&mut self.lan_cidr).desired_width(130.0));
        });
        let busy = self.lan_task.is_some();
        ui.horizontal(|ui| {
            if ui
                .add_enabled(!busy, egui::Button::new("Apply static"))
                .clicked()
            {
                self.apply_lan_static();
            }
            if ui
                .add_enabled(!busy, egui::Button::new("Use DHCP"))
                .clicked()
            {
                self.apply_lan_dhcp();
            }
        });
    }

    fn show_config_panel(&mut self, ui: &mut egui::Ui) {
        if ui.button("Load SystemConfig.json").clicked() {
            self.load_config();
        }
        if ui.button("Save SystemConfig.json").clicked() {
            self.save_config();
        }
        ui.label("CSV path");
        ui.horizontal(|ui| {
            ui.add(egui::TextEdit::singleline(&mut self.log_path).desired_width(150.0));
            if self.logger.is_some() {
                if ui.button("Stop").clicked() {
                    self.stop_logging();
                }
            } else if ui.button("Start").clicked() {
                self.start_logging();
            }
        });
        ui.label(if self.logger.is_some() {
            "CSV: logging"
        } else {
            "CSV: stopped"
        });
    }

    fn show_manual_panel(&mut self, ui: &mut egui::Ui) {
        ui.label("Default date is fixed at 2025-01-01 (valid WMM2025 date).");
        egui::Grid::new("manual-wmm-grid")
            .num_columns(2)
            .show(ui, |ui| {
                for (label, value) in [
                    ("Latitude", &mut self.manual_lat),
                    ("Longitude", &mut self.manual_lon),
                    ("Altitude km", &mut self.manual_altitude),
                    ("Year", &mut self.manual_year),
                    ("Month", &mut self.manual_month),
                    ("Day", &mut self.manual_day),
                ] {
                    ui.label(label);
                    ui.text_edit_singleline(value);
                    ui.end_row();
                }
            });
        if ui.button("Calculate WMM2025").clicked() {
            self.calculate_manual_wmm();
        }
        if let Some(error) = &self.manual_error {
            ui.colored_label(Color32::LIGHT_RED, error);
        }
        if let Some(result) = self.manual_result {
            egui::Grid::new("manual-wmm-result")
                .num_columns(2)
                .striped(true)
                .show(ui, |ui| {
                    for (label, value) in [
                        ("X", result.x),
                        ("Y", result.y),
                        ("Z", result.z),
                        ("Declination", result.declination),
                        ("Inclination", result.inclination),
                        ("Intensity", result.total_intensity),
                    ] {
                        ui.label(label);
                        ui.label(format!("{value:.4}"));
                        ui.end_row();
                    }
                });
        }
    }

    /// One X/Y/Z card: live readout on top, PID gains below. Always visible so
    /// nothing that can stop an axis hides behind navigation.
    fn axis_column(&mut self, ui: &mut egui::Ui, axis: usize) {
        let label = AXES[axis];
        ui.group(|ui| {
            ui.set_min_width(ui.available_width().max(0.0));
            let running = self.pid_running[axis];
            ui.horizontal(|ui| {
                status_pill(ui, &format!("Axis {label}"), LinkState::from_open(running));
                if ui.button(if running { "Pause" } else { "Start" }).clicked() {
                    self.pid_running[axis] = !running;
                    if !self.pid_running[axis] {
                        self.outputs[axis] = 0.0;
                    }
                }
                if ui.button("Reset").clicked() {
                    self.reset_axis(axis);
                }
            });

            // `ProcessedData` carries |error| for C# parity, which cannot tell
            // overshoot from undershoot; recompute the signed value for display.
            let error = self.pid_settings[axis].setpoint - self.filtered[axis];
            let error_percent = [
                self.processed.error_per_x,
                self.processed.error_per_y,
                self.processed.error_per_z,
            ][axis];

            ui.add_space(2.0);
            ui.label(
                egui::RichText::new(format!("{:+.3}", self.filtered[axis]))
                    .monospace()
                    .size(26.0)
                    .strong(),
            );
            ui.label(egui::RichText::new("filtered").small().weak());
            ui.label(format!("set {:+.3}", self.pid_settings[axis].setpoint));
            if self.pid_settings[axis].setpoint == 0.0 {
                // A percentage against a zero setpoint is undefined, and
                // `calculate_percent` reports 0.0 there - which would paint a
                // fully saturated axis green. Show the raw error only.
                ui.label(format!("err {error:+.3}"));
            } else {
                ui.colored_label(
                    error_color(error_percent),
                    format!("err {error:+.3} ({error_percent:.2}%)"),
                );
            }
            ui.horizontal(|ui| {
                ui.label(
                    egui::RichText::new(format!("raw {:+.3}", self.raw[axis]))
                        .small()
                        .weak(),
                );
                ui.label(
                    egui::RichText::new(format!("cal {:+.3}", self.calibrated[axis]))
                        .small()
                        .weak(),
                );
            });

            ui.add_space(4.0);
            let fraction = output_fraction(
                self.outputs[axis],
                self.pid_settings[axis].min_output,
                self.pid_settings[axis].max_output,
            );
            // At the limit the loop has no authority left, so the reading looks
            // steady for the wrong reason. Say so instead of just filling the bar.
            let saturated = running
                && (self.outputs[axis] >= self.pid_settings[axis].max_output
                    || self.outputs[axis] <= self.pid_settings[axis].min_output);
            let mut bar = egui::ProgressBar::new(fraction as f32).text(if saturated {
                format!("output {:+.3}  SATURATED", self.outputs[axis])
            } else {
                format!("output {:+.3}", self.outputs[axis])
            });
            if saturated {
                bar = bar.fill(STOP_RED);
            }
            ui.add(bar);

            ui.separator();
            egui::Grid::new(format!("pid-grid-{axis}"))
                .num_columns(2)
                .striped(true)
                .show(ui, |ui| {
                    for (name, value) in [
                        ("Kp", &mut self.pid_settings[axis].kp),
                        ("Ki", &mut self.pid_settings[axis].ki),
                        ("Kd", &mut self.pid_settings[axis].kd),
                        ("Min out", &mut self.pid_settings[axis].min_output),
                        ("Max out", &mut self.pid_settings[axis].max_output),
                        ("Setpoint", &mut self.pid_settings[axis].setpoint),
                    ] {
                        ui.label(name);
                        ui.add(egui::DragValue::new(value).speed(0.1));
                        ui.end_row();
                    }
                });
            ui.label(egui::RichText::new("Kalman filter").small().weak());
            egui::Grid::new(format!("filter-grid-{axis}"))
                .num_columns(2)
                .striped(true)
                .show(ui, |ui| {
                    for (name, value, speed) in [
                        ("Q process", &mut self.filter_settings[axis].q, 0.05),
                        ("R measure", &mut self.filter_settings[axis].r, 1.0),
                    ] {
                        ui.label(name);
                        ui.add(egui::DragValue::new(value).speed(speed).range(1e-6..=1e9));
                        ui.end_row();
                    }
                });
        });
    }

    fn show_axis_row(&mut self, ui: &mut egui::Ui) {
        if fits_columns(ui, 3) {
            ui.columns(3, |columns| {
                for axis in 0..3 {
                    self.axis_column(&mut columns[axis], axis);
                }
            });
        } else {
            for axis in 0..3 {
                self.axis_column(ui, axis);
            }
        }
    }

    fn show_magson_strip(&self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("Magson").strong());
            ui.separator();
            for (label, value) in [
                ("X", self.magson[0]),
                ("Y", self.magson[1]),
                ("Z", self.magson[2]),
                ("|B|", self.magson_total),
            ] {
                ui.label(egui::RichText::new(format!("{label} {value:+.3}")).monospace());
                ui.add_space(6.0);
            }
        });
    }

    fn show_cage(&mut self, ui: &mut egui::Ui) {
        // The view wants signed drive normalised against each axis' own limit,
        // not raw controller units.
        let drive = std::array::from_fn(|axis| {
            let settings = &self.pid_settings[axis];
            let span = settings.min_output.abs().max(settings.max_output.abs());
            if span > 0.0 {
                (self.outputs[axis] / span).clamp(-1.0, 1.0)
            } else {
                0.0
            }
        });
        egui::CollapsingHeader::new("Coil cage")
            .default_open(true)
            .show(ui, |ui| cage::show(ui, &mut self.cage, drive));
    }

    fn show_plots(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("Live plots (last 500 points)").strong());
            ui.checkbox(&mut self.follow_plots, "Follow live");
            ui.label(
                egui::RichText::new(if self.follow_plots {
                    "auto-scaling \u{2022} uncheck to pan/zoom"
                } else {
                    "drag = pan \u{2022} ctrl+scroll = zoom \u{2022} right-drag = box \u{2022} double-click = reset"
                })
                .small()
                .weak(),
            );
        });
        let follow = self.follow_plots;
        let axis_plot = |ui: &mut egui::Ui, axis: usize| {
            show_plot(
                ui,
                &format!("sensor-plot-{axis}"),
                &format!("{}: setpoint vs measured", AXES[axis]),
                &[
                    (
                        "Setpoint",
                        &self.history.sensor_setpoint[axis],
                        Color32::LIGHT_RED,
                    ),
                    (
                        "Measured",
                        &self.history.sensor_measured[axis],
                        Color32::LIGHT_BLUE,
                    ),
                ],
                follow,
            );
        };
        if fits_columns(ui, 3) {
            ui.columns(3, |columns| {
                for axis in 0..3 {
                    axis_plot(&mut columns[axis], axis);
                }
            });
        } else {
            for axis in 0..3 {
                axis_plot(ui, axis);
            }
        }
    }

    /// Cage on the left, the two whole-system plots stacked on the right, so the
    /// square 3D view does not leave half a row empty.
    fn show_cage_row(&mut self, ui: &mut egui::Ui) {
        if fits_columns(ui, 2) {
            ui.columns(2, |columns| {
                self.show_cage(&mut columns[0]);
                self.show_summary_plots(&mut columns[1]);
            });
        } else {
            self.show_cage(ui);
            self.show_summary_plots(ui);
        }
    }

    fn show_summary_plots(&self, ui: &mut egui::Ui) {
        let follow = self.follow_plots;
        let magnitude_plot = |ui: &mut egui::Ui| {
            show_plot(
                ui,
                "sensor-magnitude-plot",
                "|B|: setpoint vs measured",
                &[
                    (
                        "Setpoint",
                        &self.history.sensor_magnitude_setpoint,
                        Color32::LIGHT_RED,
                    ),
                    (
                        "Measured",
                        &self.history.sensor_magnitude_measured,
                        Color32::LIGHT_BLUE,
                    ),
                ],
                follow,
            );
        };
        let magson_plot = |ui: &mut egui::Ui| {
            show_plot(
                ui,
                "magson-plot",
                "Magson X/Y/Z/total",
                &[
                    ("X", &self.history.magson[0], Color32::LIGHT_RED),
                    ("Y", &self.history.magson[1], Color32::LIGHT_GREEN),
                    ("Z", &self.history.magson[2], Color32::LIGHT_BLUE),
                    ("Total", &self.history.magson[3], Color32::YELLOW),
                ],
                follow,
            );
        };
        magnitude_plot(ui);
        magson_plot(ui);
    }
}

impl Drop for IgrfApp {
    fn drop(&mut self) {
        self.sensor_manager.disconnect();
        self.controller_manager.disconnect();
        self.magson_client.disconnect();
        self.magson_receiver = None;
        self.logger = None;
    }
}

impl eframe::App for IgrfApp {
    fn logic(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.poll_io();
        self.run_pid();
        ctx.request_repaint_after(UI_INTERVAL);
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::Panel::top("top-bar").show(ui, |ui| {
            ui.add_space(4.0);
            self.show_top_bar(ui);
            ui.add_space(4.0);
        });
        egui::Panel::bottom("status").show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.label(format!("Status: {}", self.status));
                if let Some(error) = &self.error {
                    ui.colored_label(Color32::LIGHT_RED, format!("Error: {error}"));
                    if ui.small_button("Dismiss").clicked() {
                        self.error = None;
                    }
                }
            });
        });
        egui::Panel::left("setup")
            .resizable(true)
            .default_size(300.0)
            .size_range(240.0..=460.0)
            .show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    egui::CollapsingHeader::new("Connections")
                        .default_open(true)
                        .show(ui, |ui| self.show_connection_panel(ui));
                    egui::CollapsingHeader::new("LAN static IP")
                        .default_open(false)
                        .show(ui, |ui| self.show_lan_panel(ui));
                    egui::CollapsingHeader::new("Config / logging")
                        .default_open(true)
                        .show(ui, |ui| self.show_config_panel(ui));
                    egui::CollapsingHeader::new("Manual WMM2025 calculator")
                        .default_open(false)
                        .show(ui, |ui| self.show_manual_panel(ui));
                });
            });
        egui::CentralPanel::default().show(ui, |ui| {
            egui::ScrollArea::vertical()
                .auto_shrink([false; 2])
                .show(ui, |ui| {
                    self.show_axis_row(ui);
                    ui.add_space(4.0);
                    self.show_magson_strip(ui);
                    ui.separator();
                    self.show_plots(ui);
                    ui.separator();
                    self.show_cage_row(ui);
                });
        });
    }
}

#[derive(Clone, Copy)]
enum LinkState {
    Off,
    Wait,
    On,
}

impl LinkState {
    fn from_open(open: bool) -> Self {
        if open {
            Self::On
        } else {
            Self::Off
        }
    }

    fn color(self) -> Color32 {
        match self {
            Self::Off => Color32::from_gray(120),
            Self::Wait => Color32::from_rgb(230, 170, 60),
            Self::On => Color32::from_rgb(80, 200, 120),
        }
    }
}

fn fits_columns(ui: &egui::Ui, count: usize) -> bool {
    ui.available_width() >= MIN_COLUMN_WIDTH * count as f32
}

/// An open port that stopped delivering packets leaves the last reading in
/// place, and the PID would keep integrating against that frozen value until the
/// output saturates. Never having received a packet counts as stale too.
fn sensor_is_stale(age: Option<Duration>) -> bool {
    age.is_none_or(|age| age > SENSOR_TIMEOUT)
}

fn status_pill(ui: &mut egui::Ui, label: &str, state: LinkState) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::hover());
    ui.painter()
        .circle_filled(rect.center(), 4.0, state.color());
    ui.colored_label(state.color(), label);
}

fn error_color(error_percent: f64) -> Color32 {
    match error_percent.abs() {
        value if !value.is_finite() => Color32::LIGHT_RED,
        value if value < 1.0 => Color32::from_rgb(80, 200, 120),
        value if value < 5.0 => Color32::from_rgb(230, 170, 60),
        _ => Color32::LIGHT_RED,
    }
}

/// Position of `output` inside `[min, max]`, clamped to 0..=1. Degenerate or
/// non-finite ranges collapse to 0 so the bar never renders garbage.
fn output_fraction(output: f64, min: f64, max: f64) -> f64 {
    let span = max - min;
    if !span.is_finite() || span <= 0.0 || !output.is_finite() {
        return 0.0;
    }
    ((output - min) / span).clamp(0.0, 1.0)
}

fn pid_from_settings(settings: PidSettings) -> PidController {
    let mut pid = PidController::default();
    apply_pid_settings(&mut pid, &settings);
    pid
}

fn apply_pid_settings(pid: &mut PidController, settings: &PidSettings) {
    pid.kp = settings.kp;
    pid.ki = settings.ki;
    pid.kd = settings.kd;
    pid.min_output = settings.min_output;
    pid.max_output = settings.max_output;
}

fn validate_pid_settings(settings: &PidSettings) -> Result<(), &'static str> {
    if ![
        settings.kp,
        settings.ki,
        settings.kd,
        settings.min_output,
        settings.max_output,
        settings.setpoint,
    ]
    .into_iter()
    .all(f64::is_finite)
    {
        return Err("all values must be finite");
    }
    if settings.min_output >= settings.max_output {
        return Err("minimum output must be less than maximum output");
    }
    Ok(())
}

/// Lists `/dev/serial/by-id/*` ahead of the kernel names. A suspend/resume
/// re-enumerates USB, and `ttyUSB0` can come back as `ttyUSB1`; the by-id path
/// carries the adapter's serial number, so a saved config still points at the
/// same physical device.
fn stable_first(ports: Vec<serialport::SerialPortInfo>) -> Vec<String> {
    let mut names: Vec<String> = Vec::new();
    if let Ok(entries) = std::fs::read_dir("/dev/serial/by-id") {
        names.extend(
            entries
                .flatten()
                .map(|entry| entry.path().to_string_lossy().into_owned()),
        );
        names.sort();
    }
    names.extend(ports.into_iter().map(|port| port.port_name));
    names
}

fn port_selector(ui: &mut egui::Ui, id: &str, selected: &mut String, ports: &[String]) {
    let selected_text = if selected.trim().is_empty() {
        "Select port"
    } else {
        selected.as_str()
    };
    egui::ComboBox::from_id_salt(id)
        .selected_text(selected_text)
        .show_ui(ui, |ui| {
            for port in ports {
                ui.selectable_value(selected, port.clone(), port);
            }
        });
}

fn show_plot(
    ui: &mut egui::Ui,
    id: &str,
    title: &str,
    series: &[(&str, &History, Color32)],
    follow: bool,
) {
    ui.label(title);
    Plot::new(id)
        .legend(Legend::default())
        .height(200.0)
        .show(ui, |plot_ui| {
            if follow {
                plot_ui.set_auto_bounds(true);
            }
            for (name, history, color) in series {
                plot_ui
                    .line(Line::new(*name, PlotPoints::new(history.points.clone())).color(*color));
            }
        });
}

fn parse_f64(value: &str, label: &str) -> Result<f64, String> {
    let value = value
        .trim()
        .parse::<f64>()
        .map_err(|_| format!("{label} must be a number"))?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(format!("{label} must be finite"))
    }
}

fn parse_i32(value: &str, label: &str) -> Result<i32, String> {
    value
        .trim()
        .parse::<i32>()
        .map_err(|_| format!("{label} must be an integer"))
}

fn parse_u8(value: &str, label: &str) -> Result<u8, String> {
    value
        .trim()
        .parse::<u8>()
        .map_err(|_| format!("{label} must be an integer from 0 to 255"))
}

fn parse_baud(value: &str) -> Result<u32, String> {
    let baud = value
        .trim()
        .parse::<u32>()
        .map_err(|_| "must be a positive integer".to_owned())?;
    if baud == 0 {
        Err("must be greater than zero".to_owned())
    } else {
        Ok(baud)
    }
}

fn parse_tcp_port(value: &str) -> Result<u16, String> {
    let port = value
        .trim()
        .parse::<u16>()
        .map_err(|_| "must be an integer from 1 to 65535".to_owned())?;
    if port == 0 {
        Err("must be greater than zero".to_owned())
    } else {
        Ok(port)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sensor_is_stale_before_the_first_packet_and_after_the_timeout() {
        assert!(sensor_is_stale(None));
        assert!(!sensor_is_stale(Some(Duration::ZERO)));
        assert!(!sensor_is_stale(Some(SENSOR_TIMEOUT)));
        assert!(sensor_is_stale(Some(
            SENSOR_TIMEOUT + Duration::from_millis(1)
        )));
    }

    #[test]
    fn output_fraction_clamps_and_survives_degenerate_ranges() {
        assert_eq!(output_fraction(0.0, -1.0, 1.0), 0.5);
        assert_eq!(output_fraction(-5.0, -1.0, 1.0), 0.0);
        assert_eq!(output_fraction(5.0, -1.0, 1.0), 1.0);
        assert_eq!(output_fraction(1.0, 2.0, 2.0), 0.0);
        assert_eq!(output_fraction(f64::NAN, -1.0, 1.0), 0.0);
        assert_eq!(output_fraction(0.0, f64::NEG_INFINITY, 1.0), 0.0);
    }

    #[test]
    fn history_is_capped_and_rejects_non_finite_points() {
        let mut history = History::default();
        for index in 0..(HISTORY_LIMIT + 25) {
            history.push(index as f64, index as f64);
        }
        history.push(f64::NAN, 1.0);
        history.push(1.0, f64::INFINITY);
        assert_eq!(history.points.len(), HISTORY_LIMIT);
        assert_eq!(history.points[0], [25.0, 25.0]);
        assert_eq!(history.points[HISTORY_LIMIT - 1], [524.0, 524.0]);
    }

    #[test]
    fn plot_history_clear_removes_magnitude_series() {
        let mut history = PlotHistory::default();
        history.sensor_magnitude_setpoint.push(1.0, 2.0);
        history.sensor_magnitude_measured.push(1.0, 3.0);

        history.clear();

        assert!(history.sensor_magnitude_setpoint.points.is_empty());
        assert!(history.sensor_magnitude_measured.points.is_empty());
    }

    #[test]
    fn input_parsing_rejects_invalid_values_without_panicking() {
        assert!(parse_baud("0").is_err());
        assert!(parse_tcp_port("70000").is_err());
        assert!(parse_f64("NaN", "x").is_err());
        assert_eq!(parse_u8("12", "day"), Ok(12));
    }

    #[test]
    fn pid_settings_reject_non_finite_and_reversed_output_limits() {
        let mut settings = PidSettings::default();
        assert!(validate_pid_settings(&settings).is_ok());
        settings.kp = f64::NAN;
        assert!(validate_pid_settings(&settings).is_err());
        settings = PidSettings::default();
        settings.min_output = settings.max_output;
        assert!(validate_pid_settings(&settings).is_err());
        settings.min_output = 101.0;
        assert!(validate_pid_settings(&settings).is_err());
    }
}
