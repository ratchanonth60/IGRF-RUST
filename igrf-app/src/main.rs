mod cage;
mod netcfg;

use chrono::{Datelike, Timelike};
use eframe::egui::{self, Color32};
use egui_plot::{Legend, Line, Plot, PlotPoint, PlotPoints, Points, Text};
use igrf_core::geomagnetism::{
    Coordinate, GeomagnetismCalculator, GeomagnetismResult, UtcDateTime,
};
use igrf_core::satellite::{SatelliteTracker, SatellitePosition, PRESETS};
use igrf_core::{
    contour_segments, field_from_magnitude, AppConfig, CalculationService, CalibrationSettings,
    ContourSegment, FilterSettings, MapGrid, PidController, PidSettings, ProcessedData,
    SensorService, SetpointProfile, SlewLimiter, FIRMWARE_MAX_OUTPUT, NOMINAL_TICK_SECONDS,
};
use igrf_io::{
    write_controller_packet, ControllerReplyCounter, CsvLogger, MagsonSample, MagsonTcpClient,
    SerialPortManager, SetpointServer, DEFAULT_BIND_ADDRESS,
};
use std::sync::mpsc::{self, Receiver};
use std::thread;
use std::time::{Duration, Instant, SystemTime};

const CONFIG_PATH: &str = "SystemConfig.json";
/// Column-for-column the header the C# build wrote, so existing analysis
/// scripts keep working against logs from either implementation.
/// The C# columns, in the C# order, plus the four this build adds at the end:
/// the commanded setpoint before the slew limiter and the real tick interval.
/// Appending keeps every existing analysis script working.
const LOG_HEADER: &str = "Timestamp,MagX,MagY,MagZ,MagTotal,SetX,SetY,SetZ,SetTotal,ErrX,ErrY,ErrZ,OutX,OutY,OutZ,KpX,KiX,KdX,KpY,KiY,KdY,KpZ,KiZ,KdZ,Mag2X,Mag2Y,Mag2Z,Mag2Total,CmdX,CmdY,CmdZ,TickMs";
const HANDSHAKE: [u8; 6] = [0x2A, 0x30, 0x30, 0x57, 0x45, 0x0D];
const HISTORY_LIMIT: usize = 500;
const CONTOUR_LINE_COLOR: Color32 = Color32::WHITE;
/// Fixed contour step in nT, matching the C# app's hardcoded
/// `ContourLevelStep = 2000` - no UI input for this.
const CONTOUR_LEVEL_STEP_NT: f64 = 2000.0;
const PID_INTERVAL: Duration = Duration::from_millis(100);
const UI_INTERVAL: Duration = Duration::from_millis(50);
/// A running loop is stopped once the newest sensor packet is older than this.
const SENSOR_TIMEOUT: Duration = Duration::from_millis(1000);
/// How long every raw count may sit unchanged before the sensor counts as dead.
///
/// The staleness watchdog only sees missing packets. A sensor that keeps
/// sending the same reading is worse: the loop believes it, the error stays
/// constant, the integrator winds to its clamp and the coils drive hard while
/// the real field walks away unmeasured.
///
/// Well above SENSOR_TIMEOUT because this is a claim about physics rather than
/// about the link. One HMR2300 count is 6.667 nT and its noise floor is larger
/// than that, so all three axes holding identical counts for seconds is a
/// frozen sensor, not a quiet cage.
const SENSOR_FROZEN_TIMEOUT: Duration = Duration::from_secs(5);
/// The C# build reopened the port after this long without a packet, so an
/// unattended run survives a USB hiccup. The watchdog above only stops the
/// coils; it never brings the link back.
const SENSOR_RECONNECT_AFTER: Duration = Duration::from_secs(15);
/// How often a reconnect is retried while the sensor stays silent.
const RECONNECT_INTERVAL: Duration = Duration::from_secs(10);
const AXES: [char; 3] = ['X', 'Y', 'Z'];
/// A commanded field with no fresh command for this long ramps back to zero.
/// An external propagator that dies would otherwise leave the coils holding
/// its last vector for as long as the app runs.
const SETPOINT_SOURCE_TIMEOUT: Duration = Duration::from_secs(10);
/// Soft-iron terms above this far from their transpose are a config typo, not a
/// calibration: an ellipsoid fit is symmetric by construction.
const SOFT_IRON_ASYMMETRY_LIMIT: f64 = 1e-3;
/// Output limits more lopsided than this are reported. A coil pair drives the
/// same both ways, so the expected ratio is 1.0; 1.5 leaves room for a
/// deliberately trimmed axis without passing over a missing digit.
const AUTHORITY_RATIO_LIMIT: f64 = 1.5;
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

#[derive(Default)]
struct PlotHistory {
    sensor_setpoint: [History; 3],
    sensor_measured: [History; 3],
    sensor_magnitude_setpoint: History,
    sensor_magnitude_measured: History,
    magson: [History; 4],
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

/// Where the commanded field comes from. Only one is live at a time, so a
/// profile cannot fight a socket for the coils.
#[derive(Clone, Copy, PartialEq, Eq)]
enum SetpointSource {
    /// Typed into the UI.
    Manual,
    /// Replayed from a CSV of `time_s,bx_nt,by_nt,bz_nt`.
    Profile,
    /// Pushed over UDP by an external propagator.
    Socket,
}

impl SetpointSource {
    fn label(self) -> &'static str {
        match self {
            Self::Manual => "Manual",
            Self::Profile => "CSV profile",
            Self::Socket => "UDP socket",
        }
    }
}

/// Separated UI Tab IGRF Control and IGRF Model
#[derive(Clone, Copy, PartialEq, Eq, Default)]
enum AppTab {
    #[default]
    Control,
    Model,
}

impl AppTab {
    fn label(self) -> &'static str {
        match self {
            Self::Control => "IGRF Control",
            Self::Model => "IGRF Model",
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
    controller_replies: ControllerReplyCounter,
    controller_sent: u64,
    controller_rejected: u64,
    controller_reject_reported: bool,
    magson_client: MagsonTcpClient,
    magson_receiver: Option<Receiver<MagsonSample>>,
    sensor_service: SensorService,
    calculation: CalculationService,
    pid_settings: [PidSettings; 3],
    filter_settings: [FilterSettings; 3],
    calibration: CalibrationSettings,
    pids: [PidController; 3],
    pid_running: [bool; 3],

    /// Rate limit between a commanded field and what the PID actually chases.
    slew: SlewLimiter,
    setpoint_source: SetpointSource,
    setpoint_server: SetpointServer,
    setpoint_receiver: Option<Receiver<[f64; 3]>>,
    last_setpoint_command: Option<Instant>,
    setpoint_port: String,
    setpoint_bind_address: String,
    profile: Option<SetpointProfile>,
    profile_path: String,
    profile_started: Option<Instant>,
    slew_rate: String,
    manual_magnitude: String,
    manual_setpoint_error: Option<String>,

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
    /// When a raw count last differed from the one before it, with the counts
    /// that were current then. Raw rather than filtered: the Kalman output
    /// keeps creeping for a while after its input freezes.
    last_sensor_change: Option<Instant>,
    last_sensor_raw: Option<[f64; 3]>,
    /// Commanded field at the previous sensor packet, so the Kalman filter can
    /// be told how far the ramp moved instead of having to discover it.
    /// `None` until the second packet, where the step is unknown, not zero.
    last_filter_setpoint: Option<[f64; 3]>,
    sensor_intended: bool,
    /// Whether the operator wants the controller link up. Drives the same
    /// auto-reconnect the sensor gets: the firmware has no receive timeout, so
    /// a dropped link leaves the coils energised at the last command until
    /// something reopens the port.
    controller_intended: bool,
    last_reconnect: Option<Instant>,
    last_controller_reconnect: Option<Instant>,
    resume_after_reconnect: bool,
    paused_by_watchdog: [bool; 3],
    resume_pending: bool,

    logger: Option<CsvLogger>,
    manual_lat: String,
    manual_lon: String,
    manual_result: Option<GeomagnetismResult>,
    manual_error: Option<String>,

    /// "IGRF Model" group: geomagnetic grid map.
    map_grid_path: String,
    map_grid: Option<MapGrid>,
    map_grid_error: Option<String>,
    /// Traced by `igrf_core::contour_segments` whenever a grid loads or
    /// "Generate Model" is pressed - the actual marching-squares math lives
    /// in igrf-core, this just holds the result for rendering.
    map_contours: Option<Vec<ContourSegment>>,
    /// Bumped by "Generate Model" to reset the map plot's pan/zoom, by
    /// changing the `Plot`'s egui id so it reinitialises its view.
    map_view_generation: u64,

    /// "Satellite Position" group: TLE tracking and simulated time.
    satellite_preset: Option<usize>,
    satellite_name: String,
    tle_line1: String,
    tle_line2: String,
    satellite_tracker: Option<SatelliteTracker>,
    satellite_tracking: bool,
    sim_time_speed: i32,
    sim_time_offset_s: f64,
    sim_last_tick: Option<Instant>,
    satellite_position: Option<SatellitePosition>,
    satellite_field: Option<GeomagnetismResult>,
    satellite_error: Option<String>,

    active_tab: AppTab,
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
        let initial_setpoint = std::array::from_fn(|axis| pid_settings[axis].setpoint);
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
            controller_replies: ControllerReplyCounter::default(),
            controller_sent: 0,
            controller_rejected: 0,
            controller_reject_reported: false,
            magson_client: MagsonTcpClient::default(),
            magson_receiver: None,
            sensor_service: SensorService::with_calibration(config.calibration.clone()),
            calculation: CalculationService::default(),
            pid_settings,
            filter_settings,
            calibration: config.calibration.clone(),
            pids,
            pid_running: [false; 3],
            slew: SlewLimiter::new(config.setpoint_slew_nt_per_second, initial_setpoint),
            setpoint_source: SetpointSource::Manual,
            setpoint_server: SetpointServer::default(),
            setpoint_receiver: None,
            last_setpoint_command: None,
            setpoint_bind_address: config.setpoint_source_bind_address.clone(),
            setpoint_port: if config.setpoint_source_port > 0 {
                config.setpoint_source_port.to_string()
            } else {
                "5005".to_owned()
            },
            profile: None,
            profile_path: config.setpoint_profile_path.clone(),
            profile_started: None,
            slew_rate: config.setpoint_slew_nt_per_second.to_string(),
            manual_magnitude: "0".to_owned(),
            manual_setpoint_error: None,
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
            last_sensor_change: None,
            last_sensor_raw: None,
            last_filter_setpoint: None,
            sensor_intended: false,
            controller_intended: false,
            last_reconnect: None,
            last_controller_reconnect: None,
            resume_after_reconnect: false,
            paused_by_watchdog: [false; 3],
            resume_pending: false,
            logger: None,
            manual_lat: "13.7563".to_owned(),
            manual_lon: "100.5018".to_owned(),
            manual_result: None,
            manual_error: None,
            map_grid_path: String::new(),
            map_grid: None,
            map_grid_error: None,
            map_contours: None,
            map_view_generation: 0,
            satellite_preset: Some(0),
            satellite_name: PRESETS[0].name.to_owned(),
            tle_line1: PRESETS[0].line1.to_owned(),
            tle_line2: PRESETS[0].line2.to_owned(),
            satellite_tracker: None,
            satellite_tracking: false,
            sim_time_speed: 0,
            sim_time_offset_s: 0.0,
            sim_last_tick: None,
            satellite_position: None,
            satellite_field: None,
            satellite_error: None,
            active_tab: AppTab::default(),
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
                self.last_sensor_change = None;
                self.last_sensor_raw = None;
                self.last_filter_setpoint = None;
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
        self.last_sensor_change = None;
        self.last_sensor_raw = None;
        self.last_filter_setpoint = None;
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
            Ok(()) => {
                self.reset_controller_link_stats();
                self.controller_intended = true;
                self.set_status(format!(
                    "Controller connected: {} @ {baud}",
                    self.controller_port.trim()
                ))
            }
            Err(error) => self.set_error(format!("Controller connect failed: {error}")),
        }
    }

    fn disconnect_controller(&mut self) {
        // Stop driving before the port closes: the controller holds the last
        // output it was given, so a bare disconnect leaves the coils energised
        // and the integrators winding for the next connect.
        self.stop_all();
        self.controller_manager.disconnect();
        self.controller_intended = false;
        self.set_status("Controller disconnected");
    }

    /// Best-effort zero on the coils, shared by every path that stops driving
    /// them. The controller keeps the last packet it received, so anything that
    /// stops the loop has to send zeros first; a failed write means the link is
    /// gone anyway, so the port is closed.
    fn zero_outputs(&mut self) -> std::io::Result<()> {
        self.outputs = [0.0; 3];
        if !self.controller_manager.is_open() {
            return Ok(());
        }
        let result = write_controller_packet(&mut self.controller_manager, 0.0, 0.0, 0.0);
        if result.is_err() {
            self.controller_manager.disconnect();
        }
        result
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
        self.clear_magson();
        self.set_status("Magson disconnected");
    }

    /// Flags a soft-iron matrix that is not symmetric. An ellipsoid fit always
    /// produces one, so an outlier is a mistyped digit rather than a real
    /// calibration - and the error only shows up once the cage is driving a
    /// field, as cross-axis leak that reads like poor uniformity.
    fn calibration_warning(&self) -> Option<String> {
        let asymmetry = self.calibration.asymmetry();
        (asymmetry > SOFT_IRON_ASYMMETRY_LIMIT).then(|| {
            format!(
                "soft-iron is asymmetric by {asymmetry:.4}; at 50000 nT on one axis that leaks \
                 {:.0} nT into another. Check the SoftIron matrix in {CONFIG_PATH}.",
                asymmetry * 50_000.0
            )
        })
    }

    /// Flags an axis that can push the field much harder one way than the
    /// other. The hardware cannot do that - a Helmholtz pair is symmetric - so
    /// the limits are describing the config, not the cage, and the loop will
    /// saturate on one side long before the other.
    fn authority_warning(&self) -> Option<String> {
        let (axis, settings) = self.pid_settings.iter().enumerate().max_by(|left, right| {
            left.1
                .authority_ratio()
                .total_cmp(&right.1.authority_ratio())
        })?;
        let ratio = settings.authority_ratio();
        (ratio > AUTHORITY_RATIO_LIMIT).then(|| {
            format!(
                "PID {} drives {ratio:.1}x harder positive than negative ({:.0} against \
                 {:.0}). A coil pair is symmetric; check MaxOutput and MinOutput in \
                 {CONFIG_PATH}.",
                AXES[axis], settings.max_output, settings.min_output
            )
        })
    }

    fn apply_calibration(&mut self) {
        self.calibration.sanitize();
        self.sensor_service.calibration = self.calibration.clone();
    }

    fn apply_filter_settings(&mut self) {
        for (axis, name) in AXES.into_iter().enumerate() {
            let settings = self.filter_settings[axis].clone();
            if self
                .calculation
                .set_noise(axis, settings.q, settings.r)
                .is_err()
                || self
                    .calculation
                    .set_spike_threshold(axis, settings.spike_nt)
                    .is_err()
            {
                self.filter_settings[axis].sanitize();
                self.set_error(format!(
                    "Filter {name}: Q, R and spike must be finite and above zero; restored defaults"
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

    /// Reopens the controller port by itself, on the same terms as the sensor.
    ///
    /// More urgent than the sensor's: a sensor that is gone stops the loop and
    /// nothing moves, but a controller that is gone leaves six coils holding
    /// whatever they were last told, because the firmware never times out its
    /// receive. Reopening the port is the only thing that can zero them.
    fn maybe_reconnect_controller(&mut self) {
        if !self.controller_intended || self.controller_manager.is_open() {
            return;
        }
        if self
            .last_controller_reconnect
            .is_some_and(|last| last.elapsed() < RECONNECT_INTERVAL)
        {
            return;
        }
        self.last_controller_reconnect = Some(Instant::now());
        let port = self.controller_port.trim().to_owned();
        let Ok(baud) = parse_baud(&self.controller_baud) else {
            return;
        };
        if self.controller_manager.connect(&port, baud).is_err() {
            return;
        }
        self.reset_controller_link_stats();
        // The link is back but the coils are still holding the last command the
        // firmware got. Zero them before anything decides whether to resume.
        if let Err(error) = self.zero_outputs() {
            self.set_error(format!(
                "Controller reopened but will not accept writes: {error}"
            ));
            return;
        }
        self.set_status(format!("Controller reconnected: {port}; outputs zeroed"));
        if self.resume_after_reconnect {
            self.resume_pending = true;
        }
    }

    fn poll_io(&mut self) {
        self.poll_lan_task();
        self.apply_filter_settings();
        self.apply_calibration();
        self.maybe_reconnect_sensor();
        self.maybe_reconnect_controller();
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

        self.poll_controller_replies();

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
            self.clear_magson();
            self.set_error("Magson connection closed");
        }
    }

    fn handle_sensor_packet(&mut self, packet: &[u8]) {
        // The Kalman filter ticks on sensor packets, not on the PID interval,
        // so its interval and its control input are both measured here.
        let ticks = self
            .last_sensor_packet
            .map(|previous| previous.elapsed().as_secs_f64() / NOMINAL_TICK_SECONDS)
            .unwrap_or(1.0);
        self.last_sensor_packet = Some(Instant::now());
        self.last_sensor_packet_wall = Some(SystemTime::now());
        let calibrated = self.sensor_service.process_data(packet);
        self.raw = [
            self.sensor_service.last_raw_x(),
            self.sensor_service.last_raw_y(),
            self.sensor_service.last_raw_z(),
        ];
        // The first packet starts the clock; after that only a real move
        // restarts it, so an unchanging sensor ages out.
        if self.last_sensor_raw != Some(self.raw) {
            self.last_sensor_raw = Some(self.raw);
            self.last_sensor_change = Some(Instant::now());
        }
        self.calibrated = [calibrated.mag_x, calibrated.mag_y, calibrated.mag_z];
        // `pid_settings[..].setpoint` is where the slew limiter has ramped to,
        // so the difference across two packets is exactly how far the field was
        // asked to move in between - a known input, not something the filter
        // should have to infer from the measurement.
        let setpoint: [f64; 3] = std::array::from_fn(|axis| self.pid_settings[axis].setpoint);
        // Only an axis whose loop is closed and whose packets are reaching the
        // coils has actually been commanded to move. The setpoint keeps ramping
        // while the PID is stopped or the controller is unplugged, and
        // predicting a move that nothing is driving is the same lag with the
        // sign flipped.
        let driven = self.controller_manager.is_open();
        let command_delta = match self.last_filter_setpoint {
            Some(previous) => std::array::from_fn(|axis| {
                if driven && self.pid_running[axis] {
                    setpoint[axis] - previous[axis]
                } else {
                    0.0
                }
            }),
            None => [0.0; 3],
        };
        self.last_filter_setpoint = Some(setpoint);
        self.processed =
            self.calculation
                .process_sensor_data(&calibrated, setpoint, command_delta, ticks);
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

    /// Drops the last Magson reading when the link goes away.
    ///
    /// The `Mag2*` CSV columns are written every tick from whatever is in
    /// `self.magson`, so without this a dead link keeps publishing its final
    /// sample for the rest of the run and nothing in the file distinguishes
    /// that from a magnetometer reading a genuinely constant field.
    fn clear_magson(&mut self) {
        self.magson = [0.0; 3];
        self.magson_total = 0.0;
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
    /// Time since any raw count last moved. `None` before the second packet.
    fn sensor_change_age(&self) -> Option<Duration> {
        Some(self.last_sensor_change?.elapsed())
    }

    fn sensor_age(&self) -> Option<Duration> {
        let monotonic = self.last_sensor_packet?.elapsed();
        let wall = self
            .last_sensor_packet_wall
            .and_then(|at| at.elapsed().ok())
            .unwrap_or(Duration::ZERO);
        Some(monotonic.max(wall))
    }

    /// Commands a field vector through the slew limiter. Nothing in the app
    /// writes `pid_settings[..].setpoint` directly any more: every command,
    /// whatever its source, ramps.
    fn command_setpoint(&mut self, field_nt: [f64; 3]) {
        self.slew.command(field_nt);
    }

    /// Advances the ramp and publishes the result as the live setpoint.
    fn advance_setpoint(&mut self, dt: f64) {
        self.slew.rate_nt_per_second = self.config.setpoint_slew_nt_per_second;
        let current = self.slew.step(dt);
        for (settings, value) in self.pid_settings.iter_mut().zip(current) {
            settings.setpoint = value;
        }
    }

    /// Pulls the newest command from whichever source is live. Only the last
    /// datagram of a burst matters: a setpoint is state, not a queue to drain.
    fn poll_setpoint_source(&mut self) {
        match self.setpoint_source {
            SetpointSource::Manual => {}
            SetpointSource::Profile => {
                let Some(started) = self.profile_started else {
                    return;
                };
                let Some(profile) = &self.profile else {
                    return;
                };
                let time = started.elapsed().as_secs_f64();
                if let Some(field) = profile.sample(time) {
                    self.slew.command(field);
                }
                if time > profile.duration_s() {
                    self.profile_started = None;
                    self.set_status("Setpoint profile finished; holding the last row");
                }
            }
            SetpointSource::Socket => {
                let Some(receiver) = &self.setpoint_receiver else {
                    return;
                };
                if let Some(field) = receiver.try_iter().last() {
                    self.slew.command(field);
                    self.last_setpoint_command = Some(Instant::now());
                    return;
                }
                // A propagator that dies mid-run leaves the cage holding its
                // last command indefinitely. The sensor has a watchdog; the
                // commanded field needs one too.
                if self
                    .last_setpoint_command
                    .is_some_and(|at| at.elapsed() > SETPOINT_SOURCE_TIMEOUT)
                {
                    self.last_setpoint_command = None;
                    self.slew.command([0.0; 3]);
                    self.set_error(format!(
                        "No setpoint datagram for {}s; ramping the field to zero",
                        SETPOINT_SOURCE_TIMEOUT.as_secs()
                    ));
                }
            }
        }
    }

    fn start_setpoint_server(&mut self) {
        let port = match parse_tcp_port(&self.setpoint_port) {
            Ok(value) => value,
            Err(error) => {
                self.set_error(format!("Setpoint port: {error}"));
                return;
            }
        };
        let address = match self.setpoint_bind_address.trim() {
            "" => DEFAULT_BIND_ADDRESS.to_owned(),
            chosen => chosen.to_owned(),
        };
        match self.setpoint_server.listen(&address, port) {
            Ok(receiver) => {
                self.setpoint_receiver = Some(receiver);
                self.setpoint_source = SetpointSource::Socket;
                self.last_setpoint_command = Some(Instant::now());
                self.set_status(format!(
                    "Setpoint socket listening on UDP {address}:{port}: send \"bx,by,bz\" in nT"
                ));
                if !is_loopback(&address) {
                    self.set_error(format!(
                        "Setpoint socket is reachable from the network on {address}. Datagrams \
                         are not authenticated: any host that can route here can drive the coils."
                    ));
                }
            }
            Err(error) => self.set_error(format!("Setpoint socket failed: {error}")),
        }
    }

    fn stop_setpoint_server(&mut self) {
        self.setpoint_server.disconnect();
        self.setpoint_receiver = None;
        if self.setpoint_source == SetpointSource::Socket {
            self.setpoint_source = SetpointSource::Manual;
        }
        self.set_status("Setpoint socket stopped; holding the last command");
    }

    fn load_setpoint_profile(&mut self) {
        if self.profile_path.trim().is_empty() {
            self.set_error("Setpoint profile path is empty");
            return;
        }
        match SetpointProfile::load(self.profile_path.trim()) {
            Ok(Ok(profile)) => {
                let rows = profile.len();
                let duration = profile.duration_s();
                self.profile = Some(profile);
                self.profile_started = None;
                self.setpoint_source = SetpointSource::Profile;
                self.set_status(format!(
                    "Loaded {rows} profile rows spanning {duration:.1}s; press Play to run"
                ));
            }
            Ok(Err(problem)) => self.set_error(format!("Setpoint profile: {problem}")),
            Err(error) => self.set_error(format!("Cannot read setpoint profile: {error}")),
        }
    }

    /// Applies a magnitude plus the declination/inclination the WMM panel
    /// reports, so a run can be commanded as "the local field at 1.2x" instead
    /// of three hand-computed components.
    fn apply_manual_magnitude(&mut self) {
        let result = (|| {
            let magnitude = parse_f64(&self.manual_magnitude, "magnitude")?;
            let wmm = self
                .manual_result
                .ok_or_else(|| "run the WMM2025 calculation first".to_owned())?;
            Ok::<_, String>(field_from_magnitude(
                magnitude,
                wmm.declination,
                wmm.inclination,
            ))
        })();
        match result {
            Ok(field) => {
                self.setpoint_source = SetpointSource::Manual;
                self.manual_setpoint_error = None;
                self.command_setpoint(field);
                self.set_status(format!(
                    "Commanded |B| {:.1} nT along the WMM direction; ramping at {:.0} nT/s",
                    self.manual_magnitude.trim().parse::<f64>().unwrap_or(0.0),
                    self.config.setpoint_slew_nt_per_second
                ));
            }
            Err(problem) => self.manual_setpoint_error = Some(problem),
        }
    }

    fn run_pid(&mut self) {
        let now = Instant::now();
        let elapsed = now.duration_since(self.last_pid_tick);
        if elapsed < PID_INTERVAL {
            return;
        }
        self.last_pid_tick = now;
        // The repaint that drives this loop is quantised to the display's
        // refresh, so a nominal 100 ms tick lands anywhere from 100 to 133 ms.
        // Feeding the real interval to the PID keeps Ki and Kd meaning what
        // they meant when they were tuned.
        let dt = elapsed.as_secs_f64();
        self.poll_setpoint_source();
        self.advance_setpoint(dt);
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
            let _ = self.zero_outputs();
            self.pid_settings[axis].sanitize();
            self.set_error(format!(
                "PID {}: {error}; restored safe defaults",
                AXES[axis]
            ));
            self.log_snapshot(elapsed);
            return;
        }
        let fault = loop_fault(
            self.controller_manager.is_open(),
            self.sensor_age(),
            self.sensor_change_age(),
        );
        if self.pid_running.iter().any(|state| *state) {
            if let Some(fault) = fault {
                let reason = match fault {
                    LoopFault::ControllerDown => {
                        "controller link down; the coils hold their last command until it reopens"
                            .to_owned()
                    }
                    LoopFault::SensorFrozen => format!(
                        "sensor readings unchanged for {:.1}s",
                        self.sensor_change_age().unwrap_or_default().as_secs_f64()
                    ),
                    LoopFault::SensorStale => match self.sensor_age() {
                        Some(age) => format!("no sensor data for {:.1}s", age.as_secs_f64()),
                        None => "no sensor data received yet".to_owned(),
                    },
                };
                self.paused_by_watchdog = self.pid_running;
                self.watchdog_pause();
                self.set_error(format!("PID paused: {reason}"));
                // The rows around a fault are the ones worth having; the early
                // return would drop exactly those from the CSV.
                self.log_snapshot(elapsed);
                return;
            }
        }

        // A reconnect only proves a port reopened, not that the link behind it
        // works, so every fault has to be clear before the coils are driven.
        if self.resume_pending && fault.is_none() {
            self.resume_pending = false;
            self.pid_running = self.paused_by_watchdog;
            self.paused_by_watchdog = [false; 3];
            if self.pid_running.iter().any(|state| *state) {
                self.set_status("Links back; PID resumed");
            }
        }

        for axis in 0..3 {
            apply_pid_settings(&mut self.pids[axis], &self.pid_settings[axis]);
            self.outputs[axis] = if self.pid_running[axis] {
                self.pids[axis].calculate_dt(
                    self.pid_settings[axis].setpoint,
                    self.filtered[axis],
                    dt,
                )
            } else {
                0.0
            };
        }

        self.write_outputs();
        self.log_snapshot(elapsed);
    }

    /// Sends the current outputs to the controller. A failed write closes the
    /// port: the link is gone, and pretending otherwise would leave the coils
    /// holding the last packet with nothing watching them.
    fn write_outputs(&mut self) {
        if !self.controller_manager.is_open() {
            return;
        }
        if let Err(error) = write_controller_packet(
            &mut self.controller_manager,
            self.outputs[0],
            self.outputs[1],
            self.outputs[2],
        ) {
            self.set_error(format!("Controller write failed: {error}"));
            self.controller_manager.disconnect();
        } else {
            self.controller_sent = self.controller_sent.saturating_add(1);
        }
    }

    fn reset_controller_link_stats(&mut self) {
        self.controller_replies.reset();
        self.controller_sent = 0;
        self.controller_rejected = 0;
        self.controller_reject_reported = false;
    }

    /// Drains the controller's return path.
    ///
    /// The firmware answers only when it throws a packet away, so anything read
    /// here is a command the coils never acted on. A silent link is a healthy
    /// one; a growing count means the cable, not the control law, is the
    /// problem.
    fn poll_controller_replies(&mut self) {
        if !self.controller_manager.is_open() {
            return;
        }
        match self.controller_manager.read_raw() {
            Ok(bytes) => {
                let rejected = self.controller_replies.feed(&bytes) as u64;
                if rejected == 0 {
                    return;
                }
                self.controller_rejected = self.controller_rejected.saturating_add(rejected);
                // Once per connection: at 10 Hz a bad cable would otherwise
                // overwrite the status line with nothing else.
                if !self.controller_reject_reported {
                    self.controller_reject_reported = true;
                    self.set_error(
                        "Controller rejected a packet (CRC). Watch the reject count on the \
                         controller panel.",
                    );
                }
            }
            Err(error) => {
                self.set_error(format!("Controller read failed: {error}"));
                self.controller_manager.disconnect();
            }
        }
    }

    /// Formats one CSV row. Free-standing so a test can check it against
    /// [`LOG_HEADER`] without a running app.
    ///
    /// The first 28 columns match the C# row exactly: the filtered field as
    /// `Mag*`, the unsigned error from `ProcessedData`, F2 everywhere except
    /// the F3 gains. `Cmd*` and `TickMs` are appended by this build.
    #[allow(clippy::too_many_arguments)]
    fn snapshot_row(
        timestamp: &str,
        filtered: [f64; 3],
        setpoints: [f64; 3],
        errors: [f64; 3],
        gains: &[PidSettings; 3],
        outputs: [f64; 3],
        magson: [f64; 3],
        commanded: [f64; 3],
        tick: Duration,
    ) -> String {
        let magnitude =
            |values: [f64; 3]| values.iter().map(|value| value * value).sum::<f64>().sqrt();
        format!(
            "{},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.1}",
            timestamp,
            filtered[0],
            filtered[1],
            filtered[2],
            magnitude(filtered),
            setpoints[0],
            setpoints[1],
            setpoints[2],
            magnitude(setpoints),
            errors[0],
            errors[1],
            errors[2],
            outputs[0],
            outputs[1],
            outputs[2],
            gains[0].kp,
            gains[0].ki,
            gains[0].kd,
            gains[1].kp,
            gains[1].ki,
            gains[1].kd,
            gains[2].kp,
            gains[2].ki,
            gains[2].kd,
            magson[0],
            magson[1],
            magson[2],
            magnitude(magson),
            commanded[0],
            commanded[1],
            commanded[2],
            tick.as_secs_f64() * 1000.0,
        )
    }

    fn log_snapshot(&mut self, tick: Duration) {
        if self.logger.is_none() {
            return;
        }
        let line = Self::snapshot_row(
            &chrono::Local::now()
                .format("%Y-%m-%d %H:%M:%S%.3f")
                .to_string(),
            self.filtered,
            std::array::from_fn(|axis| self.pid_settings[axis].setpoint),
            [
                self.processed.error_x,
                self.processed.error_y,
                self.processed.error_z,
            ],
            &self.pid_settings,
            self.outputs,
            self.magson,
            // Where the ramp is headed, as opposed to the setpoints, which are
            // where it has reached this tick. Without both, a log cannot tell a
            // slow ramp from a small command.
            self.slew.target(),
            tick,
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
        self.config.calibration = self.calibration.clone();
        self.config.setpoint_profile_path = self.profile_path.clone();
        self.config.setpoint_source_bind_address = self.setpoint_bind_address.clone();
        self.config.setpoint_source_port = parse_tcp_port(&self.setpoint_port)
            .map(i32::from)
            .unwrap_or(0);
        let clamped = self.config.sanitize();
        // Saving writes back whatever sanitize settled on, so the panel has to
        // follow or the UI would keep showing a limit the file no longer holds.
        self.pid_settings = [
            self.config.pid_x.clone(),
            self.config.pid_y.clone(),
            self.config.pid_z.clone(),
        ];
        match self.config.save(CONFIG_PATH) {
            Ok(()) if clamped.iter().any(|was| *was) => self.set_error(format!(
                "Saved {CONFIG_PATH}, but output limits on {} were pulled inside the \
                 firmware ceiling ({:.0}/{:.0}/{:.0})",
                AXES.into_iter()
                    .zip(clamped)
                    .filter(|(_, was)| *was)
                    .map(|(name, _)| name.to_string())
                    .collect::<Vec<_>>()
                    .join("/"),
                FIRMWARE_MAX_OUTPUT[0],
                FIRMWARE_MAX_OUTPUT[1],
                FIRMWARE_MAX_OUTPUT[2],
            )),
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
        self.calibration = config.calibration.clone();
        self.sensor_service.calibration = config.calibration.clone();

        self.profile_path = config.setpoint_profile_path.clone();
        self.slew_rate = config.setpoint_slew_nt_per_second.to_string();
        self.setpoint_bind_address = config.setpoint_source_bind_address.clone();
        if config.setpoint_source_port > 0 {
            self.setpoint_port = config.setpoint_source_port.to_string();
        }
        // A loaded config brings its own setpoints; start the ramp there
        // rather than sweeping from wherever the last one left off.
        self.slew = SlewLimiter::new(
            config.setpoint_slew_nt_per_second,
            std::array::from_fn(|axis| self.pid_settings[axis].setpoint),
        );
        self.config = config;
        match problem {
            Some(message) => self.set_error(message),
            None => match self
                .calibration_warning()
                .or_else(|| self.authority_warning())
            {
                Some(warning) => self.set_error(format!("Loaded {CONFIG_PATH}, but {warning}")),
                None => self.set_status(format!("Loaded {CONFIG_PATH}")),
            },
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
        self.set_status(format!("Reset axis {} PID/filter", AXES[axis]));
    }

    fn master_reset(&mut self) {
        self.stop_all();
        for pid in &mut self.pids {
            pid.reset();
        }
        self.calculation.reset_filters();
        // Leaving a commanded ramp in flight would have the cage climb back
        // toward the old target the moment an axis is started again.
        self.slew.snap([0.0; 3]);
        for settings in &mut self.pid_settings {
            settings.setpoint = 0.0;
        }
        self.profile_started = None;
        self.outputs = [0.0; 3];
        self.filtered = [0.0; 3];
        self.processed = ProcessedData::default();
        self.history.clear();
        if self.error.is_none() {
            self.set_status("Master reset complete");
        }
    }

    /// Calculate Magnetism
    fn calculate_manual_wmm(&mut self) {
        let result = (|| {
            let latitude = parse_f64(&self.manual_lat, "latitude")?;
            let longitude = parse_f64(&self.manual_lon, "longitude")?;
            let coordinate =
                Coordinate::new(latitude, longitude).map_err(|error| error.to_string())?;
            let now = chrono::Utc::now();
            let date = UtcDateTime::new(
                now.year(),
                now.month() as u8,
                now.day() as u8,
                now.hour() as u8,
                now.minute() as u8,
                now.second() as u8,
                now.timestamp_subsec_millis() as u16,
            )
            .map_err(|error| error.to_string())?;
            GeomagnetismCalculator::new()
                .try_calculate_at_altitude(coordinate, 0.0, date)
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

    /// Load Model
    fn browse_map_grid(&mut self) {
        let picked = rfd::FileDialog::new()
            .set_title("Select Geomagnetic Grid Data")
            .add_filter("Text files", &["txt"])
            .add_filter("All files", &["*"])
            .pick_file();
        let Some(path) = picked else {
            return;
        };
        self.map_grid_path = path.display().to_string();
        match MapGrid::load(&path) {
            Ok(grid) => {
                self.map_grid = Some(grid);
                self.map_grid_error = None;
                self.regenerate_contours();
                self.set_status("Geomagnetic grid map loaded");
            }
            Err(error) => {
                self.map_grid = None;
                self.map_contours = None;
                self.map_grid_error = Some(error.to_string());
            }
        }
    }

    /// Generate Model
    fn regenerate_contours(&mut self) {
        let Some(grid) = &self.map_grid else {
            self.map_grid_error = Some("load a grid file first".to_owned());
            return;
        };
        self.map_contours = Some(contour_segments(grid, CONTOUR_LEVEL_STEP_NT));
        self.map_view_generation += 1;
        self.map_grid_error = None;
    }

    /// Generate Results: parse the TLE and start the tracking loop
    fn start_satellite_tracking(&mut self) {
        let name = self.satellite_name.trim();
        let name = if name.is_empty() { None } else { Some(name) };
        match SatelliteTracker::from_tle(name, self.tle_line1.trim(), self.tle_line2.trim()) {
            Ok(tracker) => {
                self.satellite_tracker = Some(tracker);
                self.satellite_tracking = true;
                self.sim_time_offset_s = 0.0;
                self.sim_last_tick = None;
                self.satellite_error = None;
                self.set_status("Satellite tracking started");
            }
            Err(error) => {
                self.satellite_tracker = None;
                self.satellite_tracking = false;
                self.satellite_error = Some(error.to_string());
            }
        }
    }

    fn stop_satellite_tracking(&mut self) {
        self.satellite_tracking = false;
    }

    // Get time
    fn simulated_time(&self) -> Option<UtcDateTime> {
        let offset = chrono::Duration::milliseconds((self.sim_time_offset_s * 1000.0).round() as i64);
        let now = chrono::Utc::now().checked_add_signed(offset)?;
        UtcDateTime::new(
            now.year(),
            now.month() as u8,
            now.day() as u8,
            now.hour() as u8,
            now.minute() as u8,
            now.second() as u8,
            now.timestamp_subsec_millis() as u16,
        )
        .ok()
    }

    fn tick_satellite_tracking(&mut self) {
        if !self.satellite_tracking {
            return;
        }
        let Some(tracker) = &self.satellite_tracker else {
            return;
        };
        let now = Instant::now();
        if let Some(last) = self.sim_last_tick {
            if now.duration_since(last) < Duration::from_secs(1) {
                return;
            }
        }
        self.sim_last_tick = Some(now);
        self.sim_time_offset_s += self.sim_time_speed as f64;

        let Some(time) = self.simulated_time() else {
            self.satellite_error = Some("simulated time is out of range".to_owned());
            return;
        };
        match tracker.position_at(time) {
            Ok(position) => {
                self.satellite_error = None;
                self.satellite_field = Coordinate::new(position.latitude, position.longitude)
                    .ok()
                    .and_then(|coordinate| {
                        GeomagnetismCalculator::new()
                            .try_calculate_at_altitude(coordinate, position.altitude_km, time)
                            .ok()
                            .flatten()
                    });
                self.satellite_position = Some(position);
            }
            Err(error) => {
                self.satellite_error = Some(error.to_string());
                self.satellite_position = None;
                self.satellite_field = None;
            }
        }
    }

    fn stop_all(&mut self) {
        self.pid_running = [false; 3];
        // A deliberate stop clears the loop: whoever presses this wants the
        // cage inert, not parked ready to resume.
        for pid in &mut self.pids {
            pid.reset();
        }
        self.paused_by_watchdog = [false; 3];
        self.resume_pending = false;
        if let Err(error) = self.zero_outputs() {
            self.set_error(format!("STOP ALL: controller write failed: {error}"));
            return;
        }
        self.set_status("STOP ALL: every axis paused, outputs zeroed");
    }

    /// Watchdog pause: coils to zero, but the loop keeps its integral.
    ///
    /// Nulling the ambient field puts nearly the whole output in the integral
    /// term. Clearing it on a one-second sensor dropout means the error jumps
    /// to the full ambient field on resume and the rebuilt integral overshoots
    /// into the output limit - a full-scale transient through the 48 V drivers
    /// every time the USB link hiccups. Holding it makes the resume bumpless.
    fn watchdog_pause(&mut self) {
        self.pid_running = [false; 3];
        for pid in &mut self.pids {
            pid.hold();
        }
        if let Err(error) = self.zero_outputs() {
            self.set_error(format!("Watchdog pause: controller write failed: {error}"));
        }
    }

    /// Top-level page navigation
    fn show_tab_strip(&mut self, ui: &mut egui::Ui) {
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            for tab in [AppTab::Control, AppTab::Model] {
                let selected = self.active_tab == tab;
                let button = egui::Button::new(
                    egui::RichText::new(tab.label())
                        .strong()
                        .size(15.0),
                )
                .selected(selected)
                .min_size(egui::vec2(120.0, 28.0));
                if ui.add(button).clicked() {
                    self.active_tab = tab;
                }
            }
        });
        ui.add_space(4.0);
        ui.separator();
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
        // Kept visible after a disconnect: a link bad enough to drop packets is
        // a link bad enough to drop entirely, and the count is the evidence.
        if self.controller_sent > 0 {
            let rate = 100.0 * self.controller_rejected as f64 / self.controller_sent as f64;
            let label = format!(
                "Rejected: {} / {} sent ({rate:.2}%)",
                self.controller_rejected, self.controller_sent
            );
            if self.controller_rejected == 0 {
                ui.label(label)
            } else {
                ui.colored_label(egui::Color32::from_rgb(220, 120, 60), label)
            }
            .on_hover_text(
                "Packets the firmware answered with \"Error\\r\" because the CRC did not \
                 match. Those commands never reached the coils. Anything above zero is a \
                 cabling or baud problem, not a tuning one.",
            );
        }

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
        // The frame layout is not confirmed (see the README), so a climbing
        // count is the difference between "this build ignores some types" and
        // "the stream is not being understood at all".
        let dropped = self.magson_client.dropped_frames();
        if dropped > 0 {
            ui.colored_label(
                egui::Color32::from_rgb(220, 120, 60),
                format!("Undecoded frames: {dropped}"),
            )
            .on_hover_text(
                "Frames read but not decoded: types this build ignores, plus                  anything discarded while resynchronising after a lost byte.",
            );
        }
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

    /// Setpoint command panel: where the field comes from and how fast it may
    /// change. Everything here is nanotesla.
    fn show_setpoint_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Source");
            for source in [
                SetpointSource::Manual,
                SetpointSource::Profile,
                SetpointSource::Socket,
            ] {
                if ui
                    .selectable_label(self.setpoint_source == source, source.label())
                    .clicked()
                {
                    self.setpoint_source = source;
                }
            }
        });

        let target = self.slew.target();
        let current = self.slew.current();
        let magnitude = |value: [f64; 3]| value.iter().map(|v| v * v).sum::<f64>().sqrt();
        ui.label(
            egui::RichText::new(format!(
                "now {:.1} nT -> target {:.1} nT",
                magnitude(current),
                magnitude(target)
            ))
            .monospace(),
        );
        if !self.slew.is_settled() {
            ui.label(egui::RichText::new("ramping").small().weak());
        }

        ui.horizontal(|ui| {
            ui.label("Slew nT/s");
            ui.add(egui::TextEdit::singleline(&mut self.slew_rate).desired_width(70.0));
            if ui.button("Set").clicked() {
                match parse_f64(&self.slew_rate, "slew rate") {
                    Ok(rate) if rate > 0.0 => {
                        self.config.setpoint_slew_nt_per_second = rate;
                        self.set_status(format!("Setpoint ramps at {rate:.0} nT/s"));
                    }
                    Ok(_) => self.set_error("Slew rate must be above zero"),
                    Err(error) => self.set_error(format!("Slew rate: {error}")),
                }
            }
        });

        match self.setpoint_source {
            SetpointSource::Manual => {
                ui.separator();
                ui.label("Command |B| along the WMM direction");
                ui.horizontal(|ui| {
                    ui.label("|B| nT");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.manual_magnitude).desired_width(80.0),
                    );
                    if ui.button("Command").clicked() {
                        self.apply_manual_magnitude();
                    }
                });
                match self.manual_result {
                    Some(wmm) => ui.label(
                        egui::RichText::new(format!(
                            "direction D {:.2} deg, I {:.2} deg",
                            wmm.declination, wmm.inclination
                        ))
                        .small()
                        .weak(),
                    ),
                    None => ui.label(
                        egui::RichText::new("run the WMM2025 calculation for a direction")
                            .small()
                            .weak(),
                    ),
                };
                if let Some(error) = &self.manual_setpoint_error {
                    ui.colored_label(Color32::LIGHT_RED, error);
                }
                ui.label(
                    egui::RichText::new("per-axis setpoints stay editable on each axis card")
                        .small()
                        .weak(),
                );
            }
            SetpointSource::Profile => {
                ui.separator();
                ui.label("CSV: time_s,bx_nt,by_nt,bz_nt");
                ui.horizontal(|ui| {
                    ui.add(egui::TextEdit::singleline(&mut self.profile_path).desired_width(150.0));
                    if ui.button("Load").clicked() {
                        self.load_setpoint_profile();
                    }
                });
                match &self.profile {
                    Some(profile) => {
                        let rows = profile.len();
                        let duration = profile.duration_s();
                        ui.label(
                            egui::RichText::new(format!("{rows} rows, {duration:.1}s"))
                                .small()
                                .weak(),
                        );
                        ui.horizontal(|ui| {
                            let running = self.profile_started.is_some();
                            if ui.button(if running { "Stop" } else { "Play" }).clicked() {
                                self.profile_started = (!running).then(Instant::now);
                            }
                            if let Some(started) = self.profile_started {
                                ui.label(
                                    egui::RichText::new(format!(
                                        "t = {:.1}s",
                                        started.elapsed().as_secs_f64()
                                    ))
                                    .monospace(),
                                );
                            }
                        });
                    }
                    None => {
                        ui.label(egui::RichText::new("no profile loaded").small().weak());
                    }
                }
            }
            SetpointSource::Socket => {
                ui.separator();
                ui.horizontal(|ui| {
                    ui.label("UDP port");
                    ui.add(egui::TextEdit::singleline(&mut self.setpoint_port).desired_width(64.0));
                    ui.label("Bind");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.setpoint_bind_address)
                            .desired_width(96.0),
                    )
                    .on_hover_text(
                        "Interface the listener accepts datagrams on. 127.0.0.1 keeps it on \
                         this machine. Anything else lets any host that can route here drive \
                         the coils, with no authentication.",
                    );
                    if self.setpoint_server.is_listening() {
                        if ui.button("Stop").clicked() {
                            self.stop_setpoint_server();
                        }
                    } else if ui.button("Listen").clicked() {
                        self.start_setpoint_server();
                    }
                });
                ui.label(
                    egui::RichText::new(if self.setpoint_server.is_listening() {
                        "listening - send \"bx,by,bz\" in nT, newest datagram wins"
                    } else {
                        "stopped"
                    })
                    .small()
                    .weak(),
                );
            }
        }
    }

    /// Sensor calibration. These belong to the physical unit in the cage, so
    /// re-fitting the ellipsoid must not need a rebuild.
    fn show_calibration_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("nT/count");
            ui.add(
                egui::DragValue::new(&mut self.calibration.count_to_nt)
                    .speed(0.001)
                    .range(1e-6..=1e6),
            );
        });
        ui.label(egui::RichText::new("Hard iron nT").small().weak());
        egui::Grid::new("hard-iron-grid")
            .num_columns(3)
            .show(ui, |ui| {
                for value in &mut self.calibration.hard_iron {
                    ui.add(egui::DragValue::new(value).speed(1.0));
                }
                ui.end_row();
            });
        ui.label(egui::RichText::new("Soft iron").small().weak());
        egui::Grid::new("soft-iron-grid")
            .num_columns(3)
            .show(ui, |ui| {
                for row in &mut self.calibration.soft_iron {
                    for value in row {
                        ui.add(egui::DragValue::new(value).speed(0.0001).max_decimals(6));
                    }
                    ui.end_row();
                }
            });
        for warning in [self.calibration_warning(), self.authority_warning()]
            .into_iter()
            .flatten()
        {
            ui.colored_label(Color32::LIGHT_RED, warning);
        }
        ui.label(
            egui::RichText::new("Save SystemConfig.json to keep these")
                .small()
                .weak(),
        );
    }

    fn show_manual_panel(&mut self, ui: &mut egui::Ui) {
        egui::Grid::new("manual-wmm-grid")
            .num_columns(2)
            .show(ui, |ui| {
                for (label, value) in [
                    ("Latitude", &mut self.manual_lat),
                    ("Longitude", &mut self.manual_lon),
                ] {
                    ui.label(label);
                    ui.text_edit_singleline(value);
                    ui.end_row();
                }
            });
        if ui.button("Calculate Magnetism WMM2025").clicked() {
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
                        ("Declination", result.declination),
                        ("Inclination", result.inclination),
                        ("Intensity", result.total_intensity),
                        ("X", result.x),
                        ("Y", result.y),
                        ("Z", result.z),
                    ] {
                        ui.label(label);
                        ui.label(format!("{value:.4}"));
                        ui.end_row();
                    }
                });
        }
    }

    /// Tab left side: IGRF Model group loaded from a text file, and a "Generate Model"
    fn show_map_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            if ui.button("Load Model").clicked() {
                self.browse_map_grid();
            }
        });
        if !self.map_grid_path.is_empty() {
            ui.label(format!("File: {}", self.map_grid_path));
        }
        if let Some(error) = &self.map_grid_error {
            ui.colored_label(Color32::LIGHT_RED, error);
        }
    }

    /// Time group: live UTC clock
    fn show_time_panel(&self, ui: &mut egui::Ui) {
        ui.label(format!(
            "UTC: {}",
            chrono::Utc::now().format("%Y-%m-%d %H:%M:%S")
        ));
    }

    /// Satellite Position group: SGP4 propagation and the
    /// TEME->geodetic conversion happen in `igrf_core::satellite`; this
    /// owns the TLE text fields, the tracking toggle, and the simulated
    /// clock's speed/offset.
    fn show_satellite_panel(&mut self, ui: &mut egui::Ui) {
        egui::ComboBox::from_id_salt("satellite-preset")
            .selected_text(match self.satellite_preset {
                Some(index) => PRESETS[index].name,
                None => "-- Manual --",
            })
            .show_ui(ui, |ui| {
                if ui
                    .selectable_label(self.satellite_preset.is_none(), "-- Manual --")
                    .clicked()
                {
                    self.satellite_preset = None;
                }
                for (index, preset) in PRESETS.iter().enumerate() {
                    if ui
                        .selectable_label(self.satellite_preset == Some(index), preset.name)
                        .clicked()
                    {
                        self.satellite_preset = Some(index);
                        self.satellite_name = preset.name.to_owned();
                        self.tle_line1 = preset.line1.to_owned();
                        self.tle_line2 = preset.line2.to_owned();
                        // Switching satellites mid-track should jump straight
                        // to the new one's live results, not leave the old
                        // tracker running until an explicit Stop+Generate.
                        if self.satellite_tracking {
                            self.start_satellite_tracking();
                        }
                    }
                }
            });

        ui.horizontal(|ui| {
            ui.label("Name");
            ui.text_edit_singleline(&mut self.satellite_name);
        });
        ui.horizontal(|ui| {
            ui.label("Line 1");
            ui.add(
                egui::TextEdit::singleline(&mut self.tle_line1)
                    .desired_width(300.0)
                    .font(egui::TextStyle::Monospace),
            );
        });
        ui.horizontal(|ui| {
            ui.label("Line 2");
            ui.add(
                egui::TextEdit::singleline(&mut self.tle_line2)
                    .desired_width(300.0)
                    .font(egui::TextStyle::Monospace),
            );
        });

        ui.horizontal(|ui| {
            if self.satellite_tracking {
                if ui.button("Stop Tracking").clicked() {
                    self.stop_satellite_tracking();
                }
            } else if ui.button("Generate Results").clicked() {
                self.start_satellite_tracking();
            }
        });
        if let Some(error) = &self.satellite_error {
            ui.colored_label(Color32::LIGHT_RED, error);
        }

        // Simulated time speed slider, with a reset button and a display of the current simulated time.
        ui.separator();
        ui.label("Simulated time speed (seconds per real second)");
        ui.horizontal(|ui| {
            ui.add(egui::Slider::new(&mut self.sim_time_speed, -600..=600));
            if ui.button("Reset").clicked() {
                self.sim_time_speed = 0;
                self.sim_time_offset_s = 0.0;
                self.sim_last_tick = None;
            }
        });
        if let Some(time) = self.simulated_time() {
            ui.label(format!(
                "Simulated time: {:04}-{:02}-{:02} {:02}:{:02}:{:02}",
                time.year, time.month, time.day, time.hour, time.minute, time.second
            ));
        }

        ui.add_space(8.0);
        ui.separator();
        ui.label(egui::RichText::new("Satellite Result").strong());
        match self.satellite_position {
            Some(position) => {
                let field = self.satellite_field;
                egui::Grid::new("satellite-position-result-grid")
                    .num_columns(2)
                    .striped(true)
                    .show(ui, |ui| {
                        ui.label("Latitude");
                        ui.label(format!("{:.3}", position.latitude));
                        ui.end_row();
                        ui.label("Longitude");
                        ui.label(format!("{:.3}", position.longitude));
                        ui.end_row();
                        ui.label("Altitude km");
                        ui.label(format!("{:.3}", position.altitude_km));
                        ui.end_row();
                        ui.label("Total Intensity");
                        match field.map(|f| f.total_intensity) {
                            Some(value) => ui.label(format!("{value:.3}")),
                            None => ui.label("--"),
                        };
                        ui.end_row();
                        // The satellite's TEME position (km) X/Y/Z
                        for (label, value) in [
                            ("X", position.teme_x_km),
                            ("Y", position.teme_y_km),
                            ("Z", position.teme_z_km),
                        ] {
                            ui.label(label);
                            ui.label(format!("{value:.3}"));
                            ui.end_row();
                        }
                    });
            }
            None => {
                ui.label("No satellite result yet. Press \"Generate Results\" to start tracking.");
            }
        }
    }

    /// Right side of the IGRF Model tab: Magnetism Result
    fn show_model_result_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Geomagnetic field map");
        match &self.map_contours {
            Some(contours) if !contours.is_empty() => {
                let mut labels: Vec<(f64, [f64; 2])> = Vec::new();
                for segment in contours {
                    let already_labelled = labels
                        .iter()
                        .any(|(level, _)| (level - segment.level).abs() < f64::EPSILON);
                    if !already_labelled {
                        labels.push((
                            segment.level,
                            [
                                (segment.start[0] + segment.end[0]) / 2.0,
                                (segment.start[1] + segment.end[1]) / 2.0,
                            ],
                        ));
                    }
                }
                let satellite_position = self.satellite_position;

                Plot::new(("model-map", self.map_view_generation))
                    .x_axis_label("Longitude")
                    .y_axis_label("Latitude")
                    .height(720.0)
                    .show(ui, |plot_ui| {
                        for segment in contours {
                            plot_ui.line(
                                Line::new(
                                    "Contour lines",
                                    PlotPoints::new(vec![segment.start, segment.end]),
                                )
                                .color(CONTOUR_LINE_COLOR),
                            );
                        }
                        for (level, point) in &labels {
                            plot_ui.text(
                                Text::new(
                                    format!("contour-label-{level}"),
                                    PlotPoint::new(point[0], point[1]),
                                    format!("{level:.0}"),
                                )
                                .color(CONTOUR_LINE_COLOR),
                            );
                        }
                        if let Some(position) = satellite_position {
                            plot_ui.points(
                                Points::new(
                                    "Satellite",
                                    vec![[position.longitude, position.latitude]],
                                )
                                .radius(5.0)
                                .color(Color32::RED),
                            );
                        }
                    });
            }
            Some(_) => {
                ui.label("No contour lines at this level step for the loaded grid.");
            }
            None => {
                ui.label("No geomagnetic field map generated yet.");
            }
        }
    }

    /// One X/Y/Z card: live readout on top, PID gains below. Always visible so
    /// nothing that can stop an axis hides behind navigation.
    fn axis_column(&mut self, ui: &mut egui::Ui, axis: usize) {
        let label = AXES[axis];
        let target = self.slew.target();
        let mut command = None;
        let mut pause = false;
        ui.group(|ui| {
            ui.set_min_width(ui.available_width().max(0.0));
            let running = self.pid_running[axis];
            ui.horizontal(|ui| {
                status_pill(ui, &format!("Axis {label}"), LinkState::from_open(running));
                if ui.button(if running { "Pause" } else { "Start" }).clicked() {
                    self.pid_running[axis] = !running;
                    if !self.pid_running[axis] {
                        pause = true;
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
            ui.label(egui::RichText::new("filtered nT").small().weak());
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
                    ] {
                        ui.label(name);
                        ui.add(egui::DragValue::new(value).speed(0.1));
                        ui.end_row();
                    }
                    // Bounded by what the firmware acts on rather than by
                    // taste: past its ceiling the raw value goes into a 16-bit
                    // CCR and truncates, so a bigger number is a smaller field.
                    let ceiling = FIRMWARE_MAX_OUTPUT[axis];
                    ui.label("Min out");
                    ui.add(
                        egui::DragValue::new(&mut self.pid_settings[axis].min_output)
                            .speed(1.0)
                            .range(-ceiling..=0.0),
                    );
                    ui.end_row();
                    ui.label("Max out");
                    ui.add(
                        egui::DragValue::new(&mut self.pid_settings[axis].max_output)
                            .speed(1.0)
                            .range(0.0..=ceiling),
                    );
                    ui.end_row();
                    ui.label(
                        egui::RichText::new(format!("firmware ceiling {ceiling:.0}"))
                            .small()
                            .weak(),
                    );
                    ui.end_row();
                    // The live setpoint is owned by the ramp, so editing it
                    // here commands a new target rather than writing the value
                    // the PID reads this tick - otherwise the next tick would
                    // overwrite whatever was typed.
                    ui.label("Setpoint nT");
                    let mut commanded = target[axis];
                    if ui
                        .add(egui::DragValue::new(&mut commanded).speed(1.0))
                        .changed()
                    {
                        let mut field = target;
                        field[axis] = commanded;
                        command = Some(field);
                    }
                    ui.end_row();
                });
            ui.label(egui::RichText::new("Kalman filter").small().weak());
            egui::Grid::new(format!("filter-grid-{axis}"))
                .num_columns(2)
                .striped(true)
                .show(ui, |ui| {
                    for (name, value, speed) in [
                        ("Q process", &mut self.filter_settings[axis].q, 0.05),
                        ("R measure", &mut self.filter_settings[axis].r, 1.0),
                        ("Spike nT", &mut self.filter_settings[axis].spike_nt, 50.0),
                    ] {
                        ui.label(name);
                        ui.add(egui::DragValue::new(value).speed(speed).range(1e-6..=1e9));
                        ui.end_row();
                    }
                });
        });
        if pause {
            // Pausing one axis stops that axis' PID, but the controller holds
            // whatever it was last sent for all three. Push a packet now with
            // this axis at zero instead of waiting for the next tick.
            self.pids[axis].hold();
            self.outputs[axis] = 0.0;
            self.write_outputs();
        }
        if let Some(field) = command {
            self.setpoint_source = SetpointSource::Manual;
            self.command_setpoint(field);
        }
    }

    fn show_axis_row(&mut self, ui: &mut egui::Ui) {
        if fits_columns(ui, 3) {
            ui.columns(3, |columns| {
                for (axis, column) in columns.iter_mut().enumerate() {
                    self.axis_column(column, axis);
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
                &format!("{} nT: setpoint vs measured", AXES[axis]),
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
                for (axis, column) in columns.iter_mut().enumerate() {
                    axis_plot(column, axis);
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
                "|B| nT: setpoint vs measured",
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
                "Magson X/Y/Z/total (nT)",
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
        let _ = self.zero_outputs();
        self.setpoint_server.disconnect();
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
        self.tick_satellite_tracking();
        ctx.request_repaint_after(UI_INTERVAL);
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::Panel::top("tab-strip").show(ui, |ui| {
            self.show_tab_strip(ui);
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
        match self.active_tab {
            AppTab::Control => {
                egui::Panel::top("top-bar").show(ui, |ui| {
                    ui.add_space(4.0);
                    self.show_top_bar(ui);
                    ui.add_space(4.0);
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
                            egui::CollapsingHeader::new("Setpoint command")
                                .default_open(true)
                                .show(ui, |ui| self.show_setpoint_panel(ui));
                            egui::CollapsingHeader::new("Sensor calibration")
                                .default_open(false)
                                .show(ui, |ui| self.show_calibration_panel(ui));
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
            AppTab::Model => {
                egui::Panel::left("model-setup")
                    .resizable(true)
                    .default_size(300.0)
                    .size_range(240.0..=460.0)
                    .show(ui, |ui| {
                        egui::ScrollArea::vertical().show(ui, |ui| {
                            egui::CollapsingHeader::new("IGRF Model")
                                .default_open(true)
                                .show(ui, |ui| self.show_map_panel(ui));
                            egui::CollapsingHeader::new("Time")
                                .default_open(true)
                                .show(ui, |ui| self.show_time_panel(ui));
                            egui::CollapsingHeader::new("Satellite Position")
                                .default_open(true)
                                .show(ui, |ui| self.show_satellite_panel(ui));
                            // egui::CollapsingHeader::new("Manual WMM2025 calculator")
                            //     .default_open(true)
                            //     .show(ui, |ui| self.show_manual_panel(ui));
                        });
                    });
                egui::CentralPanel::default().show(ui, |ui| {
                    egui::ScrollArea::vertical()
                        .auto_shrink([false; 2])
                        .show(ui, |ui| self.show_model_result_panel(ui));
                });
            }
        }
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
/// Whether a bind address keeps the setpoint listener on this machine.
///
/// Resolved rather than string-matched: `localhost` is loopback, `0.0.0.0` is
/// not, and neither is obvious from the text. An address that will not resolve
/// is reported as exposed, because the warning has to be the safe default.
fn is_loopback(address: &str) -> bool {
    use std::net::ToSocketAddrs;
    (address, 0_u16)
        .to_socket_addrs()
        .map(|mut resolved| resolved.all(|socket| socket.ip().is_loopback()))
        .unwrap_or(false)
}

/// Unlike [`sensor_is_stale`], `None` is not a fault: it only means no second
/// packet has arrived yet, which staleness already covers.
fn sensor_is_frozen(age: Option<Duration>) -> bool {
    age.is_some_and(|age| age > SENSOR_FROZEN_TIMEOUT)
}

/// Why the loop must not be driving the coils, if it must not.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LoopFault {
    ControllerDown,
    SensorFrozen,
    SensorStale,
}

/// The one gate every path uses, so pausing and resuming cannot disagree about
/// what counts as healthy.
///
/// Ordered by what the operator has to deal with first. A controller that is
/// gone makes the sensor's state irrelevant - and is the worse fault, because
/// the firmware has no receive timeout and holds its last command until the
/// port reopens. A frozen sensor outranks a silent one only because it is the
/// more specific diagnosis of the two.
fn loop_fault(
    controller_open: bool,
    sensor_age: Option<Duration>,
    sensor_change_age: Option<Duration>,
) -> Option<LoopFault> {
    if !controller_open {
        return Some(LoopFault::ControllerDown);
    }
    if sensor_is_frozen(sensor_change_age) {
        return Some(LoopFault::SensorFrozen);
    }
    if sensor_is_stale(sensor_age) {
        return Some(LoopFault::SensorStale);
    }
    None
}

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