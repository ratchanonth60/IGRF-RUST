mod calculation;
mod config;
pub mod geomagnetism;
mod kalman;
mod map_grid;
mod packet;
mod pid;
mod sensor;
mod setpoint;

pub use calculation::{
    CalculationError, CalculationService, ProcessedData, DEFAULT_SPIKE_THRESHOLD_NT,
    REJECTS_BEFORE_FAULT,
};
pub use config::{AppConfig, CalibrationSettings, FilterSettings, PidSettings};
pub use kalman::KalmanFilter;
pub use map_grid::MapGrid;
pub use packet::{
    build_controller_packet, calculate_mod_rtu_crc, clamp_to_firmware, CONTROLLER_HEADER,
    CONTROLLER_PACKET_LEN, FIRMWARE_MAX_OUTPUT,
};
pub use pid::{PidController, NOMINAL_TICK_SECONDS};
pub use sensor::{RawSensorData, SensorService, DEFAULT_COUNT_TO_NT};
pub use setpoint::{field_from_magnitude, ProfilePoint, SetpointProfile, SlewLimiter};
