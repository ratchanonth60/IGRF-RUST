//! SGP4 satellite propagation: given a TLE, computes the TEME position via
//! SGP4 and the Earth-fixed geodetic subpoint from it.

use std::fmt;

use chrono::NaiveDate;

use crate::geomagnetism::{UtcDateTime, Wgs84};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SatelliteError {
    Tle(String),
    Conversion(String),
    Propagation(String),
}

/// Display SatelliteError exception messages
impl fmt::Display for SatelliteError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Tle(message) => write!(f, "TLE error: {message}"),
            Self::Conversion(message) => write!(f, "time conversion error: {message}"),
            Self::Propagation(message) => write!(f, "SGP4 propagation error: {message}"),
        }
    }
}

impl std::error::Error for SatelliteError {}

/// Geodetic subpoint plus the raw TEME position SGP4 propagated to that time. The TEME position is in kilometers, and the geodetic latitude/longitude are in degrees, altitude in kilometers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SatellitePosition {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude_km: f64,
    pub teme_x_km: f64,
    pub teme_y_km: f64,
    pub teme_z_km: f64,
}

/// A parsed TLE ready to propagate, this owns the SGP4 orbital elements and constants
pub struct SatelliteTracker {
    elements: sgp4::Elements,
    constants: sgp4::Constants,
}

impl SatelliteTracker {
    pub fn from_tle(name: Option<&str>, line1: &str, line2: &str) -> Result<Self, SatelliteError> {
        let elements =
            sgp4::Elements::from_tle(name.map(str::to_owned), line1.as_bytes(), line2.as_bytes())
                .map_err(|error| SatelliteError::Tle(error.to_string()))?;
        let constants = sgp4::Constants::from_elements(&elements)
            .map_err(|error| SatelliteError::Tle(error.to_string()))?;
        Ok(Self {
            elements,
            constants,
        })
    }

    pub fn position_at(&self, time: UtcDateTime) -> Result<SatellitePosition, SatelliteError> {
        let naive = to_naive_datetime(time)?;
        let minutes = self
            .elements
            .datetime_to_minutes_since_epoch(&naive)
            .map_err(|error| SatelliteError::Conversion(error.to_string()))?;
        let prediction = self
            .constants
            .propagate(minutes)
            .map_err(|error| SatelliteError::Propagation(error.to_string()))?;

        let gmst = time.gmst_radians();
        let ecef = teme_to_ecef(prediction.position, gmst);
        let (latitude, longitude, altitude_km) = ecef_to_geodetic(ecef, Wgs84::default());

        Ok(SatellitePosition {
            latitude,
            longitude,
            altitude_km,
            teme_x_km: prediction.position[0],
            teme_y_km: prediction.position[1],
            teme_z_km: prediction.position[2],
        })
    }
}

fn to_naive_datetime(time: UtcDateTime) -> Result<chrono::NaiveDateTime, SatelliteError> {
    let date = NaiveDate::from_ymd_opt(time.year, time.month as u32, time.day as u32)
        .ok_or_else(|| SatelliteError::Conversion("invalid calendar date".to_owned()))?;
    date.and_hms_milli_opt(
        time.hour as u32,
        time.minute as u32,
        time.second as u32,
        time.millisecond as u32,
    )
    .ok_or_else(|| SatelliteError::Conversion("invalid time of day".to_owned()))
}

/// Rotate a TEME position into an Earth-fixed frame by Greenwich Mean
/// Sidereal Time. Ignores polar motion and nutation, matching the precision
/// of the lightweight SGP4 tracker this crate replaces.
fn teme_to_ecef(position_km: [f64; 3], gmst_radians: f64) -> [f64; 3] {
    let (sin_g, cos_g) = gmst_radians.sin_cos();
    [
        position_km[0] * cos_g + position_km[1] * sin_g,
        -position_km[0] * sin_g + position_km[1] * cos_g,
        position_km[2],
    ]
}

/// ECEF (km) -> geodetic latitude/longitude (degrees) and height (km), via
/// Bowring's closed-form approximation against the given ellipsoid.
fn ecef_to_geodetic(ecef_km: [f64; 3], spheroid: Wgs84) -> (f64, f64, f64) {
    let a = spheroid.equatorial_axis_m / 1000.0;
    let b = spheroid.polar_axis_m() / 1000.0;
    let e2 = spheroid.eccentricity().powi(2);
    let ep2 = (a * a - b * b) / (b * b);

    let [x, y, z] = ecef_km;
    let longitude = y.atan2(x);
    let p = (x * x + y * y).sqrt();
    let theta = (z * a).atan2(p * b);
    let (sin_theta, cos_theta) = theta.sin_cos();

    let latitude = (z + ep2 * b * sin_theta.powi(3)).atan2(p - e2 * a * cos_theta.powi(3));
    let (sin_lat, cos_lat) = latitude.sin_cos();
    let n = a / (1.0 - e2 * sin_lat * sin_lat).sqrt();
    let height = p / cos_lat - n;

    (latitude.to_degrees(), longitude.to_degrees(), height)
}

/// The satellite presets
#[derive(Debug, Clone, Copy)]
pub struct SatellitePreset {
    pub name: &'static str,
    pub line1: &'static str,
    pub line2: &'static str,
}

pub const PRESETS: &[SatellitePreset] = &[
    SatellitePreset {
        name: "ISS (ZARYA)",
        line1: "1 25544U 98067A   26036.50214262  .00012860  00000+0  24571-3 0  9997",
        line2: "2 25544  51.6316 231.4727 0011155  67.3664 292.8503 15.48414003551342",
    },
    SatellitePreset {
        name: "THEOS",
        line1: "1 33396U 08049A   26040.79524366  .00000115  00000+0  73973-4 0  9998",
        line2: "2 33396  98.5761 102.1159 0001093  84.4399 275.6902 14.20111503899882",
    },
    SatellitePreset {
        name: "STARLINK-31229",
        line1: "1 58986U 24031X   26036.39436483  .00000189  00000+0  18520-4 0  9990",
        line2: "2 58986  53.1592  87.0519 0001084  99.7418 260.3705 15.30187592111562",
    },
    SatellitePreset {
        name: "BEIDOU-3 M1",
        line1: "1 43001U 17069A   26036.57903495 -.00000046  00000+0  00000+0 0  9999",
        line2: "2 43001  56.5755  67.4198 0011656 306.0537  53.8974  1.86231175 56158",
    },
    SatellitePreset {
        name: "GPS BIIF-3  (PRN 24)",
        line1: "1 38833U 12053A   26035.19833436  .00000007  00000+0  00000+0 0  9994",
        line2: "2 38833  53.5640 149.0493 0176647  64.5822 297.2173  2.00565464 96770",
    },
    SatellitePreset {
        name: "COSMOS 2564 (761)",
        line1: "1 54377U 22161A   26036.37904223  .00000053  00000+0  00000+0 0  9991",
        line2: "2 54377  64.7420 197.0834 0008836 205.0145 154.9145  2.13102005 24824",
    },
    SatellitePreset {
        name: "Fengyun (4A)",
        line1: "1 41882U 16077A   26036.93722848 -.00000358  00000+0  00000+0 0  9999",
        line2: "2 41882   1.9889  81.7930 0006361 133.3026  22.0577  1.00276422 33612",
    },
    SatellitePreset {
        name: "Thaicom 6",
        line1: "1 39500U 14002A   26238.94608407 -.00000123  00000-0  00000-0 0  9990",
        line2: "2 39500   0.0382 121.3160 0004254  20.7556 252.2382  1.00272232 46166",
    },
    SatellitePreset {
        name: "METEOSAT-11 (MSG-4)",
        line1: "1 40732U 15034A   26036.91460836  .00000062  00000+0  00000+0 0  9999",
        line2: "2 40732   2.8710  71.8338 0001172 241.7640 161.3542  1.00267859  5905",
    },
];
