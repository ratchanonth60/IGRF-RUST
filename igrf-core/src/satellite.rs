//! SGP4 satellite propagation: given a TLE, computes the TEME position via
//! SGP4 and the Earth-fixed geodetic subpoint from it.

use std::fmt;

use chrono::{Datelike, NaiveDate, Timelike};

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
    /// Earth-fixed (ECEF) position, kilometers. Same frame `elevation_deg`
    /// expects for a ground station.
    pub ecef_km: [f64; 3],
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
            ecef_km: ecef,
        })
    }

    /// Orbital period implied by the TLE's mean motion (revolutions/day).
    pub fn orbital_period_minutes(&self) -> f64 {
        1440.0 / self.elements.mean_motion
    }

    /// Samples positions across one full orbital period centered on `center`
    /// (`center - period/2` .. `center + period/2`), for drawing a ground
    /// track. `samples` is clamped to at least 2.
    pub fn ground_track(
        &self,
        center: UtcDateTime,
        samples: usize,
    ) -> Result<Vec<SatellitePosition>, SatelliteError> {
        let period_minutes = self.orbital_period_minutes();
        let naive_center = to_naive_datetime(center)?;
        let samples = samples.max(2);
        (0..samples)
            .map(|index| {
                let frac = index as f64 / (samples - 1) as f64 - 0.5;
                let offset_ms = (frac * period_minutes * 60_000.0).round() as i64;
                let time = naive_center + chrono::Duration::milliseconds(offset_ms);
                self.position_at(naive_to_utc_datetime(time)?)
            })
            .collect()
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

fn naive_to_utc_datetime(time: chrono::NaiveDateTime) -> Result<UtcDateTime, SatelliteError> {
    UtcDateTime::new(
        time.year(),
        time.month() as u8,
        time.day() as u8,
        time.hour() as u8,
        time.minute() as u8,
        time.second() as u8,
        (time.and_utc().timestamp_subsec_millis()) as u16,
    )
    .map_err(|error| SatelliteError::Conversion(error.to_string()))
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

/// Geodetic latitude/longitude (degrees) and altitude (km) -> ECEF (km).
/// Inverse of [`ecef_to_geodetic`]; used to place the ground station in the
/// same frame as a satellite's `ecef_km` for elevation.
pub fn geodetic_to_ecef(latitude_deg: f64, longitude_deg: f64, altitude_km: f64) -> [f64; 3] {
    let spheroid = Wgs84::default();
    let a = spheroid.equatorial_axis_m / 1000.0;
    let e2 = spheroid.eccentricity().powi(2);
    let lat = latitude_deg.to_radians();
    let lon = longitude_deg.to_radians();
    let (sin_lat, cos_lat) = lat.sin_cos();
    let (sin_lon, cos_lon) = lon.sin_cos();
    let n = a / (1.0 - e2 * sin_lat * sin_lat).sqrt();
    [
        (n + altitude_km) * cos_lat * cos_lon,
        (n + altitude_km) * cos_lat * sin_lon,
        (n * (1.0 - e2) + altitude_km) * sin_lat,
    ]
}

/// Elevation angle (degrees) of `target_ecef_km` as seen from a ground
/// station at sea level at `station_lat_deg`/`station_lon_deg`, via the
/// local East-North-Up frame. 90 degrees is straight overhead, 0 is the
/// horizon, negative is below it.
pub fn elevation_deg(station_lat_deg: f64, station_lon_deg: f64, target_ecef_km: [f64; 3]) -> f64 {
    let station_ecef = geodetic_to_ecef(station_lat_deg, station_lon_deg, 0.0);
    let lat = station_lat_deg.to_radians();
    let lon = station_lon_deg.to_radians();
    let (sin_lat, cos_lat) = lat.sin_cos();
    let (sin_lon, cos_lon) = lon.sin_cos();

    let dx = target_ecef_km[0] - station_ecef[0];
    let dy = target_ecef_km[1] - station_ecef[1];
    let dz = target_ecef_km[2] - station_ecef[2];

    let north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
    let east = -sin_lon * dx + cos_lon * dy;
    let up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;

    let range = (east * east + north * north + up * up).sqrt();
    (up / range).asin().to_degrees()
}

/// Splits a ground track into separate polylines wherever consecutive
/// samples jump more than 180 degrees in longitude, so a plot draws the
/// antimeridian crossing as a break instead of a line straight across the
/// map. Returns `[longitude, latitude]` pairs per point, ready for
/// `egui_plot::PlotPoints`.
pub fn split_dateline_segments(track: &[SatellitePosition]) -> Vec<Vec<[f64; 2]>> {
    let mut segments = Vec::new();
    let mut current: Vec<[f64; 2]> = Vec::new();
    let mut previous_longitude: Option<f64> = None;

    for position in track {
        if let Some(previous) = previous_longitude {
            if (position.longitude - previous).abs() > 180.0 && !current.is_empty() {
                segments.push(std::mem::take(&mut current));
            }
        }
        current.push([position.longitude, position.latitude]);
        previous_longitude = Some(position.longitude);
    }
    if !current.is_empty() {
        segments.push(current);
    }
    segments
}

/// An owned three-line element set: the program's canonical runtime TLE format.
///
/// A [`SatellitePreset`] is the same data with `'static` strings baked into the
/// binary. A TLE that arrives at runtime - fetched from Space-Track, read from a
/// file, typed into the UI - is turned into one of these before it can be
/// propagated, so every downstream consumer sees a single shape regardless of
/// where the elements came from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TleSet {
    /// Object name for display. `None` when the source gave only the two
    /// orbital lines.
    pub name: Option<String>,
    /// Line 1 of the TLE, exactly as issued (69 columns, checksum included).
    pub line1: String,
    /// Line 2 of the TLE, exactly as issued.
    pub line2: String,
}

impl TleSet {
    /// A set with no object name.
    pub fn new(line1: impl Into<String>, line2: impl Into<String>) -> Self {
        Self {
            name: None,
            line1: line1.into(),
            line2: line2.into(),
        }
    }

    /// A named set.
    pub fn named(
        name: impl Into<String>,
        line1: impl Into<String>,
        line2: impl Into<String>,
    ) -> Self {
        Self {
            name: Some(name.into()),
            line1: line1.into(),
            line2: line2.into(),
        }
    }

    /// Attaches (or replaces) the object name.
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// NORAD catalog number carried in field 2 of line 1 (for example the
    /// `25544` in `1 25544U 98067A ...`). The trailing classification letter is
    /// stripped before parsing.
    pub fn catalog_number(&self) -> Result<u64, SatelliteError> {
        let token = self
            .line1
            .split_whitespace()
            .nth(1)
            .ok_or_else(|| SatelliteError::Tle("line 1 has no catalog-number field".to_owned()))?;
        token
            .trim_end_matches(|character: char| !character.is_ascii_digit())
            .parse()
            .map_err(|_| SatelliteError::Tle(format!("`{token}` is not a catalog number")))
    }

    /// Parses and validates the elements, returning a tracker ready to
    /// propagate. This is the single point where a runtime TLE enters SGP4.
    pub fn tracker(&self) -> Result<SatelliteTracker, SatelliteError> {
        SatelliteTracker::from_tle(self.name.as_deref(), &self.line1, &self.line2)
    }
}

impl From<SatellitePreset> for TleSet {
    fn from(preset: SatellitePreset) -> Self {
        Self::named(preset.name, preset.line1, preset.line2)
    }
}

/// The satellite presets
#[derive(Debug, Clone, Copy)]
pub struct SatellitePreset {
    pub name: &'static str,
    pub line1: &'static str,
    pub line2: &'static str,
}

impl SatellitePreset {
    /// This preset as an owned [`TleSet`].
    pub fn to_tle_set(&self) -> TleSet {
        TleSet::named(self.name, self.line1, self.line2)
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    fn iss_tracker() -> SatelliteTracker {
        let preset = PRESETS[0];
        SatelliteTracker::from_tle(Some(preset.name), preset.line1, preset.line2).unwrap()
    }

    #[test]
    fn iss_orbital_period_is_about_93_minutes() {
        let period = iss_tracker().orbital_period_minutes();
        assert!((period - 93.0).abs() < 1.0, "expected ~93 min, got {period}");
    }

    #[test]
    fn elevation_is_near_90_degrees_directly_under_the_satellite() {
        let tracker = iss_tracker();
        let time = UtcDateTime::date(2026, 3, 1).unwrap();
        let position = tracker.position_at(time).unwrap();

        // The station sits at the satellite's own subpoint, so straight up
        // from the station should point almost exactly at the satellite.
        let elevation = elevation_deg(position.latitude, position.longitude, position.ecef_km);
        assert!(
            elevation > 89.9,
            "expected elevation near 90 deg, got {elevation}"
        );
    }

    #[test]
    fn ground_track_returns_the_requested_sample_count() {
        let tracker = iss_tracker();
        let time = UtcDateTime::date(2026, 3, 1).unwrap();
        let track = tracker.ground_track(time, 41).unwrap();
        assert_eq!(track.len(), 41);
    }

    fn pos(latitude: f64, longitude: f64) -> SatellitePosition {
        SatellitePosition {
            latitude,
            longitude,
            altitude_km: 400.0,
            teme_x_km: 0.0,
            teme_y_km: 0.0,
            teme_z_km: 0.0,
            ecef_km: [0.0; 3],
        }
    }

    #[test]
    fn a_track_crossing_the_dateline_splits_into_two_segments() {
        let track = vec![
            pos(0.0, 170.0),
            pos(1.0, 175.0),
            pos(2.0, -178.0),
            pos(3.0, -170.0),
        ];
        let segments = split_dateline_segments(&track);
        assert_eq!(segments.len(), 2);
        assert_eq!(segments[0].len(), 2);
        assert_eq!(segments[1].len(), 2);
    }

    #[test]
    fn a_track_that_never_crosses_the_dateline_stays_one_segment() {
        let track = vec![pos(0.0, -10.0), pos(1.0, 0.0), pos(2.0, 10.0)];
        let segments = split_dateline_segments(&track);
        assert_eq!(segments.len(), 1);
        assert_eq!(segments[0].len(), 3);
    }
}
