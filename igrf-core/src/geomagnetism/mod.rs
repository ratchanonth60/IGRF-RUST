mod wmm2025;

pub use wmm2025::Wmm2025;

use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Coordinate {
    pub latitude: f64,
    pub longitude: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoordinateError {
    NonFiniteLatitude,
    NonFiniteLongitude,
    LatitudeOutOfRange,
    LongitudeOutOfRange,
    NonFiniteElevation,
}

impl fmt::Display for CoordinateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::NonFiniteLatitude => "latitude must be finite",
            Self::NonFiniteLongitude => "longitude must be finite",
            Self::LatitudeOutOfRange => "latitude must be between -90 and 90 degrees",
            Self::LongitudeOutOfRange => "longitude must be between -180 and 180 degrees",
            Self::NonFiniteElevation => "elevation must be finite",
        })
    }
}

impl std::error::Error for CoordinateError {}

impl Coordinate {
    pub fn new(latitude: f64, longitude: f64) -> Result<Self, CoordinateError> {
        if !latitude.is_finite() {
            return Err(CoordinateError::NonFiniteLatitude);
        }
        if !longitude.is_finite() {
            return Err(CoordinateError::NonFiniteLongitude);
        }
        if !(-90.0..=90.0).contains(&latitude) {
            return Err(CoordinateError::LatitudeOutOfRange);
        }
        if !(-180.0..=180.0).contains(&longitude) {
            return Err(CoordinateError::LongitudeOutOfRange);
        }
        Ok(Self {
            latitude,
            longitude,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CoordinateZ {
    pub latitude: f64,
    pub longitude: f64,
    /// The same altitude unit used by the original C# calculator. Ground
    /// calculations use zero; callers may supply kilometres for high altitude.
    pub elevation: f64,
}

impl CoordinateZ {
    pub fn new(latitude: f64, longitude: f64, elevation: f64) -> Result<Self, CoordinateError> {
        Coordinate::new(latitude, longitude)?;
        if !elevation.is_finite() {
            return Err(CoordinateError::NonFiniteElevation);
        }
        Ok(Self {
            latitude,
            longitude,
            elevation,
        })
    }

    pub fn coordinate(self) -> Coordinate {
        Coordinate {
            latitude: self.latitude,
            longitude: self.longitude,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct UtcDateTime {
    pub year: i32,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
    pub millisecond: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DateError {
    InvalidDate,
    InvalidTime,
}

impl fmt::Display for DateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::InvalidDate => "invalid UTC calendar date",
            Self::InvalidTime => "invalid UTC time",
        })
    }
}

impl std::error::Error for DateError {}

impl UtcDateTime {
    pub fn new(
        year: i32,
        month: u8,
        day: u8,
        hour: u8,
        minute: u8,
        second: u8,
        millisecond: u16,
    ) -> Result<Self, DateError> {
        if !(1..=9999).contains(&year)
            || !(1..=12).contains(&month)
            || !(1..=days_in_month(year, month)).contains(&day)
        {
            return Err(DateError::InvalidDate);
        }
        if hour > 23 || minute > 59 || second > 59 || millisecond > 999 {
            return Err(DateError::InvalidTime);
        }
        Ok(Self {
            year,
            month,
            day,
            hour,
            minute,
            second,
            millisecond,
        })
    }

    pub fn date(year: i32, month: u8, day: u8) -> Result<Self, DateError> {
        Self::new(year, month, day, 0, 0, 0, 0)
    }

    fn julian_day(self) -> f64 {
        let is_julian = self.year < 1582
            || (self.year == 1582 && (self.month < 10 || (self.month == 10 && self.day < 5)));
        let m = if self.month > 2 {
            self.month as i32
        } else {
            self.month as i32 + 12
        };
        let y = if self.month > 2 {
            self.year
        } else {
            self.year - 1
        };
        let d = self.day as f64
            + self.hour as f64 / 24.0
            + self.minute as f64 / 1440.0
            + (self.second as f64 + self.millisecond as f64 / 1000.0) / 86400.0;
        let b = if is_julian {
            0
        } else {
            2 - y / 100 + (y / 100) / 4
        };
        (365.25 * (y + 4716) as f64).trunc() + (30.6001 * (m + 1) as f64).trunc() + d + b as f64
            - 1524.5
    }
}

/// The radius the WMM spherical-harmonic expansion is defined against
/// (WMM2025 technical report, eq. 1). It is a model constant, not a property of
/// the ellipsoid: using the WGS 84 mean radius instead scales the field by
/// roughly 1e-4 and misses the published test values by about 4 nT.
const GEOMAGNETIC_REFERENCE_RADIUS_KM: f64 = 6371.2;

fn days_in_month(year: i32, month: u8) -> u8 {
    match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if year % 4 == 0 && (year % 100 != 0 || year % 400 == 0) => 29,
        2 => 28,
        _ => 0,
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Wgs84 {
    pub equatorial_axis_m: f64,
    pub inverse_flattening: f64,
}

impl Default for Wgs84 {
    fn default() -> Self {
        Self {
            equatorial_axis_m: 6_378_137.0,
            inverse_flattening: 298.257223563,
        }
    }
}

impl Wgs84 {
    pub fn flattening(self) -> f64 {
        1.0 / self.inverse_flattening
    }

    pub fn polar_axis_m(self) -> f64 {
        self.equatorial_axis_m * (1.0 - self.flattening())
    }

    pub fn eccentricity(self) -> f64 {
        let flattening = self.flattening();
        (2.0 * flattening - flattening * flattening).sqrt()
    }

    pub fn mean_radius_m(self) -> f64 {
        (2.0 * self.equatorial_axis_m + self.polar_axis_m()) / 3.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeomagnetismResult {
    pub coordinate: CoordinateZ,
    pub date: UtcDateTime,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub declination: f64,
    pub inclination: f64,
    pub total_intensity: f64,
    pub horizontal_intensity: f64,
}

impl GeomagnetismResult {
    fn new(coordinate: CoordinateZ, date: UtcDateTime, x: f64, y: f64, z: f64) -> Self {
        // atan2 is defined at the origin, so a near-zero component needs no
        // guard: blanking the whole vector would report a field of zero for a
        // location that merely has no eastward component.
        let horizontal_intensity = (x * x + y * y).sqrt();
        Self {
            coordinate,
            date,
            x,
            y,
            z,
            declination: y.atan2(x).to_degrees(),
            inclination: z.atan2(horizontal_intensity).to_degrees(),
            total_intensity: (x * x + y * y + z * z).sqrt(),
            horizontal_intensity,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GeomagnetismError {
    Coordinate(CoordinateError),
}

impl fmt::Display for GeomagnetismError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Coordinate(error) => error.fmt(f),
        }
    }
}

impl std::error::Error for GeomagnetismError {}

impl From<CoordinateError> for GeomagnetismError {
    fn from(value: CoordinateError) -> Self {
        Self::Coordinate(value)
    }
}

pub struct GeomagnetismCalculator {
    pub spheroid: Wgs84,
}

impl Default for GeomagnetismCalculator {
    fn default() -> Self {
        Self::new()
    }
}

impl GeomagnetismCalculator {
    pub fn new() -> Self {
        Self {
            spheroid: Wgs84::default(),
        }
    }

    pub fn with_spheroid(spheroid: Wgs84) -> Self {
        Self { spheroid }
    }

    pub fn try_calculate(
        &self,
        coordinate: Coordinate,
        date: UtcDateTime,
    ) -> Result<Option<GeomagnetismResult>, GeomagnetismError> {
        self.try_calculate_z(
            CoordinateZ::new(coordinate.latitude, coordinate.longitude, 0.0)?,
            date,
        )
    }

    pub fn try_calculate_at_altitude(
        &self,
        coordinate: Coordinate,
        altitude: f64,
        date: UtcDateTime,
    ) -> Result<Option<GeomagnetismResult>, GeomagnetismError> {
        self.try_calculate_z(
            CoordinateZ::new(coordinate.latitude, coordinate.longitude, altitude)?,
            date,
        )
    }

    pub fn try_calculate_z(
        &self,
        coordinate: CoordinateZ,
        date: UtcDateTime,
    ) -> Result<Option<GeomagnetismResult>, GeomagnetismError> {
        let coordinate = CoordinateZ::new(
            coordinate.latitude,
            coordinate.longitude,
            coordinate.elevation,
        )?;
        if date < Wmm2025::valid_from() || date >= Wmm2025::valid_to() {
            return Ok(None);
        }

        let (theta, r) = self.geodetic_to_spherical(coordinate);
        let lat = coordinate.latitude.to_radians();
        let lon = coordinate.longitude.to_radians();
        let bound = 13;

        let c = theta.cos();
        let s = theta.sin();
        let inv_s = if (s - 0.0).abs() < f64::EPSILON {
            1.0 / (s + 1e-8)
        } else {
            1.0 / s
        };

        let mut p = [[0.0_f64; 13]; 13];
        let mut dp = [[0.0_f64; 13]; 13];
        p[0][0] = 1.0;
        p[1][1] = s;
        dp[0][0] = 0.0;
        dp[1][1] = c;
        p[1][0] = c;
        dp[1][0] = -s;

        for i in 2..bound {
            let root = ((2.0 * i as f64 - 1.0) / (2.0 * i as f64)).sqrt();
            p[i][i] = p[i - 1][i - 1] * s * root;
            dp[i][i] = (dp[i - 1][i - 1] * s + p[i - 1][i - 1] * c) * root;
        }
        for i in 0..bound {
            let i2 = (i * i) as f64;
            for j in (i + 1).max(2)..bound {
                let root1 = (((j - 1) * (j - 1)) as f64 - i2).sqrt();
                let root2 = 1.0 / ((j * j) as f64 - i2).sqrt();
                p[j][i] = (p[j - 1][i] * c * (2.0 * j as f64 - 1.0) - p[j - 2][i] * root1) * root2;
                dp[j][i] = ((dp[j - 1][i] * c - p[j - 1][i] * s) * (2.0 * j as f64 - 1.0)
                    - dp[j - 2][i] * root1)
                    * root2;
            }
        }

        let mut b_radial = 0.0;
        let mut b_theta = 0.0;
        let mut b_phi = 0.0;
        let fn0 = GEOMAGNETIC_REFERENCE_RADIUS_KM / r;
        let mut factor = fn0 * fn0;
        let mut sm = [0.0_f64; 13];
        let mut cm = [0.0_f64; 13];
        sm[0] = 0.0;
        cm[0] = 1.0;
        let yearfrac = (date.julian_day() - Wmm2025::valid_from().julian_day()) / 365.25;

        for i in 1..bound {
            sm[i] = (i as f64 * lon).sin();
            cm[i] = (i as f64 * lon).cos();
            let mut c1 = 0.0;
            let mut c2 = 0.0;
            let mut c3 = 0.0;
            for j in 0..=i {
                let g = Wmm2025::MAIN_G[i][j] + yearfrac * Wmm2025::SECULAR_G[i][j];
                let h = Wmm2025::MAIN_H[i][j] + yearfrac * Wmm2025::SECULAR_H[i][j];
                let c0 = g * cm[j] + h * sm[j];
                c1 += c0 * p[i][j];
                c2 += c0 * dp[i][j];
                c3 += j as f64 * (g * sm[j] - h * cm[j]) * p[i][j];
            }
            factor *= fn0;
            b_radial += (i as f64 + 1.0) * c1 * factor;
            b_theta -= c2 * factor;
            b_phi += c3 * factor * inv_s;
        }

        let psi = theta - (std::f64::consts::FRAC_PI_2 - lat);
        let x = -b_theta * psi.cos() - b_radial * psi.sin();
        let y = b_phi;
        let z = b_theta * psi.sin() - b_radial * psi.cos();
        Ok(Some(GeomagnetismResult::new(coordinate, date, x, y, z)))
    }

    /// Convert geodetic latitude/elevation to the spherical WMM colatitude and
    /// radius used by the original Geo calculator. Radius is in kilometres.
    pub fn geodetic_to_spherical(&self, coordinate: CoordinateZ) -> (f64, f64) {
        let lat = coordinate.latitude.to_radians();
        let elevation = coordinate.elevation;
        let a = self.spheroid.equatorial_axis_m / 1000.0;
        let b = a * (1.0 - self.spheroid.flattening());
        let sin_lat = lat.sin();
        let cos_lat = lat.cos();
        let sin_lat2 = sin_lat * sin_lat;
        let cos_lat2 = cos_lat * cos_lat;
        let a2 = a * a;
        let a4 = a2 * a2;
        let b2 = b * b;
        let b4 = b2 * b2;
        let sr = (a2 * cos_lat2 + b2 * sin_lat2).sqrt();
        let theta = (cos_lat * (elevation * sr + a2)).atan2(sin_lat * (elevation * sr + b2));
        let r = (elevation * elevation
            + 2.0 * elevation * sr
            + (a4 - (a4 - b4) * sin_lat2) / (a2 - (a2 - b2) * sin_lat2))
            .sqrt();
        (theta, r)
    }
}

impl Wmm2025 {
    pub fn valid_from() -> UtcDateTime {
        UtcDateTime::date(Self::VALID_FROM_YEAR, 1, 1).expect("valid WMM date")
    }

    pub fn valid_to() -> UtcDateTime {
        UtcDateTime::date(Self::VALID_TO_YEAR, 1, 1).expect("valid WMM date")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn date(year: i32, month: u8, day: u8) -> UtcDateTime {
        UtcDateTime::date(year, month, day).unwrap()
    }

    #[test]
    fn wgs84_conversion_matches_expected_equatorial_radius() {
        let calculator = GeomagnetismCalculator::new();
        let coordinate = CoordinateZ::new(0.0, 0.0, 0.0).unwrap();
        let (theta, radius) = calculator.geodetic_to_spherical(coordinate);

        assert!((theta - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        assert!((radius - 6378.137).abs() < 1e-9);
    }

    #[test]
    fn wmm2025_returns_finite_field_inside_valid_period() {
        let calculator = GeomagnetismCalculator::new();
        let coordinate = Coordinate::new(13.7563, 100.5018).unwrap();
        let result = calculator
            .try_calculate(coordinate, date(2025, 1, 1))
            .unwrap()
            .unwrap();

        assert!(result.x.is_finite());
        assert!(result.y.is_finite());
        assert!(result.z.is_finite());
        assert!(result.total_intensity > 0.0);
        assert!(
            (result.total_intensity.powi(2)
                - (result.x.powi(2) + result.y.powi(2) + result.z.powi(2)))
            .abs()
                < 1e-6
        );
    }

    /// The official NOAA/BGS WMM2025 test values for 2025.0 (WMM2025 test
    /// value table). Published to 0.1 nT, so the tolerance is 0.2 nT. The
    /// finite/self-consistent test above passes for any tidy but wrong field;
    /// only a real reference vector catches a scale or sign error.
    #[test]
    fn wmm2025_matches_the_published_test_values() {
        // lat, lon, height km, X, Y, Z, H, F, I, D
        let rows = [
            (
                80.0, 0.0, 0.0, 6521.6, 145.9, 54791.5, 6523.2, 55178.5, 83.21, 1.28,
            ),
            (
                0.0, 120.0, 0.0, 39677.8, -109.6, -10580.2, 39677.9, 41064.3, -14.93, -0.16,
            ),
            (
                -80.0, -120.0, 0.0, 6117.5, 15751.9, -52022.5, 16898.1, 54698.2, -72.00, 68.78,
            ),
            (
                80.0, 0.0, 100.0, 6216.0, 92.4, 52598.8, 6216.7, 52964.9, 83.26, 0.85,
            ),
            (
                0.0, 120.0, 100.0, 37688.6, -96.2, -10152.1, 37688.7, 39032.1, -15.08, -0.15,
            ),
            (
                -80.0, -120.0, 100.0, 5907.6, 14780.3, -49540.7, 15917.1, 52035.0, -72.19, 68.21,
            ),
        ];
        let calculator = GeomagnetismCalculator::new();

        for (latitude, longitude, height, x, y, z, h, f, i, d) in rows {
            let coordinate = Coordinate::new(latitude, longitude).unwrap();
            let result = calculator
                .try_calculate_at_altitude(coordinate, height, date(2025, 1, 1))
                .unwrap()
                .unwrap();
            let check = |name: &str, actual: f64, expected: f64, tolerance: f64| {
                assert!(
                    (actual - expected).abs() < tolerance,
                    "{name} at {latitude},{longitude},{height}km: {actual} != {expected}"
                );
            };

            check("X", result.x, x, 0.2);
            check("Y", result.y, y, 0.2);
            check("Z", result.z, z, 0.2);
            check("H", result.horizontal_intensity, h, 0.2);
            check("F", result.total_intensity, f, 0.2);
            check("I", result.inclination, i, 0.01);
            check("D", result.declination, d, 0.01);
        }
    }

    /// `UtcDateTime::date` pins milliseconds to zero, so the field tests above
    /// cannot see a bad millisecond scale factor.
    #[test]
    fn julian_day_counts_milliseconds_as_thousandths_of_a_second() {
        let base = UtcDateTime::new(2025, 6, 1, 12, 0, 0, 0).unwrap();
        let half_second = UtcDateTime::new(2025, 6, 1, 12, 0, 0, 500).unwrap();

        // The Julian day is ~2.46e6, so differencing two of them costs ~5e-10.
        assert!((half_second.julian_day() - base.julian_day() - 0.5 / 86_400.0).abs() < 1e-8);
    }

    #[test]
    fn dates_outside_model_and_invalid_coordinates_are_rejected() {
        let calculator = GeomagnetismCalculator::new();
        let coordinate = Coordinate::new(0.0, 0.0).unwrap();
        assert_eq!(
            calculator.try_calculate(coordinate, date(2024, 12, 31)),
            Ok(None)
        );
        assert!(UtcDateTime::date(0, 1, 1).is_err());
        assert!(UtcDateTime::date(10_000, 1, 1).is_err());
        assert!(Coordinate::new(91.0, 0.0).is_err());
        assert!(CoordinateZ::new(0.0, 0.0, f64::NAN).is_err());
    }
}
