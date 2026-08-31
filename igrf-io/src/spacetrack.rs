//! Space-Track.org TLE query

use std::fmt;
use std::time::Duration;

use serde::Deserialize;

use igrf_core::satellite::TleSet;

const LOGIN_URL: &str = "https://www.space-track.org/ajaxauth/login";
const LOGOUT_URL: &str = "https://www.space-track.org/ajaxauth/logout";
const QUERY_BASE: &str = "https://www.space-track.org/basicspacedata/query";

/// Space-Track's recommended "everything current" query: newest `gp` element
/// set per object, decayed objects and stale epochs excluded, ordered by
/// catalog number. `%3E` is `>`.
const ALL_GP_QUERY: &str =
    "class/gp/decay_date/null-val/epoch/%3Enow-30/orderby/norad_cat_id/format/json";

/// Upper bound on a query response we will buffer, well above the ~40 MB full
/// catalog but not unbounded.
const MAX_RESPONSE_BYTES: u64 = 256 * 1024 * 1024;

/// Environment variables read by [`Credentials::from_env`].
pub const IDENTITY_ENV: &str = "SPACETRACK_IDENTITY";
pub const PASSWORD_ENV: &str = "SPACETRACK_PASSWORD";

/// Exceptions messaged to the caller
#[derive(Debug)]
pub enum SpaceTrackError {
    /// Transport-level failure: DNS, TLS, connection reset, timeout.
    Http(String),
    /// The credentials were missing or rejected by the login endpoint.
    Auth(String),
    /// A 4xx/5xx from the query endpoint, or a Space-Track `{"error": ...}` body.
    Api(String),
    /// The response was not the JSON array of rows we expected.
    Decode(String),
}

impl fmt::Display for SpaceTrackError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Http(message) => write!(f, "Space-Track transport error: {message}"),
            Self::Auth(message) => write!(f, "Space-Track authentication failed: {message}"),
            Self::Api(message) => write!(f, "Space-Track query error: {message}"),
            Self::Decode(message) => write!(f, "Space-Track response was not understood: {message}"),
        }
    }
}

impl std::error::Error for SpaceTrackError {}

/// A Space-Track authentication
#[derive(Clone)]
pub struct Credentials {
    pub identity: String,
    pub password: String,
}

/// Hand-written so a stray `{:?}` in a log line or panic message never prints
/// the password.
impl fmt::Debug for Credentials {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Credentials")
            .field("identity", &self.identity)
            .field("password", &"<redacted>")
            .finish()
    }
}

impl Credentials {
    pub fn new(identity: impl Into<String>, password: impl Into<String>) -> Self {
        Self {
            identity: identity.into(),
            password: password.into(),
        }
    }

    /// Reads the credentials from the process environment. The binary is
    /// expected to have loaded a project-local `.env` into the environment
    /// first; this function only sees `std::env`.
    pub fn from_env() -> Result<Self, SpaceTrackError> {
        let identity = std::env::var(IDENTITY_ENV).map_err(|_| {
            SpaceTrackError::Auth(format!("{IDENTITY_ENV} is not set (add it to .env)"))
        })?;
        let password = std::env::var(PASSWORD_ENV).map_err(|_| {
            SpaceTrackError::Auth(format!("{PASSWORD_ENV} is not set (add it to .env)"))
        })?;
        // Handling in case identity and password are set but contain empty strings
        if identity.trim().is_empty() || password.is_empty() {
            return Err(SpaceTrackError::Auth(format!(
                "{IDENTITY_ENV}/{PASSWORD_ENV} are set but empty"
            )));
        }
        Ok(Self { identity, password })
    }
}

/// One row from the `gp` class, every field pulled out with its own type.
/// Space-Track sends numbers as JSON strings and `null` for anything an object
/// lacks, so each field takes a lenient deserializer: text -> `""`, number
/// -> `0`.
#[derive(Debug, Clone, Deserialize)]
pub struct SpaceTrackTle {
    // --- Identity and the raw TLE ---
    #[serde(rename = "NORAD_CAT_ID", deserialize_with = "de_flexible_u64")]
    pub norad_cat_id: u64,
    #[serde(rename = "OBJECT_NAME", default, deserialize_with = "de_string_or_null")]
    pub object_name: String,
    /// International designator, e.g. `1998-067A`.
    #[serde(rename = "OBJECT_ID", default, deserialize_with = "de_string_or_null")]
    pub object_id: String,
    /// The `0 ISS (ZARYA)` name line, if the row carried one.
    #[serde(rename = "TLE_LINE0", default, deserialize_with = "de_string_or_null")]
    pub tle_line0: String,
    /// Defaulted so one catalog row missing its lines can't fail the whole
    /// full-catalog parse; [`crate::TleStore`] drops entries with empty lines.
    #[serde(rename = "TLE_LINE1", default, deserialize_with = "de_string_or_null")]
    pub line1: String,
    #[serde(rename = "TLE_LINE2", default, deserialize_with = "de_string_or_null")]
    pub line2: String,

    // --- OMM header ---
    #[serde(rename = "CCSDS_OMM_VERS", default, deserialize_with = "de_string_or_null")]
    pub ccsds_omm_vers: String,
    #[serde(rename = "COMMENT", default, deserialize_with = "de_string_or_null")]
    pub comment: String,
    /// When Space-Track generated this element set.
    #[serde(rename = "CREATION_DATE", default, deserialize_with = "de_string_or_null")]
    pub creation_date: String,
    #[serde(rename = "ORIGINATOR", default, deserialize_with = "de_string_or_null")]
    pub originator: String,
    #[serde(rename = "CENTER_NAME", default, deserialize_with = "de_string_or_null")]
    pub center_name: String,
    #[serde(rename = "REF_FRAME", default, deserialize_with = "de_string_or_null")]
    pub ref_frame: String,
    #[serde(rename = "TIME_SYSTEM", default, deserialize_with = "de_string_or_null")]
    pub time_system: String,
    #[serde(rename = "MEAN_ELEMENT_THEORY", default, deserialize_with = "de_string_or_null")]
    pub mean_element_theory: String,

    // --- Epoch and mean Keplerian elements ---
    /// Element-set epoch as issued, e.g. `2026-02-05 12:03:05`.
    #[serde(rename = "EPOCH", default, deserialize_with = "de_string_or_null")]
    pub epoch: String,
    /// Revolutions per day.
    #[serde(rename = "MEAN_MOTION", default, deserialize_with = "de_f64_flex")]
    pub mean_motion: f64,
    #[serde(rename = "ECCENTRICITY", default, deserialize_with = "de_f64_flex")]
    pub eccentricity: f64,
    /// Degrees.
    #[serde(rename = "INCLINATION", default, deserialize_with = "de_f64_flex")]
    pub inclination: f64,
    /// Right ascension of the ascending node, degrees.
    #[serde(rename = "RA_OF_ASC_NODE", default, deserialize_with = "de_f64_flex")]
    pub ra_of_asc_node: f64,
    /// Argument of pericenter, degrees.
    #[serde(rename = "ARG_OF_PERICENTER", default, deserialize_with = "de_f64_flex")]
    pub arg_of_pericenter: f64,
    /// Degrees.
    #[serde(rename = "MEAN_ANOMALY", default, deserialize_with = "de_f64_flex")]
    pub mean_anomaly: f64,
    #[serde(rename = "EPHEMERIS_TYPE", default, deserialize_with = "de_i64_flex")]
    pub ephemeris_type: i64,
    /// `U` unclassified / `C` classified / `S` secret.
    #[serde(rename = "CLASSIFICATION_TYPE", default, deserialize_with = "de_string_or_null")]
    pub classification_type: String,
    #[serde(rename = "ELEMENT_SET_NO", default, deserialize_with = "de_i64_flex")]
    pub element_set_no: i64,
    #[serde(rename = "REV_AT_EPOCH", default, deserialize_with = "de_i64_flex")]
    pub rev_at_epoch: i64,
    /// Drag term, earth radii^-1.
    #[serde(rename = "BSTAR", default, deserialize_with = "de_f64_flex")]
    pub bstar: f64,
    /// First derivative of mean motion (ballistic coefficient).
    #[serde(rename = "MEAN_MOTION_DOT", default, deserialize_with = "de_f64_flex")]
    pub mean_motion_dot: f64,
    #[serde(rename = "MEAN_MOTION_DDOT", default, deserialize_with = "de_f64_flex")]
    pub mean_motion_ddot: f64,

    // --- Derived quantities Space-Track adds ---
    /// Kilometers.
    #[serde(rename = "SEMIMAJOR_AXIS", default, deserialize_with = "de_f64_flex")]
    pub semimajor_axis: f64,
    /// Orbital period, minutes.
    #[serde(rename = "PERIOD", default, deserialize_with = "de_f64_flex")]
    pub period: f64,
    /// Apogee altitude, kilometers.
    #[serde(rename = "APOAPSIS", default, deserialize_with = "de_f64_flex")]
    pub apoapsis: f64,
    /// Perigee altitude, kilometers.
    #[serde(rename = "PERIAPSIS", default, deserialize_with = "de_f64_flex")]
    pub periapsis: f64,

    // --- Catalog metadata (the search UI filters on these) ---
    /// `PAYLOAD` / `ROCKET BODY` / `DEBRIS` / `UNKNOWN` / `TBA`.
    #[serde(rename = "OBJECT_TYPE", default, deserialize_with = "de_string_or_null")]
    pub object_type: String,
    /// `SMALL` / `MEDIUM` / `LARGE`, or empty when unknown.
    #[serde(rename = "RCS_SIZE", default, deserialize_with = "de_string_or_null")]
    pub rcs_size: String,
    #[serde(rename = "COUNTRY_CODE", default, deserialize_with = "de_string_or_null")]
    pub country_code: String,
    /// `yyyy-mm-dd`.
    #[serde(rename = "LAUNCH_DATE", default, deserialize_with = "de_string_or_null")]
    pub launch_date: String,
    /// Launch site code, e.g. `AFETR`, `TYMSC`.
    #[serde(rename = "SITE", default, deserialize_with = "de_string_or_null")]
    pub site: String,
    /// `yyyy-mm-dd`, empty while the object is still on orbit.
    #[serde(rename = "DECAY_DATE", default, deserialize_with = "de_string_or_null")]
    pub decay_date: String,
    #[serde(rename = "DATA_SOURCE", default, deserialize_with = "de_string_or_null")]
    pub data_source: String,
    /// Space-Track's own row id for this element set.
    #[serde(rename = "GP_ID", default, deserialize_with = "de_i64_flex")]
    pub gp_id: i64,
}

impl SpaceTrackTle {
    /// Whether both orbital lines are present - the minimum to propagate.
    pub fn has_elements(&self) -> bool {
        !self.line1.trim().is_empty() && !self.line2.trim().is_empty()
    }

    /// Convert to runtime TLE format.
    pub fn to_tle_set(&self) -> TleSet {
        let mut set = TleSet::new(self.line1.trim(), self.line2.trim());
        let name = self.object_name.trim();
        if !name.is_empty() {
            set = set.with_name(name);
        }
        set
    }
}

/// Accepts a JSON string or `null`, mapping `null` (and a missing key) to an
/// empty string. Space-Track uses `null` liberally for catalog metadata.
fn de_string_or_null<'de, D>(deserializer: D) -> Result<String, D::Error>
where
    D: serde::Deserializer<'de>,
{
    Ok(Option::<String>::deserialize(deserializer)?.unwrap_or_default())
}

/// Accepts a JSON number, a numeric string, or `null` (`gp` sends numbers as
/// strings). `null`, an empty string, and a missing key all become `0.0`.
fn de_f64_flex<'de, D>(deserializer: D) -> Result<f64, D::Error>
where
    D: serde::Deserializer<'de>,
{
    use serde::de::Error;
    match serde_json::Value::deserialize(deserializer)? {
        serde_json::Value::Null => Ok(0.0),
        serde_json::Value::String(text) => {
            let text = text.trim();
            if text.is_empty() {
                Ok(0.0)
            } else {
                text.parse().map_err(Error::custom)
            }
        }
        serde_json::Value::Number(number) => {
            number.as_f64().ok_or_else(|| Error::custom("not a float"))
        }
        other => Err(Error::custom(format!("expected a number, got {other}"))),
    }
}

/// Integer counterpart of [`de_f64_flex`]; also tolerates a value written with a
/// decimal point.
fn de_i64_flex<'de, D>(deserializer: D) -> Result<i64, D::Error>
where
    D: serde::Deserializer<'de>,
{
    use serde::de::Error;
    match serde_json::Value::deserialize(deserializer)? {
        serde_json::Value::Null => Ok(0),
        serde_json::Value::String(text) => {
            let text = text.trim();
            if text.is_empty() {
                Ok(0)
            } else {
                text.parse::<i64>()
                    .or_else(|_| text.parse::<f64>().map(|value| value as i64))
                    .map_err(Error::custom)
            }
        }
        serde_json::Value::Number(number) => number
            .as_i64()
            .or_else(|| number.as_f64().map(|value| value as i64))
            .ok_or_else(|| Error::custom("not an integer")),
        other => Err(Error::custom(format!("expected a number, got {other}"))),
    }
}

/// `NORAD_CAT_ID` comes back as a JSON string on the `gp` class and as a number
/// on some others; accept either.
fn de_flexible_u64<'de, D>(deserializer: D) -> Result<u64, D::Error>
where
    D: serde::Deserializer<'de>,
{
    use serde::de::Error;
    match serde_json::Value::deserialize(deserializer)? {
        serde_json::Value::String(text) => text.trim().parse().map_err(D::Error::custom),
        serde_json::Value::Number(number) => number
            .as_u64()
            .ok_or_else(|| D::Error::custom("not a non-negative integer")),
        other => Err(D::Error::custom(format!("expected a catalog number, got {other}"))),
    }
}

/// An authenticated Space-Track session.
pub struct SpaceTrackClient {
    agent: ureq::Agent,
    logged_in: bool,
}

impl Default for SpaceTrackClient {
    fn default() -> Self {
        Self::new()
    }
}

impl SpaceTrackClient {
    pub fn new() -> Self {
        // `http_status_as_error(false)`: a 4xx/5xx from Space-Track carries a
        // useful JSON body, so we read the status and body ourselves rather than
        // letting ureq turn it into an opaque error. The session cookie is kept
        // automatically by the agent (the `cookies` feature).
        let config = ureq::Agent::config_builder()
            // Generous: the full `gp` catalog is tens of MB and can take a
            // while to stream on a slow link. This bounds the whole request,
            // body read included.
            .timeout_global(Some(Duration::from_secs(240)))
            .http_status_as_error(false)
            .user_agent("igrf-rust-spacetrack/0.1")
            .build();
        Self {
            agent: ureq::Agent::new_with_config(config),
            logged_in: false,
        }
    }

    /// Authenticates and keeps the session cookie for subsequent queries.
    pub fn login(&mut self, credentials: &Credentials) -> Result<(), SpaceTrackError> {
        let mut response = self
            .agent
            .post(LOGIN_URL)
            .send_form([
                ("identity", credentials.identity.as_str()),
                ("password", credentials.password.as_str()),
            ])
            .map_err(|error| SpaceTrackError::Http(error.to_string()))?;
        let status = response.status().as_u16();
        // A successful login returns an empty body; a rejected one returns 401
        // (or 200 with `{"Login":"Failed"}` on some deployments).
        let body = response.body_mut().read_to_string().unwrap_or_default();
        if status == 401
            || status == 403
            || (body.contains("\"Login\"") && body.contains("Failed"))
        {
            return Err(SpaceTrackError::Auth(
                "the login endpoint rejected the credentials".to_owned(),
            ));
        }
        if !(200..300).contains(&status) {
            return Err(SpaceTrackError::Api(format!("login returned HTTP {status}")));
        }
        self.logged_in = true;
        Ok(())
    }

    /// Best-effort session teardown. Failures are swallowed: the cookie expires
    /// on its own and there is nothing useful for a caller to do about it.
    pub fn logout(&mut self) {
        if self.logged_in {
            let _ = self.agent.get(LOGOUT_URL).call();
            self.logged_in = false;
        }
    }

    /// The most recent element set for each of the given NORAD catalog numbers,
    /// in one request. **An empty slice fetches the entire on-orbit catalog**
    /// (see [`Self::all_tles`]) - tens of thousands of objects, tens of MB.
    pub fn latest_tles(&self, norad_ids: &[u64]) -> Result<Vec<SpaceTrackTle>, SpaceTrackError> {
        if norad_ids.is_empty() {
            return self.all_tles();
        }
        self.run_query(&latest_by_ids_query(norad_ids))
    }

    /// The newest element set for every on-orbit object Space-Track tracks
    /// (`gp` class, decayed objects excluded). One request; the response is
    /// large, so Space-Track asks that this be run at most a few times a day.
    pub fn all_tles(&self) -> Result<Vec<SpaceTrackTle>, SpaceTrackError> {
        self.run_query(ALL_GP_QUERY)
    }

    /// The most recent element sets for objects whose name matches a Space-Track
    /// predicate value: `^STARLINK` for "starts with", `~~ISS` for "contains"
    /// (both case-insensitive), or a bare `ISS (ZARYA)` for an exact match.
    pub fn latest_tles_by_name(
        &self,
        name_predicate: &str,
    ) -> Result<Vec<SpaceTrackTle>, SpaceTrackError> {
        self.run_query(&latest_by_name_query(name_predicate))
    }

    /// Every on-orbit object of one GP `OBJECT_TYPE` - one of `PAYLOAD`,
    /// `ROCKET BODY`, `DEBRIS`, `UNKNOWN`. Case-insensitive; a space in
    /// `ROCKET BODY` is URL-encoded for you.
    pub fn tles_by_object_type(
        &self,
        object_type: &str,
    ) -> Result<Vec<SpaceTrackTle>, SpaceTrackError> {
        self.run_query(&by_object_type_query(object_type))
    }

    /// Runs a raw query: `predicate_path` is everything after
    /// `/basicspacedata/query/`, for example
    /// `class/gp/DECAY_DATE/null-val/EPOCH/%3Enow-30/format/json`. Always end it
    /// with `format/json`; this parser expects a JSON array.
    pub fn run_query(&self, predicate_path: &str) -> Result<Vec<SpaceTrackTle>, SpaceTrackError> {
        if !self.logged_in {
            return Err(SpaceTrackError::Auth("call login() first".to_owned()));
        }
        let url = format!("{QUERY_BASE}/{}", predicate_path.trim_start_matches('/'));
        let mut response = self
            .agent
            .get(&url)
            .call()
            .map_err(|error| SpaceTrackError::Http(error.to_string()))?;
        let status = response.status().as_u16();
        // ureq's default cap is 10 MB; the full catalog is several times that.
        // 256 MB is far above any real Space-Track response but still bounds a
        // runaway one instead of reading until memory runs out.
        let body = response
            .body_mut()
            .with_config()
            .limit(MAX_RESPONSE_BYTES)
            .lossy_utf8(true)
            .read_to_string()
            .map_err(|error| SpaceTrackError::Http(error.to_string()))?;
        if !(200..300).contains(&status) {
            let trimmed = body.trim();
            let detail = if trimmed.is_empty() {
                String::new()
            } else if trimmed.starts_with('<') {
                // An HTML page rather than an API body means the request never
                // reached the query handler - usually a wrong class or path.
                ": non-JSON response (check the query class/path)".to_owned()
            } else {
                format!(": {}", trimmed.chars().take(300).collect::<String>())
            };
            return Err(SpaceTrackError::Api(format!("HTTP {status}{detail}")));
        }
        parse_rows(&body)
    }
}

impl Drop for SpaceTrackClient {
    fn drop(&mut self) {
        self.logout();
    }
}

/// URL-encodes spaces in a predicate value (an object name can contain them);
/// Space-Track values otherwise need no escaping for the inputs this module
/// builds.
fn encode(value: &str) -> String {
    value.replace(' ', "%20")
}

fn latest_by_ids_query(norad_ids: &[u64]) -> String {
    let ids = norad_ids
        .iter()
        .map(u64::to_string)
        .collect::<Vec<_>>()
        .join(",");
    // `gp` returns the newest element set per object by default, so no ORDINAL
    // filter is needed. (The old `tle_latest` class was deprecated - it 404s.)
    format!("class/gp/NORAD_CAT_ID/{ids}/format/json")
}

fn latest_by_name_query(name_predicate: &str) -> String {
    format!(
        "class/gp/OBJECT_NAME/{}/format/json",
        encode(name_predicate)
    )
}

fn by_object_type_query(object_type: &str) -> String {
    format!(
        "class/gp/OBJECT_TYPE/{}/orderby/norad_cat_id/format/json",
        encode(object_type.trim())
    )
}

fn parse_rows(body: &str) -> Result<Vec<SpaceTrackTle>, SpaceTrackError> {
    let trimmed = body.trim_start();
    // A query error is returned as a JSON object
    if trimmed.starts_with('{') {
        let message = serde_json::from_str::<serde_json::Value>(trimmed)
            .ok()
            .and_then(|value| {
                value
                    .get("error")
                    .and_then(serde_json::Value::as_str)
                    .map(str::to_owned)
            })
            .unwrap_or_else(|| trimmed.chars().take(300).collect());
        return Err(SpaceTrackError::Api(message));
    }
    serde_json::from_str(trimmed).map_err(|error| SpaceTrackError::Decode(error.to_string()))
}
