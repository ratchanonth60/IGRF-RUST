//! Project-local SQLite store for TLEs fetched from Space-Track, via Diesel.

use std::path::{Path, PathBuf};

use chrono::Utc;
use diesel::connection::SimpleConnection;
use diesel::prelude::*;
use diesel::sqlite::Sqlite;
use diesel::sqlite::SqliteConnection;

use igrf_core::satellite::TleSet;

use crate::spacetrack::{Credentials, SpaceTrackClient, SpaceTrackError, SpaceTrackTle};

mod schema {
    diesel::table! {
        tle (norad_cat_id) {
            norad_cat_id -> BigInt,
            object_name -> Text,
            object_id -> Text,
            tle_line0 -> Text,
            line1 -> Text,
            line2 -> Text,
            ccsds_omm_vers -> Text,
            comment -> Text,
            creation_date -> Text,
            originator -> Text,
            center_name -> Text,
            ref_frame -> Text,
            time_system -> Text,
            mean_element_theory -> Text,
            epoch -> Text,
            mean_motion -> Double,
            eccentricity -> Double,
            inclination -> Double,
            ra_of_asc_node -> Double,
            arg_of_pericenter -> Double,
            mean_anomaly -> Double,
            ephemeris_type -> BigInt,
            classification_type -> Text,
            element_set_no -> BigInt,
            rev_at_epoch -> BigInt,
            bstar -> Double,
            mean_motion_dot -> Double,
            mean_motion_ddot -> Double,
            semimajor_axis -> Double,
            period -> Double,
            apoapsis -> Double,
            periapsis -> Double,
            object_type -> Text,
            rcs_size -> Text,
            country_code -> Text,
            launch_date -> Text,
            site -> Text,
            decay_date -> Text,
            data_source -> Text,
            gp_id -> BigInt,
            fetched_at -> Text,
        }
    }
}

use schema::tle;

/// Every column except the `norad_cat_id` primary key: `(name, full definition)`.
/// The single source of truth for both `CREATE TABLE` and the add-missing-column
/// migration of an older database.
const COLUMNS: &[(&str, &str)] = &[
    ("object_name", "object_name TEXT NOT NULL DEFAULT ''"),
    ("object_id", "object_id TEXT NOT NULL DEFAULT ''"),
    ("tle_line0", "tle_line0 TEXT NOT NULL DEFAULT ''"),
    ("line1", "line1 TEXT NOT NULL DEFAULT ''"),
    ("line2", "line2 TEXT NOT NULL DEFAULT ''"),
    ("ccsds_omm_vers", "ccsds_omm_vers TEXT NOT NULL DEFAULT ''"),
    ("comment", "comment TEXT NOT NULL DEFAULT ''"),
    ("creation_date", "creation_date TEXT NOT NULL DEFAULT ''"),
    ("originator", "originator TEXT NOT NULL DEFAULT ''"),
    ("center_name", "center_name TEXT NOT NULL DEFAULT ''"),
    ("ref_frame", "ref_frame TEXT NOT NULL DEFAULT ''"),
    ("time_system", "time_system TEXT NOT NULL DEFAULT ''"),
    ("mean_element_theory", "mean_element_theory TEXT NOT NULL DEFAULT ''"),
    ("epoch", "epoch TEXT NOT NULL DEFAULT ''"),
    ("mean_motion", "mean_motion REAL NOT NULL DEFAULT 0"),
    ("eccentricity", "eccentricity REAL NOT NULL DEFAULT 0"),
    ("inclination", "inclination REAL NOT NULL DEFAULT 0"),
    ("ra_of_asc_node", "ra_of_asc_node REAL NOT NULL DEFAULT 0"),
    ("arg_of_pericenter", "arg_of_pericenter REAL NOT NULL DEFAULT 0"),
    ("mean_anomaly", "mean_anomaly REAL NOT NULL DEFAULT 0"),
    ("ephemeris_type", "ephemeris_type INTEGER NOT NULL DEFAULT 0"),
    ("classification_type", "classification_type TEXT NOT NULL DEFAULT ''"),
    ("element_set_no", "element_set_no INTEGER NOT NULL DEFAULT 0"),
    ("rev_at_epoch", "rev_at_epoch INTEGER NOT NULL DEFAULT 0"),
    ("bstar", "bstar REAL NOT NULL DEFAULT 0"),
    ("mean_motion_dot", "mean_motion_dot REAL NOT NULL DEFAULT 0"),
    ("mean_motion_ddot", "mean_motion_ddot REAL NOT NULL DEFAULT 0"),
    ("semimajor_axis", "semimajor_axis REAL NOT NULL DEFAULT 0"),
    ("period", "period REAL NOT NULL DEFAULT 0"),
    ("apoapsis", "apoapsis REAL NOT NULL DEFAULT 0"),
    ("periapsis", "periapsis REAL NOT NULL DEFAULT 0"),
    ("object_type", "object_type TEXT NOT NULL DEFAULT ''"),
    ("rcs_size", "rcs_size TEXT NOT NULL DEFAULT ''"),
    ("country_code", "country_code TEXT NOT NULL DEFAULT ''"),
    ("launch_date", "launch_date TEXT NOT NULL DEFAULT ''"),
    ("site", "site TEXT NOT NULL DEFAULT ''"),
    ("decay_date", "decay_date TEXT NOT NULL DEFAULT ''"),
    ("data_source", "data_source TEXT NOT NULL DEFAULT ''"),
    ("gp_id", "gp_id INTEGER NOT NULL DEFAULT 0"),
    ("fetched_at", "fetched_at TEXT NOT NULL DEFAULT ''"),
];

#[derive(Debug)]
pub enum TleStoreError {
    /// Anything the SQLite layer reported: open failure, constraint, I/O.
    Sql(String),
    /// A lookup for a catalog number that is not in the database.
    NotFound(u64),
}

impl std::fmt::Display for TleStoreError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Sql(message) => write!(f, "TLE store error: {message}"),
            Self::NotFound(id) => write!(f, "no stored TLE for catalog number {id}"),
        }
    }
}

impl std::error::Error for TleStoreError {}

impl From<diesel::result::Error> for TleStoreError {
    fn from(error: diesel::result::Error) -> Self {
        Self::Sql(error.to_string())
    }
}

impl From<diesel::ConnectionError> for TleStoreError {
    fn from(error: diesel::ConnectionError) -> Self {
        Self::Sql(error.to_string())
    }
}

/// The Diesel row for the `tle` table - one field per Space-Track `gp` field.
/// Kept private; callers see the identical [`StoredTle`]. Field order matches
/// the `table!` above.
#[derive(Queryable, Selectable, Insertable, Debug, Clone)]
#[diesel(table_name = tle)]
#[diesel(check_for_backend(diesel::sqlite::Sqlite))]
struct TleRow {
    norad_cat_id: i64,
    object_name: String,
    object_id: String,
    tle_line0: String,
    line1: String,
    line2: String,
    ccsds_omm_vers: String,
    comment: String,
    creation_date: String,
    originator: String,
    center_name: String,
    ref_frame: String,
    time_system: String,
    mean_element_theory: String,
    epoch: String,
    mean_motion: f64,
    eccentricity: f64,
    inclination: f64,
    ra_of_asc_node: f64,
    arg_of_pericenter: f64,
    mean_anomaly: f64,
    ephemeris_type: i64,
    classification_type: String,
    element_set_no: i64,
    rev_at_epoch: i64,
    bstar: f64,
    mean_motion_dot: f64,
    mean_motion_ddot: f64,
    semimajor_axis: f64,
    period: f64,
    apoapsis: f64,
    periapsis: f64,
    object_type: String,
    rcs_size: String,
    country_code: String,
    launch_date: String,
    site: String,
    decay_date: String,
    data_source: String,
    gp_id: i64,
    fetched_at: String,
}

impl TleRow {
    fn from_fetched(tle: &SpaceTrackTle) -> Self {
        Self {
            // SQLite integers are i64; a NORAD id is far smaller, so the cast is
            // lossless in both directions.
            norad_cat_id: tle.norad_cat_id as i64,
            object_name: tle.object_name.trim().to_owned(),
            object_id: tle.object_id.trim().to_owned(),
            tle_line0: tle.tle_line0.trim().to_owned(),
            line1: tle.line1.trim().to_owned(),
            line2: tle.line2.trim().to_owned(),
            ccsds_omm_vers: tle.ccsds_omm_vers.trim().to_owned(),
            comment: tle.comment.trim().to_owned(),
            creation_date: tle.creation_date.trim().to_owned(),
            originator: tle.originator.trim().to_owned(),
            center_name: tle.center_name.trim().to_owned(),
            ref_frame: tle.ref_frame.trim().to_owned(),
            time_system: tle.time_system.trim().to_owned(),
            mean_element_theory: tle.mean_element_theory.trim().to_owned(),
            epoch: tle.epoch.trim().to_owned(),
            mean_motion: tle.mean_motion,
            eccentricity: tle.eccentricity,
            inclination: tle.inclination,
            ra_of_asc_node: tle.ra_of_asc_node,
            arg_of_pericenter: tle.arg_of_pericenter,
            mean_anomaly: tle.mean_anomaly,
            ephemeris_type: tle.ephemeris_type,
            classification_type: tle.classification_type.trim().to_uppercase(),
            element_set_no: tle.element_set_no,
            rev_at_epoch: tle.rev_at_epoch,
            bstar: tle.bstar,
            mean_motion_dot: tle.mean_motion_dot,
            mean_motion_ddot: tle.mean_motion_ddot,
            semimajor_axis: tle.semimajor_axis,
            period: tle.period,
            apoapsis: tle.apoapsis,
            periapsis: tle.periapsis,
            object_type: tle.object_type.trim().to_uppercase(),
            rcs_size: tle.rcs_size.trim().to_uppercase(),
            country_code: tle.country_code.trim().to_uppercase(),
            launch_date: tle.launch_date.trim().to_owned(),
            site: tle.site.trim().to_uppercase(),
            decay_date: tle.decay_date.trim().to_owned(),
            data_source: tle.data_source.trim().to_owned(),
            gp_id: tle.gp_id,
            fetched_at: Utc::now().to_rfc3339(),
        }
    }

    fn into_stored(self) -> StoredTle {
        StoredTle {
            norad_cat_id: self.norad_cat_id as u64,
            object_name: self.object_name,
            object_id: self.object_id,
            tle_line0: self.tle_line0,
            line1: self.line1,
            line2: self.line2,
            ccsds_omm_vers: self.ccsds_omm_vers,
            comment: self.comment,
            creation_date: self.creation_date,
            originator: self.originator,
            center_name: self.center_name,
            ref_frame: self.ref_frame,
            time_system: self.time_system,
            mean_element_theory: self.mean_element_theory,
            epoch: self.epoch,
            mean_motion: self.mean_motion,
            eccentricity: self.eccentricity,
            inclination: self.inclination,
            ra_of_asc_node: self.ra_of_asc_node,
            arg_of_pericenter: self.arg_of_pericenter,
            mean_anomaly: self.mean_anomaly,
            ephemeris_type: self.ephemeris_type,
            classification_type: self.classification_type,
            element_set_no: self.element_set_no,
            rev_at_epoch: self.rev_at_epoch,
            bstar: self.bstar,
            mean_motion_dot: self.mean_motion_dot,
            mean_motion_ddot: self.mean_motion_ddot,
            semimajor_axis: self.semimajor_axis,
            period: self.period,
            apoapsis: self.apoapsis,
            periapsis: self.periapsis,
            object_type: self.object_type,
            rcs_size: self.rcs_size,
            country_code: self.country_code,
            launch_date: self.launch_date,
            site: self.site,
            decay_date: self.decay_date,
            data_source: self.data_source,
            gp_id: self.gp_id,
            fetched_at: self.fetched_at,
        }
    }
}

/// One catalog row as it sits in the database - every `gp` field with its own
/// type.
#[derive(Debug, Clone, PartialEq)]
pub struct StoredTle {
    pub norad_cat_id: u64,
    pub object_name: String,
    /// International designator, e.g. `1998-067A`.
    pub object_id: String,
    /// The `0 ISS (ZARYA)` name line.
    pub tle_line0: String,
    pub line1: String,
    pub line2: String,

    pub ccsds_omm_vers: String,
    pub comment: String,
    /// When Space-Track generated this element set.
    pub creation_date: String,
    pub originator: String,
    pub center_name: String,
    pub ref_frame: String,
    pub time_system: String,
    pub mean_element_theory: String,

    /// Element-set epoch as issued by Space-Track, e.g. `2026-02-05 12:03:05`.
    pub epoch: String,
    /// Revolutions per day.
    pub mean_motion: f64,
    pub eccentricity: f64,
    /// Degrees.
    pub inclination: f64,
    /// Right ascension of the ascending node, degrees.
    pub ra_of_asc_node: f64,
    /// Argument of pericenter, degrees.
    pub arg_of_pericenter: f64,
    /// Degrees.
    pub mean_anomaly: f64,
    pub ephemeris_type: i64,
    /// `U` / `C` / `S`.
    pub classification_type: String,
    pub element_set_no: i64,
    pub rev_at_epoch: i64,
    pub bstar: f64,
    pub mean_motion_dot: f64,
    pub mean_motion_ddot: f64,
    /// Kilometers.
    pub semimajor_axis: f64,
    /// Orbital period, minutes.
    pub period: f64,
    /// Apogee altitude, kilometers.
    pub apoapsis: f64,
    /// Perigee altitude, kilometers.
    pub periapsis: f64,

    /// `PAYLOAD` / `ROCKET BODY` / `DEBRIS` / `UNKNOWN` (upper-cased on store).
    pub object_type: String,
    /// `SMALL` / `MEDIUM` / `LARGE`, or empty.
    pub rcs_size: String,
    pub country_code: String,
    /// `yyyy-mm-dd`.
    pub launch_date: String,
    pub site: String,
    /// `yyyy-mm-dd`, empty while on orbit.
    pub decay_date: String,
    pub data_source: String,
    /// Space-Track's own row id for this element set.
    pub gp_id: i64,
    /// RFC 3339 UTC instant this row was last written.
    pub fetched_at: String,
}

impl StoredTle {
    /// This row in the program's canonical runtime TLE format.
    pub fn to_tle_set(&self) -> TleSet {
        let mut set = TleSet::new(self.line1.trim(), self.line2.trim());
        if !self.object_name.trim().is_empty() {
            set = set.with_name(self.object_name.trim());
        }
        set
    }
}

/// A search over the stored catalog. Every field is optional; a blank field is
/// not a constraint, so several set at once are AND-ed together.
#[derive(Debug, Default, Clone)]
pub struct TleFilter {
    /// Substring match on the object name.
    pub object_name: Option<String>,
    /// Substring match on the catalog number rendered as text.
    pub norad_cat_id: Option<String>,
    /// Exact match on RCS size (`SMALL` / `MEDIUM` / `LARGE`).
    pub rcs_size: Option<String>,
    /// Exact match on object type (`PAYLOAD` / `ROCKET BODY` / ...).
    pub object_type: Option<String>,
    /// Substring match on the launch site code.
    pub site: Option<String>,
    /// Substring match on the country code.
    pub country_code: Option<String>,
    /// `yyyy-mm-dd`; keeps rows whose launch date is on or after this.
    pub launch_date_from: Option<String>,
    /// `yyyy-mm-dd`; keeps rows whose decay date is on or after this
    /// (still-on-orbit rows, which have no decay date, are excluded).
    pub decay_date_from: Option<String>,
}

impl TleFilter {
    /// True when no field constrains the search - it would match everything.
    pub fn is_empty(&self) -> bool {
        [
            &self.object_name,
            &self.norad_cat_id,
            &self.rcs_size,
            &self.object_type,
            &self.site,
            &self.country_code,
            &self.launch_date_from,
            &self.decay_date_from,
        ]
        .into_iter()
        .all(|field| nonblank(field).is_none())
    }
}

/// One page of search results.
#[derive(Debug, Clone)]
pub struct TlePage {
    pub rows: Vec<StoredTle>,
    /// Total matches across all pages.
    pub total: usize,
    /// Zero-based page index this covers.
    pub page: usize,
    pub per_page: usize,
}

impl TlePage {
    pub fn page_count(&self) -> usize {
        self.total.div_ceil(self.per_page.max(1)).max(1)
    }
}

fn nonblank(value: &Option<String>) -> Option<&str> {
    value
        .as_deref()
        .map(str::trim)
        .filter(|text| !text.is_empty())
}

pub struct TleStore {
    connection: SqliteConnection,
    path: PathBuf,
}

impl TleStore {
    /// Opens (creating if needed) a database file and applies the schema.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, TleStoreError> {
        let path = path.as_ref().to_path_buf();
        let connection = SqliteConnection::establish(&path.to_string_lossy())?;
        let mut store = Self { connection, path };
        store.migrate()?;
        Ok(store)
    }

    /// An anonymous in-memory database, for tests and callers that only want
    /// the query/format helpers without a file on disk.
    pub fn open_in_memory() -> Result<Self, TleStoreError> {
        let connection = SqliteConnection::establish(":memory:")?;
        let mut store = Self {
            connection,
            path: PathBuf::from(":memory:"),
        };
        store.migrate()?;
        Ok(store)
    }

    /// The file this store is backed by.
    pub fn path(&self) -> &Path {
        &self.path
    }

    fn migrate(&mut self) -> Result<(), TleStoreError> {
        let column_defs = COLUMNS
            .iter()
            .map(|(_, ddl)| *ddl)
            .collect::<Vec<_>>()
            .join(",\n    ");
        self.connection.batch_execute(&format!(
            "CREATE TABLE IF NOT EXISTS tle (\n    norad_cat_id INTEGER PRIMARY KEY,\n    {column_defs}\n);\n\
             CREATE INDEX IF NOT EXISTS tle_object_name ON tle (object_name);"
        ))?;

        // A database from an earlier build has fewer columns; add the missing
        // ones. `PRAGMA table_info` lists what the table currently has.
        #[derive(QueryableByName)]
        struct ColumnName {
            #[diesel(sql_type = diesel::sql_types::Text)]
            name: String,
        }
        let present: Vec<String> = diesel::sql_query("PRAGMA table_info(tle)")
            .load::<ColumnName>(&mut self.connection)?
            .into_iter()
            .map(|column| column.name)
            .collect();
        for (name, ddl) in COLUMNS {
            if !present.iter().any(|column| column == name) {
                diesel::sql_query(format!("ALTER TABLE tle ADD COLUMN {ddl}"))
                    .execute(&mut self.connection)?;
            }
        }
        // `raw_json` was a short-lived approach - drop it if a build left one.
        if present.iter().any(|column| column == "raw_json") {
            let _ = diesel::sql_query("ALTER TABLE tle DROP COLUMN raw_json")
                .execute(&mut self.connection);
        }

        // Now that `object_type` is guaranteed to exist, its index can be built.
        self.connection
            .batch_execute("CREATE INDEX IF NOT EXISTS tle_object_type ON tle (object_type);")?;
        Ok(())
    }

    /// Replaces the stored row for this object: on a catalog-number conflict the
    /// old row is deleted and this one inserted in its place. There is no epoch
    /// check - the table holds the last fetch, not the "best" one.
    pub fn replace(&mut self, tle: &SpaceTrackTle) -> Result<(), TleStoreError> {
        diesel::replace_into(tle::table)
            .values(TleRow::from_fetched(tle))
            .execute(&mut self.connection)?;
        Ok(())
    }

    /// [`Self::replace`] for several objects, in one transaction. Rows without
    /// both orbital lines (some full-catalog entries) are skipped. Returns how
    /// many rows were written.
    ///
    /// Inserted one row per statement rather than as a single multi-row INSERT:
    /// the full catalog is tens of thousands of rows, well past SQLite's bound
    /// parameter ceiling for one statement.
    pub fn replace_many(&mut self, tles: &[SpaceTrackTle]) -> Result<usize, TleStoreError> {
        let written = self
            .connection
            .transaction::<usize, diesel::result::Error, _>(|connection| {
                let mut written = 0;
                for tle in tles {
                    if !tle.has_elements() {
                        continue;
                    }
                    diesel::replace_into(tle::table)
                        .values(TleRow::from_fetched(tle))
                        .execute(connection)?;
                    written += 1;
                }
                Ok(written)
            })?;
        Ok(written)
    }

    /// Drops the stored row for one catalog number. Returns whether a row was
    /// actually removed.
    pub fn remove(&mut self, norad_cat_id: u64) -> Result<bool, TleStoreError> {
        let affected = diesel::delete(tle::table.find(norad_cat_id as i64))
            .execute(&mut self.connection)?;
        Ok(affected > 0)
    }

    /// The stored row for one catalog number, if present.
    pub fn get(&mut self, norad_cat_id: u64) -> Result<Option<StoredTle>, TleStoreError> {
        let row = tle::table
            .find(norad_cat_id as i64)
            .select(TleRow::as_select())
            .first(&mut self.connection)
            .optional()?;
        Ok(row.map(TleRow::into_stored))
    }

    /// Every stored row, ordered by catalog number.
    pub fn all(&mut self) -> Result<Vec<StoredTle>, TleStoreError> {
        let rows = tle::table
            .order(tle::norad_cat_id.asc())
            .select(TleRow::as_select())
            .load(&mut self.connection)?;
        Ok(rows.into_iter().map(TleRow::into_stored).collect())
    }

    /// Stored rows whose object name matches a SQL `LIKE` pattern (`%` and `_`
    /// wildcards), case-insensitively for ASCII.
    pub fn search_by_name(&mut self, like: &str) -> Result<Vec<StoredTle>, TleStoreError> {
        let rows = tle::table
            .filter(tle::object_name.like(like))
            .order(tle::object_name.asc())
            .select(TleRow::as_select())
            .load(&mut self.connection)?;
        Ok(rows.into_iter().map(TleRow::into_stored).collect())
    }

    /// The catalog metadata search behind the Satellite Position panel. Returns
    /// one page of matches (`per_page` rows starting at `page * per_page`) plus
    /// the total count. An entirely blank filter matches everything.
    pub fn search(
        &mut self,
        filter: &TleFilter,
        page: usize,
        per_page: usize,
    ) -> Result<TlePage, TleStoreError> {
        let per_page = per_page.max(1);
        let offset = (page as i64) * per_page as i64;

        let total: i64 = apply_filter(tle::table.into_boxed(), filter)
            .count()
            .get_result(&mut self.connection)?;

        let rows: Vec<TleRow> = apply_filter(tle::table.into_boxed(), filter)
            .order((tle::object_name.asc(), tle::norad_cat_id.asc()))
            .select(TleRow::as_select())
            .limit(per_page as i64)
            .offset(offset)
            .load(&mut self.connection)?;

        Ok(TlePage {
            rows: rows.into_iter().map(TleRow::into_stored).collect(),
            total: total.max(0) as usize,
            page,
            per_page,
        })
    }

    /// The formatting step: a stored row as an [`igrf_core::satellite::TleSet`],
    /// ready to hand to the propagator. Errors with [`TleStoreError::NotFound`]
    /// when nothing is stored for that catalog number.
    pub fn tle_set(&mut self, norad_cat_id: u64) -> Result<TleSet, TleStoreError> {
        self.get(norad_cat_id)?
            .map(|stored| stored.to_tle_set())
            .ok_or(TleStoreError::NotFound(norad_cat_id))
    }

    /// Every stored row in runtime form, ordered by catalog number.
    pub fn all_tle_sets(&mut self) -> Result<Vec<TleSet>, TleStoreError> {
        Ok(self.all()?.iter().map(StoredTle::to_tle_set).collect())
    }
}

/// Adds one `WHERE` clause per non-blank filter field to a boxed query.
fn apply_filter<'a>(
    mut query: tle::BoxedQuery<'a, Sqlite>,
    filter: &TleFilter,
) -> tle::BoxedQuery<'a, Sqlite> {
    use diesel::sql_types::{Bool, Text};

    if let Some(name) = nonblank(&filter.object_name) {
        query = query.filter(tle::object_name.like(format!("%{name}%")));
    }
    if let Some(id) = nonblank(&filter.norad_cat_id) {
        // Substring match on the number as text, so "255" finds 25544.
        query = query.filter(
            diesel::dsl::sql::<Bool>("CAST(norad_cat_id AS TEXT) LIKE ")
                .bind::<Text, _>(format!("%{id}%")),
        );
    }
    if let Some(size) = nonblank(&filter.rcs_size) {
        query = query.filter(tle::rcs_size.eq(size.to_uppercase()));
    }
    if let Some(kind) = nonblank(&filter.object_type) {
        query = query.filter(tle::object_type.eq(kind.to_uppercase()));
    }
    if let Some(site) = nonblank(&filter.site) {
        query = query.filter(tle::site.like(format!("%{}%", site.to_uppercase())));
    }
    if let Some(country) = nonblank(&filter.country_code) {
        query = query.filter(tle::country_code.like(format!("%{}%", country.to_uppercase())));
    }
    if let Some(from) = nonblank(&filter.launch_date_from) {
        query = query.filter(tle::launch_date.ge(from.to_owned()));
    }
    if let Some(from) = nonblank(&filter.decay_date_from) {
        query = query
            .filter(tle::decay_date.ne(""))
            .filter(tle::decay_date.ge(from.to_owned()));
    }
    query
}

/// Logs in to Space-Track, fetches the latest element sets, replaces those rows
/// in the local database, and returns how many rows were written. One login and
/// one query; the session is logged out before returning.
///
/// An **empty** `norad_ids` fetches the entire on-orbit catalog (tens of
/// thousands of objects); otherwise just those catalog numbers.
pub fn refresh_from_spacetrack(
    credentials: &Credentials,
    store: &mut TleStore,
    norad_ids: &[u64],
) -> Result<usize, RefreshError> {
    let mut client = SpaceTrackClient::new();
    client.login(credentials)?;
    let fetched = client.latest_tles(norad_ids)?;
    client.logout();

    Ok(store.replace_many(&fetched)?)
}

#[derive(Debug)]
pub enum RefreshError {
    SpaceTrack(SpaceTrackError),
    Store(TleStoreError),
}

impl std::fmt::Display for RefreshError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SpaceTrack(error) => write!(f, "{error}"),
            Self::Store(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for RefreshError {}

impl From<SpaceTrackError> for RefreshError {
    fn from(error: SpaceTrackError) -> Self {
        Self::SpaceTrack(error)
    }
}

impl From<TleStoreError> for RefreshError {
    fn from(error: TleStoreError) -> Self {
        Self::Store(error)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn row(id: u64, epoch: &str) -> SpaceTrackTle {
        serde_json::from_value(serde_json::json!({
            "NORAD_CAT_ID": id.to_string(),
            "OBJECT_NAME": "ISS (ZARYA)",
            "OBJECT_ID": "1998-067A",
            "EPOCH": epoch,
            "TLE_LINE1": "1 25544U 98067A   26036.50214262  .00012860  00000+0  24571-3 0  9997",
            "TLE_LINE2": "2 25544  51.6316 231.4727 0011155  67.3664 292.8503 15.48414003551342",
        }))
        .unwrap()
    }

    fn detailed(
        id: u64,
        name: &str,
        object_type: &str,
        rcs: &str,
        site: &str,
        launch: &str,
        decay: &str,
        country: &str,
    ) -> SpaceTrackTle {
        serde_json::from_value(serde_json::json!({
            "NORAD_CAT_ID": id.to_string(),
            "OBJECT_NAME": name,
            "OBJECT_ID": "2000-000A",
            "EPOCH": "2026-02-05 12:00:00",
            "TLE_LINE1": "1 25544U 98067A   26036.50214262  .00012860  00000+0  24571-3 0  9997",
            "TLE_LINE2": "2 25544  51.6316 231.4727 0011155  67.3664 292.8503 15.48414003551342",
            "OBJECT_TYPE": object_type,
            "RCS_SIZE": rcs,
            "SITE": site,
            "LAUNCH_DATE": launch,
            "DECAY_DATE": decay,
            "COUNTRY_CODE": country,
        }))
        .unwrap()
    }

    #[test]
    fn replace_then_format_round_trips_through_the_runtime_type() {
        let mut store = TleStore::open_in_memory().unwrap();
        store.replace(&row(25544, "2026-02-05 12:03:05")).unwrap();

        let set = store.tle_set(25544).unwrap();
        assert_eq!(set.name.as_deref(), Some("ISS (ZARYA)"));
        assert_eq!(set.catalog_number().unwrap(), 25544);
        assert!(set.tracker().is_ok());

        assert!(matches!(
            store.tle_set(99999),
            Err(TleStoreError::NotFound(99999))
        ));
    }

    #[test]
    fn every_gp_field_round_trips_through_its_typed_column() {
        let mut store = TleStore::open_in_memory().unwrap();
        let tle: SpaceTrackTle = serde_json::from_value(serde_json::json!({
            "NORAD_CAT_ID": "25544",
            "OBJECT_NAME": "ISS (ZARYA)",
            "OBJECT_ID": "1998-067A",
            "TLE_LINE0": "0 ISS (ZARYA)",
            "TLE_LINE1": "1 25544U 98067A   26036.50214262  .00012860  00000+0  24571-3 0  9997",
            "TLE_LINE2": "2 25544  51.6316 231.4727 0011155  67.3664 292.8503 15.48414003551342",
            "ORIGINATOR": "18 SPCS",
            "REF_FRAME": "TEME",
            "EPOCH": "2026-02-05 12:03:05",
            "MEAN_MOTION": "15.50103472",
            "ECCENTRICITY": "0.0011155",
            "INCLINATION": "51.6316",
            "BSTAR": "0.00024571",
            "REV_AT_EPOCH": "55134",
            "SEMIMAJOR_AXIS": "6795.5",
            "PERIOD": "92.9",
            "APOAPSIS": "425.1",
            "PERIAPSIS": "410.0",
            "GP_ID": 271199369,
            "OBJECT_TYPE": "PAYLOAD",
        }))
        .unwrap();
        store.replace(&tle).unwrap();

        let stored = store.get(25544).unwrap().unwrap();
        assert_eq!(stored.tle_line0, "0 ISS (ZARYA)");
        assert_eq!(stored.originator, "18 SPCS");
        assert_eq!(stored.ref_frame, "TEME");
        assert_eq!(stored.mean_motion, 15.50103472);
        assert_eq!(stored.inclination, 51.6316);
        assert_eq!(stored.bstar, 0.00024571);
        assert_eq!(stored.rev_at_epoch, 55134);
        assert_eq!(stored.semimajor_axis, 6795.5);
        assert_eq!(stored.apoapsis, 425.1);
        assert_eq!(stored.gp_id, 271199369);
    }

    #[test]
    fn replace_drops_the_old_row_for_that_object() {
        let mut store = TleStore::open_in_memory().unwrap();
        store.replace(&row(25544, "2026-02-05 00:00:00")).unwrap();
        store.replace(&row(25544, "2026-01-01 00:00:00")).unwrap();

        assert_eq!(store.all().unwrap().len(), 1);
        assert_eq!(store.get(25544).unwrap().unwrap().epoch, "2026-01-01 00:00:00");
    }

    #[test]
    fn replace_many_keeps_one_row_per_object_and_remove_works() {
        let mut store = TleStore::open_in_memory().unwrap();
        let written = store
            .replace_many(&[row(25544, "2026-02-05 12:00:00"), row(33396, "2026-02-05 13:00:00")])
            .unwrap();
        assert_eq!(written, 2);
        assert_eq!(store.all().unwrap().len(), 2);
        assert_eq!(store.all_tle_sets().unwrap().len(), 2);

        assert!(store.remove(25544).unwrap());
        assert!(!store.remove(25544).unwrap());
        assert_eq!(store.all().unwrap().len(), 1);
    }

    #[test]
    fn replace_many_skips_rows_that_have_no_orbital_lines() {
        let mut store = TleStore::open_in_memory().unwrap();
        let mut bare = row(70000, "2026-02-05 12:00:00");
        bare.line1 = String::new();
        bare.line2 = String::new();
        let written = store
            .replace_many(&[row(25544, "2026-02-05 12:00:00"), bare])
            .unwrap();
        assert_eq!(written, 1);
        assert!(store.get(70000).unwrap().is_none());
    }

    #[test]
    fn opening_a_pre_metadata_database_adds_the_missing_columns() {
        use diesel::connection::SimpleConnection;
        let path = std::env::temp_dir()
            .join(format!("igrf-tle-{}-{}.db", std::process::id(), "oldschema"));
        let _ = std::fs::remove_file(&path);

        // A database as an earlier build wrote it: table, no metadata columns.
        {
            let mut old = SqliteConnection::establish(&path.to_string_lossy()).unwrap();
            old.batch_execute(
                "CREATE TABLE tle (
                     norad_cat_id INTEGER PRIMARY KEY,
                     object_name TEXT NOT NULL DEFAULT '',
                     object_id TEXT NOT NULL DEFAULT '',
                     epoch TEXT NOT NULL DEFAULT '',
                     line1 TEXT NOT NULL,
                     line2 TEXT NOT NULL,
                     fetched_at TEXT NOT NULL
                 );
                 INSERT INTO tle VALUES (25544,'ISS','1998-067A','2026-02-05','1 25544U','2 25544','2026-02-05T00:00:00+00:00');",
            )
            .unwrap();
        }

        let mut store = TleStore::open(&path).unwrap();
        // The old row survived and now reads back through the widened schema.
        let stored = store.get(25544).unwrap().unwrap();
        assert_eq!(stored.object_name, "ISS");
        assert_eq!(stored.object_type, "");
        // New writes populate the metadata columns.
        store
            .replace(&detailed(
                40000, "SAT", "PAYLOAD", "SMALL", "AFETR", "2020-01-01", "", "US",
            ))
            .unwrap();
        let page = store
            .search(
                &TleFilter {
                    object_type: Some("PAYLOAD".into()),
                    ..Default::default()
                },
                0,
                10,
            )
            .unwrap();
        assert_eq!(page.total, 1);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn a_file_backed_store_persists_across_reopen() {
        let path = std::env::temp_dir()
            .join(format!("igrf-tle-{}-{}.db", std::process::id(), "reopen"));
        let _ = std::fs::remove_file(&path);

        {
            let mut store = TleStore::open(&path).unwrap();
            store.replace(&row(25544, "2026-02-05 12:03:05")).unwrap();
        }
        {
            let mut store = TleStore::open(&path).unwrap();
            assert_eq!(store.tle_set(25544).unwrap().catalog_number().unwrap(), 25544);
        }
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn search_ands_filters_and_paginates() {
        let mut store = TleStore::open_in_memory().unwrap();
        store
            .replace_many(&[
                detailed(1, "STARLINK-1", "PAYLOAD", "SMALL", "AFETR", "2019-05-24", "", "US"),
                detailed(2, "STARLINK-2", "PAYLOAD", "SMALL", "AFETR", "2019-11-11", "", "US"),
                detailed(3, "SL-16 R/B", "ROCKET BODY", "LARGE", "TYMSC", "1988-01-01", "2005-06-01", "CIS"),
                detailed(4, "COSMOS DEBRIS", "DEBRIS", "MEDIUM", "PKMTR", "1975-03-03", "1999-09-09", "CIS"),
            ])
            .unwrap();

        // Single filter.
        let payloads = store
            .search(
                &TleFilter {
                    object_type: Some("PAYLOAD".into()),
                    ..Default::default()
                },
                0,
                10,
            )
            .unwrap();
        assert_eq!(payloads.total, 2);

        // Multiple filters AND-ed: payload + small + launched on/after 2019-06.
        let recent = store
            .search(
                &TleFilter {
                    object_type: Some("payload".into()),
                    rcs_size: Some("small".into()),
                    launch_date_from: Some("2019-06-01".into()),
                    ..Default::default()
                },
                0,
                10,
            )
            .unwrap();
        assert_eq!(recent.total, 1);
        assert_eq!(recent.rows[0].object_name, "STARLINK-2");

        // Decay filter excludes still-on-orbit rows.
        let decayed = store
            .search(
                &TleFilter {
                    decay_date_from: Some("2000-01-01".into()),
                    ..Default::default()
                },
                0,
                10,
            )
            .unwrap();
        assert_eq!(decayed.total, 1);
        assert_eq!(decayed.rows[0].norad_cat_id, 3);

        // Pagination: 4 rows, 2 per page.
        let page0 = store.search(&TleFilter::default(), 0, 2).unwrap();
        assert_eq!(page0.total, 4);
        assert_eq!(page0.rows.len(), 2);
        assert_eq!(page0.page_count(), 2);
        let page1 = store.search(&TleFilter::default(), 1, 2).unwrap();
        assert_eq!(page1.rows.len(), 2);
        assert_ne!(page0.rows[0].norad_cat_id, page1.rows[0].norad_cat_id);

        // NORAD substring.
        let by_id = store
            .search(
                &TleFilter {
                    norad_cat_id: Some("3".into()),
                    ..Default::default()
                },
                0,
                10,
            )
            .unwrap();
        assert_eq!(by_id.total, 1);
        assert_eq!(by_id.rows[0].norad_cat_id, 3);
    }
}
