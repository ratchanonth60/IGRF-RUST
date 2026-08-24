use chrono::{Local, NaiveDate};
use std::fs::{self, File, OpenOptions};
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};

/// Appends rows to `<dir>/<base>_<YYYY-MM-DD>.csv` and starts a new file when the
/// local date changes, matching the C# logger.
///
/// The rotation is not cosmetic: a month at 10 Hz is roughly 6.5 GB and 26
/// million rows in one file, past what a spreadsheet will open at all and past
/// what a naive `read_csv` will fit in memory.
pub struct CsvLogger {
    directory: PathBuf,
    base_name: String,
    header: String,
    writer: BufWriter<File>,
    path: PathBuf,
    day: NaiveDate,
}

impl CsvLogger {
    /// `path` names the series, not the file: `sensor_log.csv` writes
    /// `logs/sensor_log_2026-08-21.csv`. An explicit directory is honoured.
    pub fn open(path: impl AsRef<Path>, header: &str) -> io::Result<Self> {
        let path = path.as_ref();
        let base_name = path
            .file_stem()
            .map(|stem| stem.to_string_lossy().into_owned())
            .filter(|stem| !stem.is_empty())
            .unwrap_or_else(|| "sensor_log".to_owned());
        let directory = match path.parent() {
            Some(parent) if !parent.as_os_str().is_empty() => parent.to_path_buf(),
            _ => PathBuf::from("logs"),
        };

        let day = Local::now().date_naive();
        let (writer, path) = open_day(&directory, &base_name, day, header)?;
        Ok(Self {
            directory,
            base_name,
            header: header.to_owned(),
            writer,
            path,
            day,
        })
    }

    /// File currently being appended to.
    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn write_line(&mut self, line: &str) -> io::Result<()> {
        let today = Local::now().date_naive();
        if today != self.day {
            let (writer, path) = open_day(&self.directory, &self.base_name, today, &self.header)?;
            self.writer = writer;
            self.path = path;
            self.day = today;
        }
        self.writer.write_all(line.as_bytes())?;
        if !line.ends_with('\n') {
            self.writer.write_all(b"\n")?;
        }
        self.writer.flush()
    }
}

fn open_day(
    directory: &Path,
    base_name: &str,
    day: NaiveDate,
    header: &str,
) -> io::Result<(BufWriter<File>, PathBuf)> {
    fs::create_dir_all(directory)?;
    let path = directory.join(format!("{base_name}_{}.csv", day.format("%Y-%m-%d")));
    let write_header = !path.exists() || path.metadata()?.len() == 0;
    let file = OpenOptions::new().create(true).append(true).open(&path)?;
    let mut writer = BufWriter::new(file);
    if write_header {
        writeln!(writer, "{header}")?;
        writer.flush()?;
    }
    Ok((writer, path))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn directory(name: &str) -> PathBuf {
        std::env::temp_dir().join(format!("igrf-log-{}-{name}", std::process::id()))
    }

    #[test]
    fn writes_header_once_and_appends_snapshot_lines() {
        let dir = directory("append");
        let _ = fs::remove_dir_all(&dir);
        let series = dir.join("sensor_log.csv");

        let mut logger = CsvLogger::open(&series, "a,b").unwrap();
        let file = logger.path().to_path_buf();
        logger.write_line("1,2").unwrap();
        drop(logger);

        let mut logger = CsvLogger::open(&series, "a,b").unwrap();
        logger.write_line("3,4").unwrap();
        assert_eq!(logger.path(), file, "same day must reuse the same file");
        drop(logger);

        assert_eq!(fs::read_to_string(&file).unwrap(), "a,b\n1,2\n3,4\n");
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn each_day_gets_its_own_file_with_its_own_header() {
        let dir = directory("rotate");
        let _ = fs::remove_dir_all(&dir);
        let first_day = NaiveDate::from_ymd_opt(2025, 8, 21).unwrap();
        let second_day = NaiveDate::from_ymd_opt(2025, 8, 22).unwrap();

        let (mut first, first_path) = open_day(&dir, "sensor_log", first_day, "a,b").unwrap();
        writeln!(first, "1,2").unwrap();
        first.flush().unwrap();
        let (mut second, second_path) = open_day(&dir, "sensor_log", second_day, "a,b").unwrap();
        writeln!(second, "3,4").unwrap();
        second.flush().unwrap();

        assert_ne!(first_path, second_path);
        assert!(
            first_path.ends_with("sensor_log_2025-08-21.csv"),
            "{first_path:?}"
        );
        assert!(
            second_path.ends_with("sensor_log_2025-08-22.csv"),
            "{second_path:?}"
        );
        assert_eq!(fs::read_to_string(&first_path).unwrap(), "a,b\n1,2\n");
        assert_eq!(fs::read_to_string(&second_path).unwrap(), "a,b\n3,4\n");
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn a_series_name_without_a_directory_lands_under_logs() {
        let dir = directory("bare");
        let _ = fs::remove_dir_all(&dir);
        let day = NaiveDate::from_ymd_opt(2026, 1, 2).unwrap();
        let (_writer, path) = open_day(Path::new("logs"), "sensor_log", day, "a").unwrap();
        assert!(path.starts_with("logs"), "{path:?}");
        assert!(path.ends_with("sensor_log_2026-01-02.csv"), "{path:?}");
        let _ = fs::remove_file(path);
    }
}
