use std::io::{self, BufRead};
use std::path::Path;

#[derive(Debug, Clone, PartialEq)]
pub struct MapGrid {
    pub latitudes: Vec<f64>,
    pub longitudes: Vec<f64>,
    /// Values are indexed as `data[longitude_index][latitude_index]`, matching
    /// the C# `double[newCols, newRows]` array used by the map renderer.
    pub data: Vec<Vec<f64>>,
    pub step: usize,
}

impl MapGrid {
    pub fn from_reader(reader: impl BufRead) -> io::Result<Self> {
        Self::from_reader_with_step(reader, 2)
    }

    pub fn from_reader_with_step(mut reader: impl BufRead, step: usize) -> io::Result<Self> {
        if step == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "grid downsample step must be greater than zero",
            ));
        }

        let lines: Vec<String> = reader.by_ref().lines().collect::<io::Result<_>>()?;
        let rows = 180 / step;
        let cols = 360 / step;
        let latitudes = (0..rows).map(|row| -90.0 + (row * step) as f64).collect();
        let longitudes = (0..cols).map(|col| -180.0 + (col * step) as f64).collect();
        let mut data = vec![vec![0.0; rows]; cols];

        for row in 0..rows {
            let Some(line) = lines.get(row * step) else {
                continue;
            };
            let values: Vec<&str> = line.split(',').filter(|value| !value.is_empty()).collect();
            for col in 0..cols {
                let Some(value) = values.get(col * step) else {
                    break;
                };
                data[col][row] = value.trim().parse().unwrap_or(0.0);
            }
        }

        Ok(Self {
            latitudes,
            longitudes,
            data,
            step,
        })
    }

    pub fn load(path: impl AsRef<Path>) -> io::Result<Self> {
        let file = std::fs::File::open(path)?;
        Self::from_reader(io::BufReader::new(file))
    }

    pub fn value(&self, longitude_index: usize, latitude_index: usize) -> Option<f64> {
        self.data
            .get(longitude_index)
            .and_then(|column| column.get(latitude_index))
            .copied()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn downsamples_csv_and_builds_axes() {
        let input = "1,2,3,4,5\n6,7,8,9,10\n11,12,13,14,15\n";
        let grid = MapGrid::from_reader_with_step(Cursor::new(input), 2).unwrap();

        assert_eq!(grid.step, 2);
        assert_eq!(grid.latitudes[0], -90.0);
        assert_eq!(grid.latitudes[1], -88.0);
        assert_eq!(grid.longitudes[0], -180.0);
        assert_eq!(grid.longitudes[1], -178.0);
        assert_eq!(grid.value(0, 0), Some(1.0));
        assert_eq!(grid.value(1, 0), Some(3.0));
        assert_eq!(grid.value(0, 1), Some(11.0));
    }

    #[test]
    fn malformed_numbers_match_try_parse_zero_and_zero_step_is_rejected() {
        let grid = MapGrid::from_reader(Cursor::new("bad,2\n")).unwrap();
        assert_eq!(grid.value(0, 0), Some(0.0));
        assert!(MapGrid::from_reader_with_step(Cursor::new(""), 0).is_err());
    }
}
