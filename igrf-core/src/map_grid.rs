use std::io::{self, BufRead};
use std::path::Path;

#[derive(Debug, Clone, PartialEq)]
pub struct MapGrid {
    pub latitudes: Vec<f64>,
    pub longitudes: Vec<f64>,
    /// Values are indexed as `data[longitude_index][latitude_index]`.
    pub data: Vec<Vec<f64>>,
    pub step: usize,
}

impl MapGrid {
    fn from_reader(reader: impl BufRead) -> io::Result<Self> {
        Self::from_reader_with_step(reader, 2)
    }

    fn from_reader_with_step(mut reader: impl BufRead, step: usize) -> io::Result<Self> {
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

        // `data` is [col][row], so neither loop variable indexes the vector it
        // iterates and clippy's enumerate() rewrite would transpose the grid.
        #[allow(clippy::needless_range_loop)]
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
}
