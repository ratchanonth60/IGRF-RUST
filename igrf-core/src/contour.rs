//! Contour-line extraction for a [`MapGrid`], via the standard marching
//! squares algorithm: rather than filling each cell with a colour (a
//! heatmap), it traces the iso-value lines through the grid.

use crate::map_grid::MapGrid;
const MAX_LEVELS: usize = 200;

/// One traced segment of a single contour line, in (longitude, latitude).
/// A full iso-line is the union of every segment sharing the same `level`;
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContourSegment {
    pub level: f64,
    pub start: [f64; 2],
    pub end: [f64; 2],
}

/// Trace iso-lines through `grid` at every multiple of `level_step` inside
/// the grid's value range. Returns an empty vector for a degenerate grid
/// (fewer than 2 rows/columns, non-finite/constant data) or a non-positive
/// step.
pub fn contour_segments(grid: &MapGrid, level_step: f64) -> Vec<ContourSegment> {
    if !level_step.is_finite() || level_step <= 0.0 {
        return Vec::new();
    }
    if grid.longitudes.len() < 2 || grid.latitudes.len() < 2 {
        return Vec::new();
    }

    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    for column in &grid.data {
        for &value in column {
            if value.is_finite() {
                min = min.min(value);
                max = max.max(value);
            }
        }
    }
    if !min.is_finite() || !max.is_finite() || min >= max {
        return Vec::new();
    }

    let mut level = (min / level_step).floor() * level_step;
    if level < min {
        level += level_step;
    }

    let mut segments = Vec::new();
    let mut levels_seen = 0;
    while level <= max && levels_seen < MAX_LEVELS {
        for row in 0..grid.latitudes.len() - 1 {
            for col in 0..grid.longitudes.len() - 1 {
                push_cell_segments(grid, col, row, level, &mut segments);
            }
        }
        level += level_step;
        levels_seen += 1;
    }
    segments
}

/// Marching squares for one grid cell at one level. Corners are numbered
/// counter-clockwise from bottom-left (0=bl, 1=br, 2=tr, 3=tl); edges are
/// named for the side of the cell they cross (bottom/right/top/left).
fn push_cell_segments(
    grid: &MapGrid,
    col: usize,
    row: usize,
    level: f64,
    segments: &mut Vec<ContourSegment>,
) {
    let x0 = grid.longitudes[col];
    let x1 = grid.longitudes[col + 1];
    let y0 = grid.latitudes[row];
    let y1 = grid.latitudes[row + 1];

    let v00 = grid.data[col][row];
    let v10 = grid.data[col + 1][row];
    let v11 = grid.data[col + 1][row + 1];
    let v01 = grid.data[col][row + 1];

    let lerp = |a: f64, b: f64, va: f64, vb: f64| -> f64 {
        if (vb - va).abs() < f64::EPSILON {
            0.5 * (a + b)
        } else {
            a + (level - va) / (vb - va) * (b - a)
        }
    };
    let bottom = [lerp(x0, x1, v00, v10), y0];
    let right = [x1, lerp(y0, y1, v10, v11)];
    let top = [lerp(x0, x1, v01, v11), y1];
    let left = [x0, lerp(y0, y1, v00, v01)];

    let high = |v: f64| v >= level;
    let case = high(v00) as u8 | (high(v10) as u8) << 1 | (high(v11) as u8) << 2 | (high(v01) as u8) << 3;
    let center_high = high((v00 + v10 + v11 + v01) / 4.0);

    let mut push = |a: [f64; 2], b: [f64; 2]| segments.push(ContourSegment { level, start: a, end: b });

    match case {
        0 | 15 => {}
        1 | 14 => push(left, bottom),
        2 | 13 => push(bottom, right),
        3 | 12 => push(left, right),
        4 | 11 => push(right, top),
        6 | 9 => push(bottom, top),
        7 | 8 => push(left, top),
        // Saddle cases: the two diagonal corners agree, so the pairing is
        // ambiguous without a tie-break. Compare the cell-centre value to
        // the level to decide which pair of corners stays connected.
        5 => {
            if center_high {
                push(left, top);
                push(bottom, right);
            } else {
                push(left, bottom);
                push(top, right);
            }
        }
        10 => {
            if center_high {
                push(left, bottom);
                push(top, right);
            } else {
                push(left, top);
                push(bottom, right);
            }
        }
        _ => unreachable!("case is a 4-bit index, 0..=15"),
    }
}
