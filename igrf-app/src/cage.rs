//! Isometric view of the three Helmholtz coil pairs, with dashes running along
//! each loop to show which way its drive is flowing and how hard.
//!
//! Geometry follows the NARIT cage: three nested square pairs, inner on Y,
//! medium on X, external on Z.

use eframe::egui::{self, Align2, Color32, FontId, Pos2, Sense, Shape, Stroke, Vec2};

const DASH: f32 = 7.0;
const GAP: f32 = 7.0;
/// Screen pixels a dash travels per second at full drive.
const FLOW_SPEED: f32 = 70.0;
/// Axis colours match the triad on the system diagram.
const AXIS_COLOR: [Color32; 3] = [
    Color32::from_rgb(235, 120, 55),
    Color32::from_rgb(95, 195, 95),
    Color32::from_rgb(95, 145, 235),
];
const AXIS_NAME: [&str; 3] = ["X medium", "Y inner", "Z external"];
/// Half side length per pair, so the Y pair nests inside X inside Z.
const HALF_SIDE: [f64; 3] = [0.86, 0.72, 1.0];
/// A square Helmholtz pair is separated by 0.5445 of its side length.
const SEPARATION: f64 = 0.5445;

/// Camera angles and the dash animation phase of each axis.
pub struct CageView {
    yaw: f32,
    pitch: f32,
    phase: [f32; 3],
}

impl Default for CageView {
    fn default() -> Self {
        Self {
            yaw: 0.7,
            pitch: 0.45,
            phase: [0.0; 3],
        }
    }
}

/// Draws the cage. `drive` is each axis output normalised to -1..=1, so the
/// dashes carry sign and magnitude rather than raw controller units.
pub fn show(ui: &mut egui::Ui, view: &mut CageView, drive: [f64; 3]) {
    let size = ui.available_width().clamp(160.0, 420.0);
    let (response, painter) = ui.allocate_painter(Vec2::splat(size), Sense::drag());
    if response.dragged() {
        let delta = response.drag_delta();
        view.yaw += delta.x * 0.01;
        view.pitch = (view.pitch + delta.y * 0.01).clamp(-1.4, 1.4);
    }

    let dt = ui.input(|input| input.stable_dt).clamp(0.0, 0.1);
    let period = DASH + GAP;
    for (phase, drive) in view.phase.iter_mut().zip(drive) {
        let step = drive.clamp(-1.0, 1.0) as f32 * FLOW_SPEED * dt;
        *phase = (*phase - step).rem_euclid(period);
    }

    let rect = response.rect;
    let center = rect.center();
    let scale = size * 0.3;
    painter.rect_filled(rect, 4.0, Color32::from_gray(16));

    // Painter's algorithm: far loops first so near ones draw over them.
    let mut loops: Vec<(usize, Vec<Pos2>, f32)> = Vec::with_capacity(6);
    for (axis, half) in HALF_SIDE.into_iter().enumerate() {
        for side in [-1.0, 1.0] {
            let projected: Vec<(Pos2, f32)> = square_loop(axis, side * SEPARATION * half, half)
                .iter()
                .map(|point| project(*point, view.yaw, view.pitch, scale, center))
                .collect();
            let depth =
                projected.iter().map(|(_, depth)| depth).sum::<f32>() / projected.len() as f32;
            loops.push((
                axis,
                projected.into_iter().map(|(pos, _)| pos).collect(),
                depth,
            ));
        }
    }
    loops.sort_by(|a, b| a.2.total_cmp(&b.2));

    // Test volume on the air bearing, drawn between the far and near loops so it
    // reads as sitting inside the cage.
    let (origin, _) = project([0.0; 3], view.yaw, view.pitch, scale, center);
    let split = loops.partition_point(|(_, _, depth)| *depth < 0.0);
    for (index, (axis, points, _)) in loops.iter().enumerate() {
        if index == split {
            painter.circle_filled(origin, size * 0.055, Color32::from_gray(70));
        }
        let magnitude = (drive[*axis].abs() as f32).clamp(0.0, 1.0);
        let color = AXIS_COLOR[*axis];
        painter.add(Shape::line(
            points.clone(),
            Stroke::new(1.0, color.gamma_multiply(0.3)),
        ));
        if magnitude > 0.005 {
            painter.extend(Shape::dashed_line_with_offset(
                points,
                Stroke::new(1.5 + 2.5 * magnitude, color),
                &[DASH],
                &[GAP],
                view.phase[*axis],
            ));
        }
    }
    if split >= loops.len() {
        painter.circle_filled(origin, size * 0.055, Color32::from_gray(70));
    }

    // The 50ms UI timer is too coarse for smooth dashes; ask for full frame rate
    // only while something is actually flowing.
    if drive.iter().any(|value| value.abs() > 0.005) {
        ui.ctx().request_repaint();
    }

    draw_triad(&painter, view, rect, scale * 0.28);

    let font = FontId::proportional(11.0);
    for (axis, name) in AXIS_NAME.iter().enumerate() {
        painter.text(
            rect.left_top() + Vec2::new(6.0, 6.0 + 14.0 * axis as f32),
            Align2::LEFT_TOP,
            format!("{name}  {:+.0}%", drive[axis] * 100.0),
            font.clone(),
            AXIS_COLOR[axis],
        );
    }
    painter.text(
        rect.left_bottom() + Vec2::new(6.0, -6.0),
        Align2::LEFT_BOTTOM,
        "drag to rotate \u{2022} dash speed = |output|",
        font,
        Color32::from_gray(130),
    );
}

/// The four corners of one square coil. `axis` is the loop's normal, `offset`
/// its position along that normal, `half` half its side length.
fn square_loop(axis: usize, offset: f64, half: f64) -> [[f64; 3]; 5] {
    let (u, v) = match axis {
        0 => (1, 2),
        1 => (0, 2),
        _ => (0, 1),
    };
    let corners = [
        (half, half),
        (-half, half),
        (-half, -half),
        (half, -half),
        (half, half),
    ];
    corners.map(|(cu, cv)| {
        let mut point = [0.0; 3];
        point[axis] = offset;
        point[u] = cu;
        point[v] = cv;
        point
    })
}

/// Orthographic projection: yaw about Z, then pitch. Returns the screen
/// position and a depth that grows toward the viewer.
fn project(point: [f64; 3], yaw: f32, pitch: f32, scale: f32, center: Pos2) -> (Pos2, f32) {
    let (x, y, z) = (point[0] as f32, point[1] as f32, point[2] as f32);
    let (sin_yaw, cos_yaw) = yaw.sin_cos();
    let x1 = x * cos_yaw + y * sin_yaw;
    let y1 = -x * sin_yaw + y * cos_yaw;
    let (sin_pitch, cos_pitch) = pitch.sin_cos();
    let screen_y = z * cos_pitch - y1 * sin_pitch;
    let depth = z * sin_pitch + y1 * cos_pitch;
    (
        center + Vec2::new(x1 * scale, -screen_y * scale),
        depth * scale,
    )
}

fn draw_triad(painter: &egui::Painter, view: &CageView, rect: egui::Rect, scale: f32) {
    let origin = rect.right_bottom() + Vec2::new(-42.0, -30.0);
    let font = FontId::proportional(10.0);
    for axis in 0..3 {
        let mut tip = [0.0; 3];
        tip[axis] = 1.0;
        let (end, _) = project(tip, view.yaw, view.pitch, scale, origin);
        painter.line_segment([origin, end], Stroke::new(1.5, AXIS_COLOR[axis]));
        painter.text(
            end,
            Align2::CENTER_CENTER,
            ["X", "Y", "Z"][axis],
            font.clone(),
            AXIS_COLOR[axis],
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn square_loop_is_closed_and_flat_on_its_normal() {
        for axis in 0..3 {
            let points = square_loop(axis, 0.25, 1.0);
            assert_eq!(points[0], points[4], "loop must close");
            for point in points {
                assert_eq!(point[axis], 0.25, "loop must stay on its normal offset");
            }
        }
    }

    #[test]
    fn projection_keeps_the_origin_at_the_view_centre() {
        let center = Pos2::new(50.0, 50.0);
        let (pos, depth) = project([0.0; 3], 0.7, 0.45, 100.0, center);
        assert_eq!(pos, center);
        assert_eq!(depth, 0.0);
    }
}
