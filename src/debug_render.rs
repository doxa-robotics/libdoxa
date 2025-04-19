use alloc::boxed::Box;
use alloc::vec::Vec;
use embedded_graphics::image::Image;
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyleBuilder, StyledDrawable};
use nalgebra::{Point2, Vector2};
use vexide::devices::display::{self};

use crate::path_planner::Path;

const FIELD_SIZE: f64 = 240.0;
const FIELD_ORIGIN: Point2<f64> = Point2::new(FIELD_SIZE / 2.0, FIELD_SIZE / 2.0);
/// Field scale multiplier to convert from mm to pixels
const FIELD_SCALE: f64 = FIELD_SIZE / (600.0 * 6.0);

pub struct DebugRenderMark {
    pub point: Point2<i32>,
    pub color: Rgb888,
    size: u32,
}

impl Default for DebugRenderMark {
    fn default() -> Self {
        Self {
            point: Point2::new(0, 0),
            color: Rgb888::new(255, 0, 0),
            size: 2,
        }
    }
}

trait Point2Ext {
    fn to_point(self) -> Point;
}

impl Point2Ext for Point2<f64> {
    fn to_point(self) -> Point {
        Point::new(self.x as i32, self.y as i32)
    }
}
impl Point2Ext for Point2<i32> {
    fn to_point(self) -> Point {
        Point::new(self.x, self.y)
    }
}

pub struct DebugRender {
    display: vexide_embedded_graphics::DisplayDriver,
    field_bmp:
        tinybmp::Bmp<'static, <vexide_embedded_graphics::DisplayDriver as DrawTarget>::Color>,

    pub paths: Vec<Box<dyn Path>>,
    pub marks: Vec<DebugRenderMark>,
}

impl DebugRender {
    pub fn new(display: display::Display) -> Self {
        Self {
            display: vexide_embedded_graphics::DisplayDriver::new(display),
            field_bmp: tinybmp::Bmp::from_slice(include_bytes!("../assets/field.bmp")).unwrap(),

            paths: Vec::new(),
            marks: Vec::new(),
        }
    }

    /// Renders the field and overlays the paths and marks
    /// This function should be called in a loop to update the display
    pub fn render(&mut self) {
        self.display.clear(Rgb888::BLACK).unwrap();

        let image = Image::with_center(&self.field_bmp, FIELD_ORIGIN.to_point());
        image.draw(&mut self.display).unwrap();

        // Draw the paths
        for path in &self.paths {
            let mut last_point = path.evaluate(0.0);
            let dt = 0.01;
            let mut t = 0.0;
            let style = PrimitiveStyleBuilder::new()
                .stroke_color(Rgb888::new(255, 0, 0))
                .stroke_width(2)
                .build();
            while t <= 1.0 {
                let current_point = path.evaluate(t);
                let line = Line::new(
                    (FIELD_ORIGIN + (Vector2::from(last_point) * FIELD_SCALE)).to_point(),
                    (FIELD_ORIGIN + (Vector2::from(current_point) * FIELD_SCALE)).to_point(),
                );
                line.draw_styled(&style, &mut self.display).unwrap();
                last_point = current_point;
                t += dt;
            }
        }

        // Draw the marks
        for mark in &self.marks {
            let circle = Circle::with_center(
                Point {
                    x: mark.point.x,
                    y: mark.point.y,
                },
                mark.size,
            )
            .into_styled(PrimitiveStyleBuilder::new().fill_color(mark.color).build());
            circle.draw(&mut self.display).unwrap();
        }

        self.display.render();
    }
}
