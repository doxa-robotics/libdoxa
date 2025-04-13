use alloc::vec::Vec;
use alloc::{boxed::Box, vec};
use vexide::{
    devices::{
        display::{self, Rect},
        math::Point2,
    },
    prelude::Rgb,
};

use crate::path_planner::Path;

pub struct DebugRenderMark {
    pub x: i16,
    pub y: i16,
    pub color: vexide::devices::rgb::Rgb<u8>,
    size: u16,
}

impl Default for DebugRenderMark {
    fn default() -> Self {
        Self {
            x: 0,
            y: 0,
            color: vexide::devices::rgb::Rgb::new(255, 0, 0),
            size: 2,
        }
    }
}

pub struct DebugRender {
    display: display::Display,
    field_image: Vec<Rgb<u8>>,
    field_width: i16,
    field_height: i16,
    field_origin: Point2<i16>,
    field_scale: f32,

    pub paths: Vec<Box<dyn Path>>,
    pub marks: Vec<DebugRenderMark>,
}

impl DebugRender {
    pub fn new(display: display::Display) -> Self {
        // Load the field image from assets
        let mut bmp = zune_bmp::BmpDecoder::new(include_bytes!("../assets/field.bmp"));
        let field_image = bmp.decode().unwrap();
        let (width, height) = bmp.get_dimensions().unwrap();
        debug_assert!(field_image.len() == (width * height * 3));
        // Convert the BMP data to RGB format
        let field_image = field_image
            .chunks_exact(4)
            // Skip the first half of the image to get the bottom half
            .skip(width * height / 2)
            .map(|chunk| {
                // Format is RGBA, we need to convert it
                Rgb::new(128 + chunk[2] / 2, 128 + chunk[1] / 2, 128 + chunk[0] / 2)
            })
            .collect::<Vec<Rgb<u8>>>()
            // Duplicate each row to fix a decoding bug
            .chunks_exact(width)
            .flat_map(|row| {
                // Reverse the row to match the display orientation
                vec![row.to_vec(), row.to_vec()]
            })
            .flatten()
            .collect::<Vec<Rgb<u8>>>();
        log::info!("Field image loaded with dimensions: {}x{}", width, height);
        Self {
            display,
            field_image,
            field_width: width as i16,
            field_height: height as i16,
            field_origin: Point2 {
                x: width as i16 / 2,
                y: height as i16 / 2,
            },
            field_scale: width as f32 / (600.0 * 6.0),

            paths: Vec::new(),
            marks: Vec::new(),
        }
    }

    /// Renders the field and overlays the paths and marks
    /// This function should be called in a loop to update the display
    pub fn render(&mut self) {
        // Clear the display
        self.display.erase((0, 0, 0));

        // Draw the field image
        self.display.draw_buffer(
            Rect {
                start: Point2 { x: 0, y: 0 },
                end: Point2 {
                    x: self.field_width,
                    y: self.field_height,
                },
            },
            self.field_image.clone(), // fun, allocations
            self.field_width as i32,
        );

        // Draw the paths
        for path in &self.paths {
            let mut last_point = path.evaluate(0.0);
            let dt = 0.01;
            let mut t = 0.0;
            let color = Rgb::new(255, 0, 0);
            while t <= 1.0 {
                let current_point = path.evaluate(t);
                self.display.fill(
                    &display::Line::new(
                        Point2 {
                            x: (last_point.x * self.field_scale) as i16 + self.field_origin.x,
                            y: (last_point.y * self.field_scale) as i16 + self.field_origin.y,
                        },
                        Point2 {
                            x: (current_point.x * self.field_scale) as i16 + self.field_origin.x,
                            y: (current_point.y * self.field_scale) as i16 + self.field_origin.y,
                        },
                    ),
                    color,
                );
                last_point = current_point;
                t += dt;
            }
        }

        // Draw the marks
        for mark in &self.marks {
            self.display.fill(
                &display::Circle::new(
                    Point2 {
                        x: (mark.x as f32 * self.field_scale) as i16 + self.field_origin.x,
                        y: (mark.y as f32 * self.field_scale) as i16 + self.field_origin.y,
                    },
                    mark.size,
                ),
                mark.color,
            );
        }

        self.display.render();
    }

    pub fn render_extra(
        &mut self,
        func: impl FnOnce(&mut display::Display, &dyn Fn(Point2<i16>) -> Point2<i16>),
    ) {
        func(&mut self.display, &|point| Point2 {
            x: (point.x as f32 * self.field_scale) as i16 + self.field_origin.x,
            y: (point.y as f32 * self.field_scale) as i16 + self.field_origin.y,
        });
    }
}
