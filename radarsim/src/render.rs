use crate::utils::{Scalar, Vec2f, Vec3f};
use tiny_skia::{Color, FillRule, Paint, PathBuilder, Pixmap, Stroke, Transform};

pub struct Render {
    pixmap: Pixmap,
    translate: Vec2f,
    scale: Vec2f,
}

impl Render {
    pub fn new(width: u32, height: u32, top_left: Vec2f, bottom_right: Vec2f) -> Render {
        let mut pixmap = Pixmap::new(width, height).unwrap();
        pixmap.fill(Color::WHITE);
        let size = bottom_right - top_left;
        let scale = Vec2f::new((width as Scalar) / size.x, (height as Scalar) / size.y);

        let mut result = Self {
            pixmap,
            translate: Vec2f::new(-top_left.x, -top_left.y),
            scale,
        };

        // draw the origin axes (x and z)
        let grey = Color::from_rgba8(224, 224, 224, 255);
        result.draw_poly_line(
            &[
                Vec3f::new(top_left.x, 0.0, 0.0),
                Vec3f::new(bottom_right.x, 0.0, 0.0),
            ],
            1.0,
            grey,
        );
        result.draw_poly_line(
            &[
                Vec3f::new(0.0, 0.0, top_left.y),
                Vec3f::new(0.0, 0.0, bottom_right.y),
            ],
            1.0,
            grey,
        );
        result
    }

    pub fn draw_poly_line<'a, I: IntoIterator<Item = &'a Vec3f>>(
        &mut self,
        points: I,
        width: f32,
        color: Color,
    ) {
        let mut pb = PathBuilder::new();

        for (i, p) in points.into_iter().enumerate() {
            let v = self.transform(*p);
            if i == 0 {
                pb.move_to(v.x, v.y);
            } else {
                pb.line_to(v.x, v.y);
            }
        }
        let path = pb.finish().unwrap();
        let stroke = Stroke {
            width,
            ..Stroke::default()
        };
        let mut paint = Paint::default();
        paint.set_color(color);
        let xform = Transform::identity();
        self.pixmap.stroke_path(&path, &paint, &stroke, xform, None);
    }

    pub(crate) fn draw_dot(&mut self, point: Vec3f, radius: Scalar, color: Color) {
        let p = self.transform(point);
        let mut paint = Paint::default();
        paint.set_color(color);
        let path = PathBuilder::from_circle(p.x, p.y, radius).unwrap();
        let fill = FillRule::Winding;
        let xform = Transform::identity();
        self.pixmap.fill_path(&path, &paint, fill, xform, None);
    }

    fn transform(&self, point: Vec3f) -> Vec2f {
        Vec2f::new(
            (point.x + self.translate.x) * self.scale.x,
            (point.z + self.translate.y) * self.scale.y,
        )
    }

    pub fn save(&self, filename: &str) {
        self.pixmap.save_png(filename).unwrap();
    }
}
