use nalgebra::{Matrix3, Vector2, Vector3};

pub type Scalar = f32;
pub type Vec2f = Vector2<Scalar>;
pub type Vec3f = Vector3<Scalar>;
pub type Matrix3f = Matrix3<Scalar>;

#[derive(Copy, Clone, PartialEq)]
pub struct Polar {
    pub range: Scalar,
    pub azimuth: Scalar, // rotation around the y axis
    pub elevation: Scalar,
}

impl Polar {
    pub fn new(range: Scalar, azimuth: Scalar, elevation: Scalar) -> Self {
        Self {
            range,
            azimuth,
            elevation,
        }
    }

    pub fn from_vector(v: Vec3f) -> Self {
        let range = v.magnitude();
        Polar {
            range,
            azimuth: Scalar::atan2(v.x, v.z), // bearing around Y axis,
            elevation: (v.y / range).asin(),
        }
    }
}
