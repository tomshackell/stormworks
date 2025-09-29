use nalgebra::{Matrix3, Vector2, Vector3};
use rand_distr::num_traits::FloatConst;

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

/// Convert Stormworks turns (1 = whole circle) into radians
pub fn turns_to_rad(turns: Scalar) -> Scalar {
    turns * 2.0 * Scalar::PI()
}

/// Calculate the angular difference between two angles (in radians), given the circular property
/// of angles. For example if `current=1.9*pi` and `target=0.1*pi`, then the angular difference
/// is `0.2*pi`.
pub fn angular_diff(current: Scalar, target: Scalar) -> Scalar {
    let pi2 = 2.0 * Scalar::PI();
    let diff = ((target % pi2) - (current % pi2) + pi2) % pi2;
    if diff > Scalar::PI() {
        diff - pi2
    } else {
        diff
    }
}
