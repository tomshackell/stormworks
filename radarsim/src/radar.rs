use crate::utils::{angular_diff, Matrix3f, Polar, Scalar, Vec3f};
use rand::prelude::Distribution;
use rand::thread_rng;
use rand_distr::num_traits::FloatConst;
use rand_distr::StandardNormal;

const USE_CART_COVARIANCE: bool = true;

pub struct Radar {
    fov_x: Scalar,
    fov_y: Scalar,
    max_range: Scalar,
    sweep_time: Scalar,
    sweep_angle: Scalar,
}

impl Radar {
    pub fn new(fov_x: Scalar, fov_y: Scalar, max_range: Scalar, sweep_time: Scalar) -> Self {
        Self {
            fov_x,
            fov_y,
            max_range,
            sweep_time,
            sweep_angle: 0.0,
        }
    }

    /// Convert Stormworks radar FOVs into radians
    pub fn fov_to_rad(v: Scalar) -> Scalar {
        v * Scalar::PI()
    }

    pub fn step(&mut self, dt: Scalar) {
        let sweep_rate = 2.0 * Scalar::PI() / self.sweep_time;
        self.sweep_angle += dt * sweep_rate;
    }

    /// Attempt to use the radar to detect a contact at the given position, returns the detected
    /// position (perturbed by radar noise) or `None` if not detected (out of radar FOV / range).
    pub fn try_detect(&self, posn: Vec3f, rng: &mut impl rand::Rng) -> Option<Vec3f> {
        let polar = Polar::from_vector(posn);
        let az_diff = angular_diff(polar.azimuth, self.sweep_angle);
        let el_diff = angular_diff(polar.elevation, 0.0);
        if polar.range <= self.max_range
            && az_diff.abs() <= self.fov_x
            && el_diff.abs() <= self.fov_y
        {
            let r_cart = if USE_CART_COVARIANCE {
                covariance_cart_direct(polar.range)
            } else {
                covariance_cart(polar)
            };
            Some(perturb_position(r_cart, posn, rng))
        } else {
            None
        }
    }
}

pub fn covariance_cart_direct(dist: Scalar) -> Matrix3f {
    // Direct cartesian matrix, see 'SW Radar Test:Cart Covariance' in Google Sheets
    let d2 = (dist / 1000.0) * (dist / 1000.0);
    let eps = 1e-6; // diagonal cannot have any values <= 0
    let r1 = (13.1 * d2 - 2.02).max(eps);
    let r5 = (13.2 * d2 - 0.385).max(eps);
    let r9 = (33.4 * d2 - 14.9).max(eps);
    Matrix3f::from_diagonal(&Vec3f::new(r1, r5, r9))
}

/// Calculate the 3x3 covariance matrix in Cartesian coordinates
pub fn covariance_cart(polar: Polar) -> Matrix3f {
    // Polar covariance matrix
    // Measured from Stormworks, see 'SW Radar Test' in Google Sheets.
    const VAR_ANG: Scalar = 1.31 / 100_000.0; // rad^2
    let r = polar.range;
    let d = r / 1000.0;
    let vr = 7.77 - 0.897 * d + 33.5 * d * d;
    let cov_polar = Matrix3f::from_diagonal(&Vec3f::new(vr, VAR_ANG, VAR_ANG));

    // Jacobian matrix J for Stormworks convention (x=right, y=up, z=forward)
    let sa = polar.azimuth.sin();
    let ca = polar.azimuth.cos();
    let se = polar.elevation.sin();
    let ce = polar.elevation.cos();

    #[rustfmt::skip]
    let jacobian = Matrix3f::new(
        sa * ce,   r * ca * ce,  -r * sa * se,
        se,        0.0,           r * ce,
        ca * ce,  -r * sa * ce,  -r * ca * se,
    );

    // Cartesian covariance matrix
    jacobian * cov_polar * jacobian.transpose()
}

/// Perturb a radar position using the given cartesian covariance matrix
pub fn perturb_position(r_cart: Matrix3f, mean_pos: Vec3f, rng: &mut impl rand::Rng) -> Vec3f {
    // Cholesky factorization
    let chol = r_cart.cholesky().expect("R_cart not SPD");
    let l = chol.l(); // lower triangular factor

    // standard normal vector
    let z = Vec3f::new(
        StandardNormal.sample(rng),
        StandardNormal.sample(rng),
        StandardNormal.sample(rng),
    );

    // correlated noise
    mean_pos + l * z
}

pub fn sample_and_test(r: Scalar, az: Scalar, el: Scalar, n_samples: usize) {
    let r_cart = covariance_cart(Polar::new(r, az, el));
    println!("R_cart =\n{}", r_cart);
    // Cholesky (lower)
    let chol = r_cart.cholesky().expect("not SPD");
    let l = chol.l();

    let mut rng = thread_rng();
    let mut mean = Vec3f::zeros();
    let mut cov = Matrix3f::zeros();

    for _ in 0..n_samples {
        let z = Vec3f::new(
            StandardNormal.sample(&mut rng),
            StandardNormal.sample(&mut rng),
            StandardNormal.sample(&mut rng),
        );
        let delta = l * z; // correlated noise
        mean += delta;
        cov += delta * delta.transpose();
    }
    mean /= n_samples as Scalar;
    cov = cov / (n_samples as Scalar) - mean * mean.transpose();

    // print expected stds vs empirical
    let expected_std = Vec3f::new(
        r_cart[(0, 0)].sqrt(),
        r_cart[(1, 1)].sqrt(),
        r_cart[(2, 2)].sqrt(),
    );
    let empirical_std = Vec3f::new(cov[(0, 0)].sqrt(), cov[(1, 1)].sqrt(), cov[(2, 2)].sqrt());
    println!("Expected std (x,y,z) = {:?}", expected_std);
    println!("Empirical std (x,y,z) = {:?}", empirical_std);
    println!("Empirical mean (should be ~0) = {:?}", mean);
}
