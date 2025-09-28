use crate::utils::{Matrix3f, Polar, Scalar, Vec3f};
use rand::prelude::Distribution;
use rand::thread_rng;
use rand_distr::StandardNormal;

/// Calculate the 3x3 covariance matrix in Cartesian coordinates
pub fn covariance_cart(polar: Polar) -> Matrix3f {
    // Polar covariance matrix
    const VAR_ANG: f32 = 1.31 / 100_000.0; // rad^2
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

/// Calculate the covariance matrix in cartesian space for a contact with the given polar
/*
pub fn covariance_cart(polar: Polar) -> Matrix3f {
    let r_meas = covariance_polar(polar.range);
    println!("r_meas: {:?}", r_meas);
    let j = j_from_polar(polar);
    println!("j: {:?}", j);
    let jr = j * r_meas;
    println!("jr: {:?}", jr);
    let r_cart = jr * j.transpose();
    println!("r_cart: {:?}", r_cart);
    r_cart + Matrix3f::identity() * 1e-6 // tiny regularize
} */

/// Calculate the radar's 3x3 covariance matrix for a polar coordinate
fn covariance_polar(r: Scalar) -> Matrix3f {
    // Found by experimentation see 'SW Radar Tests' in Google Sheets
    const VAR_ANG: Scalar = 1.31 / 100_000.0; // rad^2
    let d = r / 1000.0;
    let vr = 7.77 - 0.897 * d + 33.5 * d * d;
    Matrix3f::from_diagonal(&Vec3f::new(vr, VAR_ANG, VAR_ANG))
}

/// Calculate the jacobian of the polar covariance matrix
/*fn j_from_polar(polar: Polar) -> Matrix3f {
    let r = polar.range;
    let (ca, sa) = (polar.azimuth.cos(), polar.azimuth.sin());
    let (ce, se) = (polar.elevation.cos(), polar.elevation.sin());
    Matrix3f::from([
        [ca * ce, -r * sa * ce, -r * ca * se],
        [se, 0.0, r * ce],
        [sa * ce, r * ca * ce, -r * sa * se],
    ])
}*/

fn j_from_polar(polar: Polar) -> Matrix3f {
    let r = polar.range;
    let (ca, sa) = (polar.azimuth.cos(), polar.azimuth.sin());
    let (ce, se) = (polar.elevation.cos(), polar.elevation.sin());

    Matrix3f::from([
        [sa * ce, r * ca * ce, -r * sa * se], // d(x)/dr, d(x)/daz, d(x)/del
        [se, 0.0, r * ce],                    // d(y)/dr, d(y)/daz, d(y)/del
        [ca * ce, -r * sa * ce, -r * ca * se], // d(z)/dr, d(z)/daz, d(z)/del
    ])
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
