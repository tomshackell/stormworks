use crate::utils::{Scalar, Vec3f};
use nalgebra::{DMatrix, DVector, Matrix3};

pub struct Kalman<R> {
    x: DVector<Scalar>,
    p: DMatrix<Scalar>,
    q: DMatrix<Scalar>,
    h: DMatrix<Scalar>,
    i: DMatrix<Scalar>,
    r_function: R,
    last_t: Option<Scalar>,
}

impl<R> Kalman<R>
where
    R: Fn(Scalar) -> Matrix3<Scalar>,
{
    pub fn new(qv: Scalar, r_function: R) -> Self {
        #[rustfmt::skip]
        let q = DMatrix::from_row_slice(6, 6, &[
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,  qv, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,  qv, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0,  qv,
        ]);
        #[rustfmt::skip]
        let h = DMatrix::from_row_slice(3, 6, &[
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        ]);
        Self {
            x: DVector::zeros(6),
            p: DMatrix::from_diagonal_element(6, 6, 1000.0),
            q,
            h,
            r_function,
            i: DMatrix::identity(6, 6),
            last_t: None,
        }
    }

    pub fn update(&mut self, t: Scalar, z: Vec3f) -> bool {
        if let Some(last_t) = self.last_t {
            let dt = t - last_t;

            // predict
            let f = Self::state_transition_matrix(dt);
            self.x = &f * &self.x;
            self.p = &f * &self.p * f.transpose() + &self.q;

            // get measurement covariance R based on current distance from radar
            let (posn, _, _) = self.state().unwrap();
            let r = (self.r_function)(posn.magnitude());

            // update
            let y = z - &self.h * &self.x;
            let s = &self.h * &self.p * self.h.transpose() + &r;
            let s_inv = match s.try_inverse() {
                Some(s_inv) => s_inv,
                None => return false,
            };
            let k = &self.p * self.h.transpose() * &s_inv;
            self.x = &self.x + &k * y;
            self.p = (&self.i - &k * &self.h) * &self.p;
        } else {
            // we have an initial position
            self.x = DVector::from_row_slice(&[z.x, z.y, z.z, 0.0, 0.0, 0.0]);
        }
        self.last_t = Some(t);
        true
    }

    fn state_transition_matrix(dt: Scalar) -> DMatrix<Scalar> {
        #[rustfmt::skip]
        let f = DMatrix::from_row_slice(6, 6, &[
            1.0, 0.0, 0.0, dt, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, dt, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, dt,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ]);
        f
    }

    pub fn state(&self) -> Option<(Vec3f, Vec3f, Scalar)> {
        match self.last_t {
            Some(t) => {
                let pos = Vec3f::new(self.x[0], self.x[1], self.x[2]);
                let vel = Vec3f::new(self.x[3], self.x[4], self.x[5]);
                Some((pos, vel, t))
            }
            _ => None,
        }
    }
}
