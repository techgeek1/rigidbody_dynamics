mod revolute;
mod spherical;
mod six_dof;

pub use self::revolute::Revolute;
pub use self::spherical::Spherical;
pub use self::six_dof::SixDof;

use math::{Vector3};
use math::{MotionSubspace, Matrix3x3, Matrix6x6};

pub trait Joint {
    fn s_to_p() -> Matrix6x6;

    fn p_to_s() -> Matrix6x6;

    fn motion_subspace() -> MotionSubspace;
}

fn rx(theta: f32) -> Matrix3x3 {
    let cos = theta.cos();
    let sin = theta.sin();

    Matrix3x3::new(
        1.0, 0.0, 0.0,
        0.0,   c,   s,
        0.0,  -c,   c
    )
}

fn ry(theta: f32) -> Matrix3x3 {
    let cos = theta.cos();
    let sin = theta.sin();

    Matrix3x3::new(
          c, 0.0,  -s,
        0.0, 1.0, 0.0,
          s, 0.0,   c
    )
}

fn rz(theta: f32) -> Matrix3x3 {
    let cos = theta.cos();
    let sin = theta.sin();

    Matrix3x3::new(
          c,   s, 0.0,
         -s,   c, 0.0,
        0.0, 0.0, 0.0
    )
}

fn r_cross(r: Vector3) -> Matrix3x3 {
    Matrix3x3::new(
         0.0, -r.z,  r.y,
         r.z,  0.0, -r.x,
        -r.y,  r.x,  0.0
    )
}