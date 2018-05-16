mod revolute;
mod spherical;
mod six_dof;

pub use self::revolute::{RevoluteJoint, RevoluteAxis};
pub use self::spherical::SphericalJoint;
pub use self::six_dof::SixDOFJoint;

use math::{Vector3};
use math::{MotionSubspace, Matrix3x3, Matrix6x6};

pub trait Joint {
    fn motion_subspace(&self) -> &MotionSubspace;

    fn s_to_p(&self) -> &Matrix6x6;

    fn p_to_s(&self) -> &Matrix6x6;
}

// Joint functions

// TODO: Fix this horrible hack and implement some way to set blocks on a Matrix6x6
fn r_cross(r: Vector3) -> Matrix3x3 {
    Matrix3x3::new(
         0.0, -r.z,  r.y,
         r.z,  0.0, -r.x,
        -r.y,  r.x,  0.0
    )
}