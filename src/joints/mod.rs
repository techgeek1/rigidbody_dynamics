mod revolute;
mod spherical;
mod six_dof;

pub use self::revolute::{RevoluteJoint, RevoluteAxis};
pub use self::spherical::SphericalJoint;
pub use self::six_dof::SixDOFJoint;

use math::{MotionSubspace, Matrix6x6};

pub trait Joint {
    fn motion_subspace(&self) -> &MotionSubspace;

    fn s_to_p(&self) -> &Matrix6x6;

    fn p_to_s(&self) -> &Matrix6x6;
}
