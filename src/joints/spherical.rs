use super::Joint;
use math::{UnitQuaternion, MotionSubspace, Matrix6x6};

pub struct SphericalJoint {
    motion_subspace: MotionSubspace,
    s_to_p: Matrix6x6,
    p_to_s: Matrix6x6,
}

impl SphericalJoint {
    pub fn new(q: UnitQuaternion) -> SphericalJoint {
        let e = 
    }
}

impl Joint for SphericalJoint {
    fn motion_subspace(&self) -> &MotionSubspace {
        &self.motion_subspace
    }

    fn s_to_p(&self) -> &Matrix6x6 { 
        &self.s_to_p
    }

    fn p_to_s(&self) -> &Matrix6x6 {
        &self.p_to_s
    }
}

fn rbda_4_12(q: UnitQuaternion) -> Matrix6x6 {
    
}