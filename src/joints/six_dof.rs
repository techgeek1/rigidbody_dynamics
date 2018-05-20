use super::Joint;
use math::*;

pub struct SixDOFJoint {
    motion_subspace: MotionSubspace,
    s_to_p: Matrix6x6,
    p_to_s: Matrix6x6,
}

impl SixDOFJoint {
    pub fn new(pivot: Vector3, q: UnitQuaternion) -> SixDOFJoint {
        let motion_subspace = MotionSubspace::from_diagonal_element(6, 6, 1.0);
        let e = rbda_4_12(q);
        let neg_er = (-e * pivot).to_cross_matrix();
        let ps = Matrix6x6::from_block(&e, &neg_er, &Matrix3x3::zeros(), &Matrix3x3::repeat(1.0));// TODO: Double check this

        SixDOFJoint {
            motion_subspace: motion_subspace,
            s_to_p: ps.transpose(),
            p_to_s: ps
        }
    }
}

impl Joint for SixDOFJoint {
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