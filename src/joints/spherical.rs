use super::Joint;
use math::*;

pub struct SphericalJoint {
    motion_subspace: MotionSubspace,
    s_to_p: Matrix6x6,
    p_to_s: Matrix6x6,
}

impl SphericalJoint {
    pub fn new(q: UnitQuaternion) -> SphericalJoint {
        let motion_subspace = MotionSubspace::from_row_slice(6, 3, 
            &[
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0
            ]
        );

        let e = rbda_4_12(q);
        let ps = make_block_matrix(&e, &Matrix3x3::zeros(), &Matrix3x3::zeros(), &e);

        SphericalJoint {
            motion_subspace: motion_subspace,
            s_to_p: ps.transpose(),
            p_to_s: ps
        }
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