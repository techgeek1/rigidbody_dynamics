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

fn rbda_4_12(unit_q: UnitQuaternion) -> Matrix3x3 {
    let q = unit_q.quaternion();

    let p0 = q.coords.x;
    let p1 = q.coords.y;
    let p2 = q.coords.z;
    let p3 = q.coords.w;
    let p0sqr = p0.powf(2.0);
    let p1sqr = p1.powf(2.0);
    let p2sqr = p2.powf(2.0);
    let p3sqr = p3.powf(2.0);

    Matrix3x3::new(
        p0sqr + p1sqr - 0.5, p1 * p2 + p0 * p3, p1 * p3 - p0 * p2,
        p1 * p2 - p0 * p3, p0sqr + p2sqr - 0.5, p2 * p3 + p0 * p1,
        p1 * p3 + p0 * p2, p2 * p3 - p0 * p1, p0sqr + p3sqr - 0.5
    )
}