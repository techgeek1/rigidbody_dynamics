use super::Joint;
use math::{MotionSubspace, Matrix6x6};

pub enum RevoluteAxis {
    X,
    Y,
    Z
}

pub struct RevoluteJoint {
    axis: RevoluteAxis,
    motion_subspace: MotionSubspace,
    s_to_p: Matrix6x6,
    p_to_s: Matrix6x6,
}

impl RevoluteJoint {
    fn new(axis: RevoluteAxis) -> RevoluteJoint {
        let ps = match axis {
            RevoluteAxis::X => super::rotx(0.0),
            RevoluteAxis::Y => super::roty(0.0),
            RevoluteAxis::Z => super::rotz(0.0)   
        };

        RevoluteJoint {
            axis: axis,
            motion_subspace: MotionSubspace::from_element(6, 1, 0.0),
            s_to_p: ps.transpose(),
            p_to_s: ps
        }
    }
}

impl Joint for RevoluteJoint {
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