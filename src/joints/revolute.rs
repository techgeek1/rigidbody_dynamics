use super::Joint;
use math::{MotionSubspace, Matrix6x6, Matrix3x3};

pub enum RevoluteAxis {
    X,
    Y,
    Z
}

pub struct RevoluteJoint {
    axis: RevoluteAxis,
    motion_subspace: MotionSubspace
    s_to_p: Matrix6x6,
    p_to_s: Matrix6x6,
}

impl RevoluteJoint {
    fn new(axis: RevoluteAxis) -> RevoluteJoint {
        //let e = rx(0.0);

        RevoluteJoint {
            axis: axis,
            motion_subspace: MotionSubspace::from_element(6, 1, 0.0)
        }
    }
}

impl Joint for RevoluteJoint {
    
}