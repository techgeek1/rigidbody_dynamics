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
            RevoluteAxis::X => rotx(0.0),
            RevoluteAxis::Y => roty(0.0),
            RevoluteAxis::Z => rotz(0.0)   
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

fn rotx(theta: f32) -> Matrix6x6 {
    let c = theta.cos();
    let s = theta.sin();

    Matrix6x6::new(
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0,   c,   s, 0.0, 0.0, 0.0,
        0.0,  -c,   c, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,   c,   s,
        0.0, 0.0, 0.0, 0.0,  -c,   c
    )
}

fn roty(theta: f32) -> Matrix6x6 {
    let c = theta.cos();
    let s = theta.sin();

    Matrix6x6::new(
          c, 0.0,  -s, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
          s, 0.0,   c, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,   c, 0.0,  -s,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0,   s, 0.0,   c
    )
}

fn rotz(theta: f32) -> Matrix6x6 {
    let c = theta.cos();
    let s = theta.sin();

    Matrix6x6::new(
          c,   s, 0.0, 0.0, 0.0, 0.0,
         -s,   c, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,   c,   s, 0.0,
        0.0, 0.0, 0.0,  -s,   c, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    )
}