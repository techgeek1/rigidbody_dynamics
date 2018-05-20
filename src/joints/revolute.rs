use super::Joint;
use math::*;

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
    pub fn new(rotation: Vector3, axis: RevoluteAxis) -> RevoluteJoint {
        let angle = match axis {
            RevoluteAxis::X => rotation.x,
            RevoluteAxis::Y => rotation.y,
            RevoluteAxis::Z => rotation.z
        };

        let ps = match axis {
            RevoluteAxis::X => rotx(angle),
            RevoluteAxis::Y => roty(angle),
            RevoluteAxis::Z => rotz(angle)   
        };

        let motion_subspace = match axis {
            RevoluteAxis::X => MotionSubspace::from_column_slice(6, 1, &[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            RevoluteAxis::Y => MotionSubspace::from_column_slice(6, 1, &[0.0, 1.0, 0.0, 0.0, 0.0, 0.0]),
            RevoluteAxis::Z => MotionSubspace::from_column_slice(6, 1, &[0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        };

        RevoluteJoint {
            axis: axis,
            motion_subspace: motion_subspace,
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
    let e = Matrix3x3::new(
        1.0, 0.0, 0.0,
        0.0,   c,   s,
        0.0,  -c,   c
    );

    Matrix6x6::from_block(
        &e, &Matrix3x3::zeros(), 
        &Matrix3x3::zeros(), &e
    )
}

fn roty(theta: f32) -> Matrix6x6 {
    let c = theta.cos();
    let s = theta.sin();
    let e = Matrix3x3::new(
          c, 0.0,  -s,
        0.0, 1.0, 0.0,
        s, 0.0,   c
    );

    Matrix6x6::from_block(
        &e, &Matrix3x3::zeros(), 
        &Matrix3x3::zeros(), &e
    )
}

fn rotz(theta: f32) -> Matrix6x6 {
    let c = theta.cos();
    let s = theta.sin();
    let e = Matrix3x3::new(
          c,   s, 0.0,
        -s,   c, 0.0,
        0.0, 0.0, 0.0
    );

    Matrix6x6::from_block(
        &e, &Matrix3x3::zeros(), 
        &Matrix3x3::zeros(), &e
    )
}