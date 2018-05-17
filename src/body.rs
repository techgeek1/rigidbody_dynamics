
use math::*;
use joints::*;

pub struct Alignment {
    cb_0b: Matrix6x6,
    cb_b0: Matrix6x6
}

impl Alignment {
    pub fn from_revolute(pivot: Vector3, normal: Vector3) -> Alignment {
        unimplemented!();
    }

    pub fn from_spherical(pivot: Vector3) -> Alignment {
        unimplemented!();
    }

    pub fn from_six_dof() -> Alignment {
        unimplemented!();
    }
}

pub struct Body {
    parent: i32,
    mass: f32,
    inertia: Matrix6x6,
    joint: Box<Joint>,
    qd: SpatialVector
}

impl Body {
    pub fn new(parent: i32, mass: f32, inertia: Matrix6x6, joint: Box<Joint>) -> Body {
        Body {
            parent: parent,
            mass: mass,
            inertia: inertia,
            joint: joint,
            qd: SpatialVector::from_parts(Vector3::zeros(), Vector3::zeros())
        }
    }
}