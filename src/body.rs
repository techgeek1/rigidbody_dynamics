
use math::*;
use joints::*;

#[derive(Clone, Copy)]
pub struct Alignment {
    r: Matrix3x3,
    cb_0b: Matrix6x6,
    cb_b0: Matrix6x6
}

impl Alignment {
    pub fn from_revolute(pivot: Vector3, normal: Vector3) -> Alignment {
        let r = normal.to_rotation_matrix();

        Alignment {
            r: r,
            cb_0b: Matrix6x6::from_rt(&r, &(-r * pivot)),
            cb_b0: Matrix6x6::from_rt(&r.transpose(), &pivot)
        }
    }

    pub fn from_spherical(pivot: Vector3) -> Alignment {
        let identity = Matrix3x3::identity();

        Alignment {
            r: identity,
            cb_0b: Matrix6x6::from_rt(&identity, &-pivot),
            cb_b0: Matrix6x6::from_rt(&identity, &pivot)
        }
    }

    pub fn from_six_dof(center_of_mass: Vector3) -> Alignment {
        let identity = Matrix3x3::identity();

        Alignment {
            r: identity,
            cb_0b: Matrix6x6::from_rt(&identity, &-center_of_mass),
            cb_b0: Matrix6x6::from_rt(&identity, &center_of_mass)
        }
    }
}

pub struct Body {
    pub parent: i32,
    pub mass: f32,
    pub spatial_inertia: Matrix6x6,
    pub alignment: Alignment,
    pub joint: Box<Joint>,

    pub q: SpatialVector,
    pub qd: SpatialVector
}

impl Body {
    pub fn from_revolute(mass: f32, center_of_mass: Vector3, inertia: Matrix3x3, pivot: Vector3, normal: Vector3, axis: RevoluteAxis) -> Body {
        let alignment = Alignment::from_revolute(pivot, normal);
        let spatial_inertia = Body::spatial_inertia(mass, center_of_mass, inertia, Some(alignment));
        let joint = RevoluteJoint::new(Vector3::zeros(), axis);

        Body {
            parent: -1,
            mass: mass,
            spatial_inertia: spatial_inertia,
            alignment: alignment,
            joint: Box::new(joint),
            q: SpatialVector::zeros(),
            qd: SpatialVector::zeros()
        }
    }

    pub fn from_spherical(mass: f32, center_of_mass: Vector3, inertia: Matrix3x3, pivot: Vector3) -> Body {
        let alignment = Alignment::from_spherical(pivot);
        let spatial_inertia = Body::spatial_inertia(mass, center_of_mass, inertia, Some(alignment));
        let joint = SphericalJoint::new(UnitQuaternion::identity());// TODO: See about passing in the right rotation

        Body {
            parent: -1,
            mass: mass,
            spatial_inertia: spatial_inertia,
            alignment: alignment,
            joint: Box::new(joint),
            q: SpatialVector::zeros(),
            qd: SpatialVector::zeros()
        }
    }

    pub fn from_six_dof(mass: f32, center_of_mass: Vector3, inertia: Matrix3x3) -> Body {
        let alignment = Alignment::from_six_dof(center_of_mass);
        let spatial_inertia = Body::spatial_inertia(mass, center_of_mass, inertia, Some(alignment));
        let joint = SixDOFJoint::new(center_of_mass, UnitQuaternion::identity());// TODO: See about passing in the right rotation

        Body {
            parent: -1,
            mass: mass,
            spatial_inertia: spatial_inertia,
            alignment: alignment,
            joint: Box::new(joint),
            q: SpatialVector::zeros(),
            qd: SpatialVector::zeros()
        }
    }

    pub fn spatial_inertia(mass: f32, center_of_mass: Vector3, inertia: Matrix3x3, alignment: Option<Alignment>) -> Matrix6x6 {
        let mut center_of_mass = center_of_mass;
        let mut inertia = inertia;
        if let Some(a) = alignment {
            center_of_mass = a.r * center_of_mass;
            inertia = a.r * inertia * a.r.transpose();
        }

        mci(mass, center_of_mass, inertia)
    }
}