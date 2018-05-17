extern crate nalgebra;

mod math;
mod joints;
mod body;

use math::*;
use joints::*;
use body::*;

pub struct SystemModel {
    body_count: u32,
    bodies: Vec<Body>
}

impl SystemModel {
    pub fn transforms<'a>(&self) -> &'a [Matrix6x6] {
        /*for body in self.bodies.iter() {
            let body_xform = if body.parent == -1 {
                 body.joint.
            }
            else {

            };
        }*/

        unimplemented!();
    }
}

fn inverse_dynamics(model: &SystemModel, q: &[SpatialVector], qd: &[SpatialVector], qdd: &[SpatialVector], fext: &[SpatialVector], tau: &mut &[SpatialVector]) {

}