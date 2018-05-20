#![allow(dead_code)]

extern crate nalgebra;

mod math;
mod joints;
mod body;

use math::*;
use body::*;

pub struct SystemModel {
    body_count: u32,
    bodies: Vec<Body>
}

impl SystemModel {
    pub fn add_body(&mut self, body: Body, parent: i32, position: Vector3, rotation: Quaternion, velocity: Vector3, angular_velocity: Vector3) {
        let q = SystemModel::world_to_body_q(&body, &position, &rotation);
        let qd = SystemModel::world_to_body_qd(&body, &velocity, &angular_velocity);

        let mut body = body;
        body.parent = parent;
        body.q = q;
        body.qd = qd;

        self.bodies.push(body);
        self.body_count += 1;
    }

    pub fn set_body(&mut self, body_index: usize, position: Vector3, rotation: Quaternion, velocity: Vector3, angular_velocity: Vector3) {
        let body = &mut self.bodies[body_index];
        let q = SystemModel::world_to_body_q(body, &position, &rotation);
        let qd = SystemModel::world_to_body_qd(body, &velocity, &angular_velocity);
        
        body.q = q;
        body.qd = qd;
    }

    pub fn set_bodies(&mut self, positions: &[Vector3], rotations: &[Quaternion], velocities: &[Vector3], angular_velocity: &[Vector3]) {
        assert!((positions.len() == rotations.len()) && (velocities.len() == angular_velocity.len()));

        for i in 0..self.body_count {
            let index = i as usize;
            let body = &mut self.bodies[index];
            let q = SystemModel::world_to_body_q(body, &positions[index], &rotations[index]);
            let qd = SystemModel::world_to_body_qd(body, &velocities[index], &angular_velocity[index]);

            body.q = q;
            body.qd = qd;
        }
    }

    pub fn reset_model(&mut self) {
        self.body_count = 0;
        self.bodies.clear();
    }

    pub fn inverse_dynamics(&self, q: &[SpatialVector], qd: &[SpatialVector], qdd: &[SpatialVector], fext: &[SpatialVector]) -> &[SpatialVector] {
        
        unimplemented!()
    }

    fn transforms<'a>(&self) -> &'a [Matrix6x6] {
        /*for body in self.bodies.iter() {
            let body_xform = if body.parent == -1 {
                 body.joint.
            }
            else {

            };
        }*/

        unimplemented!();
    }

    fn world_to_body_q(body: &Body, position: &Vector3, rotation: &Quaternion) -> SpatialVector {
        unimplemented!()
    }

    fn world_to_body_qd(body: &Body, velocity: &Vector3, angular_velocity: &Vector3) -> SpatialVector {
        unimplemented!()
    }
}