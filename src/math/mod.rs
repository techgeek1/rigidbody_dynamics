use nalgebra;

mod spatial_vector;


pub type Vector3 = nalgebra::Vector3<f32>;
pub type SpatialVector = nalgebra::Vector6<f32>;
pub type UnitQuaternion = nalgebra::UnitQuaternion<f32>;

pub type MotionSubspace = nalgebra::DMatrix<f32>;

pub type Matrix3x3 = nalgebra::Matrix3<f32>;
pub type Matrix6x6 = nalgebra::Matrix6<f32>;

pub use self::spatial_vector::{SpatialVectorF, SpatialVectorM};

pub fn make_block_matrix(b00: &Matrix3x3, b01: &Matrix3x3, b10: &Matrix3x3, b11: &Matrix3x3) -> Matrix6x6 {
    let c0 = Vector<f32, U6>::new(b00.m)

    Matrix6x6::from_columns(;
}