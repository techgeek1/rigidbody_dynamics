use nalgebra;

mod spatial_vector;

pub type Vector3 = nalgebra::Vector3<f32>;
pub type SpatialVector = nalgebra::Vector6<f32>;
pub type MotionSubspace = nalgebra::DMatrix<f32>;
pub type Matrix3x3 = nalgebra::Matrix3<f32>;
pub type Matrix6x6 = nalgebra::Matrix6<f32>;
pub use self::spatial_vector::{SpatialVectorF, SpatialVectorM};