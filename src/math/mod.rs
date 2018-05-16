use nalgebra;
use nalgebra::{MatrixSlice3x1, Vector6};

mod spatial_vector;


pub type Vector3 = nalgebra::Vector3<f32>;
pub type SpatialVector = nalgebra::Vector6<f32>;
pub type UnitQuaternion = nalgebra::UnitQuaternion<f32>;

pub type MotionSubspace = nalgebra::DMatrix<f32>;

pub type Matrix3x3 = nalgebra::Matrix3<f32>;
pub type Matrix6x6 = nalgebra::Matrix6<f32>;

pub use self::spatial_vector::{SpatialVectorF, SpatialVectorM};

pub fn make_block_matrix(b00: &Matrix3x3, b01: &Matrix3x3, b10: &Matrix3x3, b11: &Matrix3x3) -> Matrix6x6 {
    let c0 = concat_columns(&b00.fixed_slice(0, 0), &b10.fixed_slice(0, 0));
    let c1 = concat_columns(&b00.fixed_slice(0, 1), &b10.fixed_slice(0, 1));
    let c2 = concat_columns(&b00.fixed_slice(0, 2), &b10.fixed_slice(0, 2));
    let c3 = concat_columns(&b01.fixed_slice(0, 0), &b11.fixed_slice(0, 0));
    let c4 = concat_columns(&b01.fixed_slice(0, 1), &b11.fixed_slice(0, 1));
    let c5 = concat_columns(&b01.fixed_slice(0, 2), &b11.fixed_slice(0, 2));

    Matrix6x6::from_columns(&[c0, c1, c2, c3, c4, c5])
}

fn concat_columns(c0: &MatrixSlice3x1<f32>, c1: &MatrixSlice3x1<f32>) -> Vector6<f32> {
    let mut result = Vec::<f32>::with_capacity(6);
    result.extend(c0.iter());
    result.extend(c1.iter());

    Vector6::from_iterator(result.into_iter())
}