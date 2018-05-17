use nalgebra;
use nalgebra::{MatrixSlice3x1, Vector6};

mod spatial_transform;

pub type Vector3 = nalgebra::Vector3<f32>;
pub type Vector4 = nalgebra::Vector4<f32>;
pub type SpatialVector = nalgebra::Vector6<f32>;
pub type Quaternion = nalgebra::Quaternion<f32>;
pub type UnitQuaternion = nalgebra::UnitQuaternion<f32>;
pub type MotionSubspace = nalgebra::DMatrix<f32>;
pub type Matrix3x3 = nalgebra::Matrix3<f32>;
pub type Matrix6x6 = nalgebra::Matrix6<f32>;

pub use self::spatial_transform::SpatialTransform;

pub trait SpatialVectorHelper {
    fn from_parts(m: Vector3, mO: Vector3) -> SpatialVector;
}

impl SpatialVectorHelper for SpatialVector {
    fn from_parts(m: Vector3, mO: Vector3) -> SpatialVector {
        SpatialVector::new(
            m.x, m.y, m.z,
            mO.x, mO.y, mO.z
        )
    }
}

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

pub fn r_cross(r: Vector3) -> Matrix3x3 {
    Matrix3x3::new(
         0.0, -r.z,  r.y,
         r.z,  0.0, -r.x,
        -r.y,  r.x,  0.0
    )
}

pub fn rbda_4_12(unit_q: UnitQuaternion) -> Matrix3x3 {
    let q = unit_q.quaternion();

    let p0 = q.coords.x;
    let p1 = q.coords.y;
    let p2 = q.coords.z;
    let p3 = q.coords.w;
    let p0sqr = p0.powf(2.0);
    let p1sqr = p1.powf(2.0);
    let p2sqr = p2.powf(2.0);
    let p3sqr = p3.powf(2.0);

    Matrix3x3::new(
        p0sqr + p1sqr - 0.5, p1 * p2 + p0 * p3, p1 * p3 - p0 * p2,
        p1 * p2 - p0 * p3, p0sqr + p2sqr - 0.5, p2 * p3 + p0 * p1,
        p1 * p3 + p0 * p2, p2 * p3 - p0 * p1, p0sqr + p3sqr - 0.5
    )
}
