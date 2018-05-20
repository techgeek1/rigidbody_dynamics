use std::f32;
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

// Traits
pub trait CrossMatrixConvertible {
    fn to_cross_matrix(&self) -> Matrix3x3;

    fn cross_product_matrix(&self) -> Matrix3x3;
}

pub trait RotationMatrixConvertible {
    fn to_rotation_matrix(&self) -> Matrix3x3;
}

pub trait SpatialVectorExtensions {
    fn from_parts(m: Vector3, m_o: Vector3) -> SpatialVector;
}

pub trait Matrix6x6Extensions {
    fn from_block(b00: &Matrix3x3, b01: &Matrix3x3, b10: &Matrix3x3, b11: &Matrix3x3) -> Matrix6x6;

    fn from_rt(e: &Matrix3x3, r: &Vector3) -> Matrix6x6;
}

impl CrossMatrixConvertible for Vector3 {
    fn to_cross_matrix(&self) -> Matrix3x3 {
        Matrix3x3::new(
                0.0, -self.z,  self.y,
             self.z,     0.0, -self.x,
            -self.y,  self.x,     0.0
        )
    }

    fn cross_product_matrix(&self) -> Matrix3x3 {
        Matrix3x3::new(
            0.0, -self.z, self.y,
            self.z, 0.0, -self.x,
            -self.y, self.x, 0.0
        )
    }
}

impl RotationMatrixConvertible for Vector3 {
    fn to_rotation_matrix(&self) -> Matrix3x3 {
        let z = self.normalize();
        let x = self.cross(&Vector3::new(0.0, 1.0, 0.0)).normalize();
        let y = z.cross(&x);

        return Matrix3x3::new(
            x.x, x.y, x.z,
            y.x, y.y, y.z,
            z.x, z.y, z.z
        )
    }
}

impl SpatialVectorExtensions for SpatialVector {
    fn from_parts(m: Vector3, m_o: Vector3) -> SpatialVector {
        SpatialVector::new(
            m.x, m.y, m.z,
            m_o.x, m_o.y, m_o.z
        )
    }
}

impl RotationMatrixConvertible for Quaternion {
    fn to_rotation_matrix(&self) -> Matrix3x3 {
        let xx = self.coords.x * self.coords.x;
        let xy = self.coords.x * self.coords.y;
        let xz = self.coords.x * self.coords.z;
        let xw = self.coords.x * self.coords.w;

        let yy = self.coords.y * self.coords.y;
        let yz = self.coords.y * self.coords.z;
        let yw = self.coords.y * self.coords.w;

        let zz = self.coords.z * self.coords.z;
        let zw = self.coords.z * self.coords.w;

        Matrix3x3::new(
            1.0 - 2.0 * (yy + zz), 2.0 * (xy - zw), 2.0 * (xz + yw),
            2.0 * (xy + zw), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - xw),
            2.0 * (xz - yw), 2.0 * (yz + xw), 1.0 - 2.0 * (zz + yy)
        )
    }
}

impl Matrix6x6Extensions for Matrix6x6 {
    fn from_block(b00: &Matrix3x3, b01: &Matrix3x3, b10: &Matrix3x3, b11: &Matrix3x3) -> Matrix6x6 {
        let c0 = concat_columns(&b00.fixed_slice(0, 0), &b10.fixed_slice(0, 0));
        let c1 = concat_columns(&b00.fixed_slice(0, 1), &b10.fixed_slice(0, 1));
        let c2 = concat_columns(&b00.fixed_slice(0, 2), &b10.fixed_slice(0, 2));
        let c3 = concat_columns(&b01.fixed_slice(0, 0), &b11.fixed_slice(0, 0));
        let c4 = concat_columns(&b01.fixed_slice(0, 1), &b11.fixed_slice(0, 1));
        let c5 = concat_columns(&b01.fixed_slice(0, 2), &b11.fixed_slice(0, 2));

        Matrix6x6::from_columns(&[c0, c1, c2, c3, c4, c5])
    }

    fn from_rt(e: &Matrix3x3, r: &Vector3) -> Matrix6x6 { 
        Matrix6x6::from_block(
            e, &Matrix3x3::zeros(),
            &r.to_cross_matrix(), e
        )
    }
}

fn concat_columns(c0: &MatrixSlice3x1<f32>, c1: &MatrixSlice3x1<f32>) -> Vector6<f32> {
    let mut result = Vec::<f32>::with_capacity(6);
    result.extend(c0.iter());
    result.extend(c1.iter());

    Vector6::from_iterator(result.into_iter())
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

pub fn mci(mass: f32, center_of_mass: Vector3, inertia: Matrix3x3) -> Matrix6x6 {
    let c = center_of_mass.cross_product_matrix();
    let ct = c.transpose();

    Matrix6x6::from_block(
        &(inertia + mass * c * ct), &(mass * c),
        &(mass * ct), &(mass * Matrix3x3::identity())
    )
}
