use math::*;

pub struct SpatialTransform {
    e: Matrix3x3,
    r: Vector3
}

impl SpatialTransform {
    pub fn new(e: Matrix3x3, r: Vector3) -> SpatialTransform {
        SpatialTransform {
            e: e,
            r: r
        }
    }

    pub fn apply(&self, v: SpatialVector) -> SpatialVector {
        let v_rxw = Vector3::new(
            v[3] - self.r[1] * v[2] + self.r[2] * v[1],
            v[4] - self.r[2] * v[0] + self.r[0] * v[2],
            v[5] - self.r[0] * v[1] + self.r[1] * v[0]
        );

        SpatialVector::new(
            self.e[(0, 0)] * v[0] + self.e[(0, 1)] * v[1] + self.e[(0, 2)] * v[2],
            self.e[(1, 0)] * v[0] + self.e[(1, 1)] * v[1] + self.e[(1, 2)] * v[2],
            self.e[(2, 0)] * v[0] + self.e[(2, 1)] * v[1] + self.e[(2, 2)] * v[2],
            self.e[(0, 0)] * v_rxw[0] + self.e[(0, 1)] * v_rxw[1] + self.e[(0, 2)] * v_rxw[2],
            self.e[(1, 0)] * v_rxw[0] + self.e[(1, 1)] * v_rxw[1] + self.e[(1, 2)] * v_rxw[2],
            self.e[(2, 0)] * v_rxw[0] + self.e[(2, 1)] * v_rxw[1] + self.e[(2, 2)] * v_rxw[2]
        )
    }

    pub fn apply_transpose(&self, f: SpatialVector) -> SpatialVector {
        let etf = Vector3::new(
            self.e[(0, 0)] * f[3] + self.e[(1, 0)] * f[4] + self.e[(2, 0)] * f[5],
            self.e[(0, 1)] * f[3] + self.e[(1, 1)] * f[4] + self.e[(2, 1)] * f[5],
            self.e[(0, 2)] * f[3] + self.e[(1, 2)] * f[4] + self.e[(2, 2)] * f[5]
        );

        SpatialVector::new(
            self.e[(0, 0)] * f[0] + self.e[(1, 0)] * f[1] + self.e[(2, 0)] * f[2] - self.r[2] * etf[1] + self.r[1] * etf[2],
            self.e[(0, 1)] * f[0] + self.e[(1, 1)] * f[1] + self.e[(2, 1)] * f[2] + self.r[2] * etf[0] - self.r[0] * etf[2],
            self.e[(0, 2)] * f[0] + self.e[(1, 2)] * f[1] + self.e[(2, 2)] * f[2] - self.r[1] * etf[0] + self.r[0] * etf[1],
            etf[0],
            etf[1],
            etf[2]
        )
    }
}