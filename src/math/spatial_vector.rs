use super::{Vector3, SpatialVector};

pub struct SpatialVectorF {
    v: SpatialVector
}

impl SpatialVectorF {
    pub fn decompose(&self) -> (Vector3, Vector3) {
        (Vector3::new(self.v.x, self.v.y, self.v.z), 
         Vector3::new(self.v.w, self.v.a, self.v.b))
    }
}

pub struct SpatialVectorM {
    v: SpatialVector
}

impl SpatialVectorM {
    pub fn new(w: Vector3, v: Vector3) -> SpatialVectorM {
        SpatialVectorM {
            v: SpatialVector::new(w.x, w.y, w.z, v.x, v.y, v.z)
        }
    }

    pub fn decompose(&self) -> (Vector3, Vector3) {
        (Vector3::new(self.v.x, self.v.y, self.v.z), 
         Vector3::new(self.v.w, self.v.a, self.v.b))
    }

    pub fn cross_f(&self, other: &SpatialVectorF) -> SpatialVectorM {
        let (w, v) = self.decompose();
        let (n, f) = other.decompose();

        SpatialVectorM::new(w.cross(&n) + v.cross(&f), w.cross(&f))
    }

    pub fn cross_m(&self, other: &SpatialVectorM) -> SpatialVectorM {
        let (w, v) = self.decompose();
        let (m, mo) = other.decompose();

        SpatialVectorM::new(w.cross(&m), w.cross(&mo) + v.cross(&m))
    }
}