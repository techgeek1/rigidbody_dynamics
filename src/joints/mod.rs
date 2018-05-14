mod revolute;
mod spherical;
mod six_dof;

pub use self::revolute::Revolute;
pub use self::spherical::Spherical;
pub use self::six_dof::SixDof;

pub trait Joint {

}