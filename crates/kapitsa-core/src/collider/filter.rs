/// Layer membership and interaction mask for collision filtering.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CollisionFilter {
    layers: u32,
    mask: u32,
}

impl CollisionFilter {
    pub const fn new(layers: u32, mask: u32) -> Self {
        Self { layers, mask }
    }

    pub const fn layers(&self) -> u32 {
        self.layers
    }

    pub const fn mask(&self) -> u32 {
        self.mask
    }

    pub const fn allows(&self, other: &Self) -> bool {
        self.layers & other.mask != 0 && other.layers & self.mask != 0
    }
}

impl Default for CollisionFilter {
    fn default() -> Self {
        Self::new(1, u32::MAX)
    }
}
