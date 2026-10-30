use glam::{Affine3A, Vec3};

pub struct MlLevelSet {
    training_grid_min: Vec3,
    training_grid_axes: [Vec3; 3],
    model_weights: Vec<Vec<f32>>,
    model_weight_shapes: Vec<Vec<usize>>,
    active_bones: Vec<MlLevelSetBone>,
    total_rotation_components: usize,
    signed_distance_scaling: f32,
}

pub struct MlLevelSetBone {
    name: String,
    reference_transform: Affine3A,
    relative_transform: Affine3A,
    rotation_components: Vec<usize>,
}

impl MlLevelSetBone {
    pub fn new(
        name: String,
        reference_transform: Affine3A,
        relative_transform: Affine3A,
        rotation_components: Vec<usize>,
    ) -> Option<Self> {
        if name.trim().is_empty()
            || !reference_transform.is_finite()
            || !relative_transform.is_finite()
            || rotation_components
                .iter()
                .enumerate()
                .any(|(index, component)| rotation_components[..index].contains(component))
        {
            return None;
        }

        Some(Self {
            name,
            reference_transform,
            relative_transform,
            rotation_components,
        })
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn reference_transform(&self) -> Affine3A {
        self.reference_transform
    }

    pub fn relative_transform(&self) -> Affine3A {
        self.relative_transform
    }

    pub fn rotation_components(&self) -> &[usize] {
        &self.rotation_components
    }
}

impl MlLevelSet {
    pub fn from_vec(
        training_grid_min: Vec3,
        training_grid_axes: [Vec3; 3],
        model_weights: Vec<Vec<f32>>,
        model_weight_shapes: Vec<Vec<usize>>,
        active_bones: Vec<MlLevelSetBone>,
        total_rotation_components: usize,
        signed_distance_scaling: f32,
    ) -> Option<Self> {
        let grid_determinant =
            training_grid_axes[0].dot(training_grid_axes[1].cross(training_grid_axes[2]));

        if !training_grid_min.is_finite()
            || training_grid_axes.iter().any(|axis| {
                !axis.is_finite()
                    || !axis.length_squared().is_finite()
                    || axis.length_squared() == 0.0
            })
            || !grid_determinant.is_finite()
            || grid_determinant == 0.0
            || model_weights.is_empty()
            || model_weights.len() != model_weight_shapes.len()
            || !active_bones_are_valid(&active_bones, total_rotation_components)
            || !signed_distance_scaling.is_finite()
            || signed_distance_scaling <= 0.0
        {
            return None;
        }

        for (weights, shape) in model_weights.iter().zip(&model_weight_shapes) {
            let weight_count = shape.iter().try_fold(1_usize, |product, dimension| {
                product.checked_mul(*dimension)
            })?;

            if shape.is_empty()
                || shape.contains(&0)
                || weights.is_empty()
                || weight_count != weights.len()
                || !weights.iter().all(|weight| weight.is_finite())
            {
                return None;
            }
        }

        Some(Self {
            training_grid_min,
            training_grid_axes,
            model_weights,
            model_weight_shapes,
            active_bones,
            total_rotation_components,
            signed_distance_scaling,
        })
    }

    pub fn training_grid_min(&self) -> Vec3 {
        self.training_grid_min
    }

    pub fn training_grid_axes(&self) -> &[Vec3; 3] {
        &self.training_grid_axes
    }

    pub fn model_weights(&self) -> &[Vec<f32>] {
        &self.model_weights
    }

    pub fn model_weight_shapes(&self) -> &[Vec<usize>] {
        &self.model_weight_shapes
    }

    pub fn active_bones(&self) -> &[MlLevelSetBone] {
        &self.active_bones
    }

    pub fn total_rotation_components(&self) -> usize {
        self.total_rotation_components
    }

    pub fn signed_distance_scaling(&self) -> f32 {
        self.signed_distance_scaling
    }
}

fn active_bones_are_valid(bones: &[MlLevelSetBone], rotation_component_count: usize) -> bool {
    !bones.is_empty()
        && bones.iter().enumerate().all(|(index, bone)| {
            bones[index + 1..]
                .iter()
                .all(|other_bone| bone.name != other_bone.name)
                && bone
                    .rotation_components
                    .iter()
                    .all(|component| *component < rotation_component_count)
        })
}
