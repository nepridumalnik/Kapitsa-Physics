use kapitsa_math::Affine3;

use super::{ShapeError, TriangleMesh};

pub struct BoneInfluence {
    bone_index: usize,
    weight: f64,
}

impl BoneInfluence {
    pub fn new(bone_index: usize, weight: f64) -> Result<Self, ShapeError> {
        if !weight.is_finite() || weight <= 0.0 || weight > 1.0 {
            return Err(ShapeError::InvalidWeight);
        }

        Ok(Self { bone_index, weight })
    }

    pub fn bone_index(&self) -> usize {
        self.bone_index
    }

    pub fn weight(&self) -> f64 {
        self.weight
    }
}

/// Mesh with local reference bone transforms and per-vertex influences.
/// Bone hierarchies and skinning evaluation are outside this data type.
/// TODO: Implement skeletal deformation, reference/current vertex positions,
/// dynamic bounding boxes, and spatial acceleration structures.
/// Also add PIMPL.
pub struct SkinnedTriangleMesh {
    mesh: TriangleMesh,
    bone_names: Vec<String>,
    vertex_influences: Vec<Vec<BoneInfluence>>,
    reference_root_transform: Affine3,
    reference_relative_transforms: Vec<Affine3>,
}

impl SkinnedTriangleMesh {
    pub fn new(
        mesh: TriangleMesh,
        bone_names: Vec<String>,
        vertex_influences: Vec<Vec<BoneInfluence>>,
        reference_root_transform: Affine3,
        reference_relative_transforms: Vec<Affine3>,
    ) -> Result<Self, ShapeError> {
        if !bone_names_are_valid(&bone_names) {
            return Err(ShapeError::InvalidBoneNames);
        }
        if vertex_influences.len() != mesh.vertices().len()
            || reference_relative_transforms.len() != bone_names.len()
        {
            return Err(ShapeError::DataLengthMismatch);
        }
        if !reference_root_transform.is_finite()
            || reference_relative_transforms
                .iter()
                .any(|transform| !transform.is_finite())
        {
            return Err(ShapeError::NonFiniteTransform);
        }
        validate_vertex_influences(&vertex_influences, bone_names.len())?;

        Ok(Self {
            mesh,
            bone_names,
            vertex_influences,
            reference_root_transform,
            reference_relative_transforms,
        })
    }

    pub fn mesh(&self) -> &TriangleMesh {
        &self.mesh
    }

    pub fn bone_names(&self) -> &[String] {
        &self.bone_names
    }

    pub fn vertex_influences(&self) -> &[Vec<BoneInfluence>] {
        &self.vertex_influences
    }

    pub fn reference_root_transform(&self) -> Affine3 {
        self.reference_root_transform
    }

    pub fn reference_relative_transforms(&self) -> &[Affine3] {
        &self.reference_relative_transforms
    }
}

fn bone_names_are_valid(names: &[String]) -> bool {
    !names.is_empty()
        && names.iter().all(|name| !name.trim().is_empty())
        && names.iter().enumerate().all(|(index, name)| {
            names[index + 1..]
                .iter()
                .all(|other_name| name != other_name)
        })
}

fn validate_vertex_influences(
    influences: &[Vec<BoneInfluence>],
    bone_count: usize,
) -> Result<(), ShapeError> {
    for vertex_influences in influences {
        if vertex_influences.is_empty() {
            return Err(ShapeError::InvalidWeight);
        }

        let mut total_weight = 0.0;
        for (index, influence) in vertex_influences.iter().enumerate() {
            if influence.bone_index >= bone_count {
                return Err(ShapeError::InvalidInfluenceIndex);
            }
            if vertex_influences[..index]
                .iter()
                .any(|previous| previous.bone_index == influence.bone_index)
            {
                return Err(ShapeError::InvalidInfluenceIndex);
            }
            total_weight += influence.weight;
        }

        if !total_weight.is_finite() || (total_weight - 1.0).abs() > 1.0e-4 {
            return Err(ShapeError::InvalidWeight);
        }
    }

    Ok(())
}
