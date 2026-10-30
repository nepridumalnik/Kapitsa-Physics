use glam::Affine3A;

use super::TriangleMesh;

pub struct BoneInfluence {
    bone_index: usize,
    weight: f32,
}

impl BoneInfluence {
    pub fn new(bone_index: usize, weight: f32) -> Option<Self> {
        (weight.is_finite() && weight > 0.0 && weight <= 1.0)
            .then_some(Self { bone_index, weight })
    }

    pub fn bone_index(&self) -> usize {
        self.bone_index
    }

    pub fn weight(&self) -> f32 {
        self.weight
    }
}

/// Mesh with local reference bone transforms and per-vertex influences.
/// Bone hierarchies and skinning evaluation are outside this data type.
pub struct SkinnedTriangleMesh {
    mesh: TriangleMesh,
    bone_names: Vec<String>,
    vertex_influences: Vec<Vec<BoneInfluence>>,
    reference_root_transform: Affine3A,
    reference_relative_transforms: Vec<Affine3A>,
}

impl SkinnedTriangleMesh {
    pub fn new(
        mesh: TriangleMesh,
        bone_names: Vec<String>,
        vertex_influences: Vec<Vec<BoneInfluence>>,
        reference_root_transform: Affine3A,
        reference_relative_transforms: Vec<Affine3A>,
    ) -> Option<Self> {
        if !bone_names_are_valid(&bone_names)
            || vertex_influences.len() != mesh.vertices().len()
            || !reference_root_transform.is_finite()
            || reference_relative_transforms.len() != bone_names.len()
            || reference_relative_transforms
                .iter()
                .any(|transform| !transform.is_finite())
            || !vertex_influences_are_valid(&vertex_influences, bone_names.len())
        {
            return None;
        }

        Some(Self {
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

    pub fn reference_root_transform(&self) -> Affine3A {
        self.reference_root_transform
    }

    pub fn reference_relative_transforms(&self) -> &[Affine3A] {
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

fn vertex_influences_are_valid(influences: &[Vec<BoneInfluence>], bone_count: usize) -> bool {
    influences.iter().all(|vertex_influences| {
        if vertex_influences.is_empty() {
            return false;
        }

        let mut total_weight = 0.0;
        for (index, influence) in vertex_influences.iter().enumerate() {
            if influence.bone_index >= bone_count {
                return false;
            }
            if vertex_influences[..index]
                .iter()
                .any(|previous| previous.bone_index == influence.bone_index)
            {
                return false;
            }
            total_weight += influence.weight;
        }

        total_weight.is_finite() && (total_weight - 1.0).abs() <= 1.0e-4
    })
}
