use super::*;
use crate::{
    aerodynamics::geometry::{
        GeometryConfig, GeometryInstance, GeometryOwner, OpenMeshPolicy, PreparedGeometry,
    },
    shape::TriangleMesh,
};
use kapitsa_math::Quat;

fn flow() -> PressureFlow {
    PressureFlow {
        atmosphere: AtmosphericState::new(1.0, Vec3::ZERO).unwrap(),
        sound_speed: 100.0,
        transform: Transform::identity(),
        center_of_mass: Vec3::ZERO,
        linear_velocity: Vec3::X * 1000.0,
        angular_velocity: Vec3::ZERO,
    }
}

fn prepare(mesh: &TriangleMesh, direction: Vec3) -> PreparedSurfacePatches {
    let instances = [GeometryInstance {
        mesh,
        local_transform: Transform::identity(),
        owner: GeometryOwner::Body,
    }];
    let geometry = PreparedGeometry::prepare(
        &instances,
        GeometryConfig {
            cell_size: 0.1,
            max_cells: 100_000,
            max_work: 10_000_000,
            open_mesh_policy: OpenMeshPolicy::SurfaceOnly,
            weld_tolerance: None,
            max_patch_work: 100_000,
        },
    )
    .unwrap();
    geometry
        .exposed_surface_patches(&instances, direction, 1e-6, 100_000)
        .unwrap()
}

fn plate() -> PreparedSurfacePatches {
    let mesh = TriangleMesh::new(vec![Vec3::ZERO, Vec3::Z, Vec3::Y], vec![[0, 2, 1]]).unwrap();
    prepare(&mesh, Vec3::X)
}

fn close(actual: Vec3, expected: Vec3) {
    assert!(
        (actual - expected).length() < 1e-8,
        "{actual:?} != {expected:?}"
    );
}

#[test]
fn prepared_cube_has_opposing_drag_and_symmetric_com_moment() {
    let mesh = TriangleMesh::new(
        vec![
            Vec3::ZERO,
            Vec3::X,
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::Y,
            Vec3::Z,
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::ONE,
            Vec3::new(0.0, 1.0, 1.0),
        ],
        vec![
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [3, 7, 6],
            [3, 6, 2],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ],
    )
    .unwrap();
    let patches = prepare(&mesh, Vec3::X);
    assert_eq!(patches.len(), 2);
    let state = PressureFlow {
        center_of_mass: Vec3::splat(0.5),
        ..flow()
    };
    let result = evaluate_prepared_mesh(&patches, state).unwrap();
    close(result.force, -Vec3::X * 1_000_000.0);
    close(result.moment, Vec3::ZERO);
    assert!(result.force.dot(state.linear_velocity) < 0.0);
}

#[test]
fn asymmetric_pyramid_nose_loads_windward_slopes_and_drags_at_nose() {
    let mesh = TriangleMesh::new(
        vec![
            Vec3::new(1.0, 0.35, 0.15),
            Vec3::new(0.0, -1.0, -0.5),
            Vec3::new(0.0, 1.0, -0.5),
            Vec3::new(0.0, 1.0, 0.5),
            Vec3::new(0.0, -1.0, 0.5),
        ],
        // Outward winding: all nose slopes point +X, rear cap points -X.
        vec![
            [0, 1, 2],
            [0, 2, 3],
            [0, 3, 4],
            [0, 4, 1],
            [1, 3, 2],
            [1, 4, 3],
        ],
    )
    .unwrap();
    let patches = prepare(&mesh, Vec3::X);
    assert_eq!(patches.len(), 4);
    assert!(patches.iter().all(|patch| patch.normal.x > 0.0));
    assert!(patches.iter().all(|patch| patch.center.x > 0.0));
    let result = evaluate_prepared_mesh(
        &patches,
        PressureFlow {
            center_of_mass: Vec3::new(0.2, 0.0, 0.0),
            linear_velocity: Vec3::X * 1000.0,
            ..flow()
        },
    )
    .unwrap();
    assert!(result.force.x < 0.0);
    assert!(result.force.length() > 0.0);
    assert!(result.moment.length() > 0.0);
}

#[test]
fn prepared_patch_offset_and_com_edit_use_body_local_arm() {
    let patches = plate();
    assert_eq!(patches.len(), 1);
    let center = patches.as_slice()[0].center;
    let result = evaluate_prepared_mesh(
        &patches,
        PressureFlow {
            center_of_mass: center - Vec3::Y,
            ..flow()
        },
    )
    .unwrap();
    close(result.force, -Vec3::X * 500_000.0);
    close(result.moment, Vec3::Z * 500_000.0);
    let moved = evaluate_prepared_mesh(
        &patches,
        PressureFlow {
            center_of_mass: center,
            ..flow()
        },
    )
    .unwrap();
    close(moved.force, result.force);
    close(moved.moment, Vec3::ZERO);
}

#[test]
fn prepared_wrench_rotates_with_motion_and_ignores_world_translation() {
    let patches = plate();
    let state = PressureFlow {
        angular_velocity: -Vec3::Z * 100.0,
        ..flow()
    };
    let original = evaluate_prepared_mesh(&patches, state).unwrap();
    assert!(original.force.length() > 0.0);
    assert!(original.moment.length() > 0.0);
    let transform = Transform::new(
        Vec3::splat(1e12),
        Quat::from_axis_angle(Vec3::Z, 0.7).unwrap(),
    )
    .unwrap();
    let rotated = evaluate_prepared_mesh(
        &patches,
        PressureFlow {
            transform,
            linear_velocity: transform.transform_vector(state.linear_velocity),
            angular_velocity: transform.transform_vector(state.angular_velocity),
            ..state
        },
    )
    .unwrap();
    close(rotated.force, transform.transform_vector(original.force));
    close(rotated.moment, transform.transform_vector(original.moment));
}

#[test]
fn prepared_wrench_matches_explicit_patch_sum_with_wind_and_rotation() {
    let mesh = TriangleMesh::new(
        vec![Vec3::ZERO, Vec3::Z, Vec3::Y, Vec3::Y + Vec3::Z],
        vec![[0, 2, 1], [1, 2, 3]],
    )
    .unwrap();
    let patches = prepare(&mesh, Vec3::X);
    assert_eq!(patches.len(), 2);
    let state = PressureFlow {
        atmosphere: AtmosphericState::new(0.8, Vec3::X * 50.0).unwrap(),
        center_of_mass: Vec3::new(0.0, -0.5, 0.25),
        angular_velocity: -Vec3::Z * 100.0,
        ..flow()
    };
    let mut expected = PressureWrench {
        force: Vec3::ZERO,
        moment: Vec3::ZERO,
    };
    for patch in patches.iter() {
        let load = PressurePatch::new(patch.center, patch.normal, patch.area)
            .unwrap()
            .evaluate(state)
            .unwrap();
        expected.force += load.force;
        expected.moment += load.moment;
    }
    assert_eq!(evaluate_prepared_mesh(&patches, state).unwrap(), expected);
    assert!(expected.force.dot(state.linear_velocity) < 0.0);
}

#[test]
fn prepared_wrench_preserves_vacuum_mach_and_input_errors() {
    let patches = plate();
    let low_mach = PressureFlow {
        linear_velocity: Vec3::X * 499.0,
        ..flow()
    };
    assert_eq!(
        evaluate_prepared_mesh(&patches, low_mach),
        Err(PressureError::UnsupportedMach)
    );
    assert!(
        evaluate_prepared_mesh(
            &patches,
            PressureFlow {
                linear_velocity: Vec3::X * 500.0,
                ..flow()
            }
        )
        .is_ok()
    );
    let zero = PressureWrench {
        force: Vec3::ZERO,
        moment: Vec3::ZERO,
    };
    assert_eq!(
        evaluate_prepared_mesh(
            &patches,
            PressureFlow {
                atmosphere: AtmosphericState::new(0.0, Vec3::ZERO).unwrap(),
                ..low_mach
            }
        )
        .unwrap(),
        zero
    );
    assert_eq!(
        evaluate_prepared_mesh(
            &patches,
            PressureFlow {
                linear_velocity: Vec3::ZERO,
                ..flow()
            }
        )
        .unwrap(),
        zero
    );
    assert_eq!(
        evaluate_prepared_mesh(
            &patches,
            PressureFlow {
                sound_speed: 0.0,
                ..flow()
            }
        ),
        Err(PressureError::InvalidSoundSpeed)
    );
    assert_eq!(
        evaluate_prepared_mesh(
            &patches,
            PressureFlow {
                center_of_mass: Vec3::splat(f64::NAN),
                ..flow()
            }
        ),
        Err(PressureError::InvalidMotion)
    );
    assert_eq!(
        evaluate_prepared_mesh(
            &patches,
            PressureFlow {
                linear_velocity: Vec3::X * f64::MAX,
                ..flow()
            }
        ),
        Err(PressureError::ArithmeticOverflow)
    );
}

#[test]
fn finite_patch_loads_that_overflow_the_sum_are_rejected() {
    let mesh = TriangleMesh::new(
        vec![Vec3::ZERO, Vec3::Z, Vec3::Y * 2.0, Vec3::Y * 2.0 + Vec3::Z],
        vec![[0, 2, 1], [1, 2, 3]],
    )
    .unwrap();
    let patches = prepare(&mesh, Vec3::X);
    assert_eq!(patches.len(), 2);
    let state = PressureFlow {
        atmosphere: AtmosphericState::new((f64::MAX / 1_000_000.0) * 0.6, Vec3::ZERO).unwrap(),
        ..flow()
    };
    for patch in patches.iter() {
        assert!(
            PressurePatch::new(patch.center, patch.normal, patch.area)
                .unwrap()
                .evaluate(state)
                .is_ok()
        );
    }
    assert_eq!(
        evaluate_prepared_mesh(&patches, state),
        Err(PressureError::ArithmeticOverflow)
    );
}

#[test]
fn empty_prepared_set_is_zero_but_still_validates_input() {
    let mesh = TriangleMesh::new(vec![Vec3::ZERO, Vec3::Z, Vec3::Y], vec![[0, 1, 2]]).unwrap();
    let patches = prepare(&mesh, Vec3::X);
    assert!(patches.is_empty());
    assert_eq!(
        evaluate_prepared_mesh(&patches, flow()).unwrap(),
        PressureWrench {
            force: Vec3::ZERO,
            moment: Vec3::ZERO
        }
    );
    assert_eq!(
        evaluate_prepared_mesh(
            &patches,
            PressureFlow {
                sound_speed: f64::NAN,
                ..flow()
            }
        ),
        Err(PressureError::InvalidSoundSpeed)
    );
}

#[test]
fn regression_half_shield_and_coplanar_overlap() -> Result<(), Box<dyn std::error::Error>> {
    let plate = TriangleMesh::new(
        vec![Vec3::ZERO, Vec3::Y, Vec3::Z, Vec3::Y + Vec3::Z],
        vec![[0, 1, 2], [1, 3, 2]],
    )?;
    let shield = TriangleMesh::new(
        vec![Vec3::ZERO, Vec3::Y * 0.5, Vec3::Z, Vec3::Y * 0.5 + Vec3::Z],
        vec![[0, 2, 1], [1, 2, 3]],
    )?;
    let instances = [
        GeometryInstance {
            mesh: &plate,
            local_transform: Transform::identity(),
            owner: GeometryOwner::Body,
        },
        GeometryInstance {
            mesh: &shield,
            local_transform: Transform::new(Vec3::X, Quat::IDENTITY)?,
            owner: GeometryOwner::Body,
        },
    ];
    let geometry = PreparedGeometry::prepare(
        &instances,
        GeometryConfig {
            cell_size: 0.25,
            max_cells: 100_000,
            max_work: 10_000_000,
            open_mesh_policy: OpenMeshPolicy::SurfaceOnly,
            weld_tolerance: None,
            max_patch_work: 100_000,
        },
    )?;
    let patches = geometry.exposed_surface_patches(&instances, Vec3::X, 1e-6, 100_000)?;
    let area: f64 = patches.iter().map(|p| p.area).sum();
    let wrench = evaluate_prepared_mesh(&patches, flow())?;
    eprintln!(
        "half shield: area={area}, force={:?}, moment={:?}; expected area=0.5, force=(-500000,0,0), moment=(0,-250000,375000)",
        wrench.force, wrench.moment
    );
    assert!((area - 0.5).abs() < 1e-9);
    close(wrench.force, -Vec3::X * 500_000.0);
    close(wrench.moment, Vec3::new(0.0, -250_000.0, 375_000.0));
    Ok(())
}
#[test]
fn regression_coplanar_overlap_different_triangulation() -> Result<(), Box<dyn std::error::Error>> {
    let first = TriangleMesh::new(
        vec![Vec3::ZERO, Vec3::Y, Vec3::Z, Vec3::Y + Vec3::Z],
        vec![[0, 1, 2], [1, 3, 2]],
    )?;
    let second = TriangleMesh::new(
        vec![Vec3::ZERO, Vec3::Y, Vec3::Z, Vec3::Y + Vec3::Z],
        vec![[0, 1, 3], [0, 3, 2]],
    )?;
    let instances = [
        GeometryInstance {
            mesh: &first,
            local_transform: Transform::identity(),
            owner: GeometryOwner::Body,
        },
        GeometryInstance {
            mesh: &second,
            local_transform: Transform::identity(),
            owner: GeometryOwner::Body,
        },
    ];
    let geometry = PreparedGeometry::prepare(
        &instances,
        GeometryConfig {
            cell_size: 0.25,
            max_cells: 100_000,
            max_work: 10_000_000,
            open_mesh_policy: OpenMeshPolicy::SurfaceOnly,
            weld_tolerance: None,
            max_patch_work: 100_000,
        },
    )?;
    let patches = geometry.exposed_surface_patches(&instances, Vec3::X, 1e-6, 100_000)?;
    let area: f64 = patches.iter().map(|p| p.area).sum();
    let wrench = evaluate_prepared_mesh(&patches, flow())?;
    eprintln!(
        "coplanar overlap: area={area}, force={:?}, moment={:?}; expected area=1, force=(-1000000,0,0), moment=(0,-500000,500000)",
        wrench.force, wrench.moment
    );
    assert!((area - 1.0).abs() < 1e-9);
    close(wrench.force, -Vec3::X * 1_000_000.0);
    close(wrench.moment, Vec3::new(0.0, -500_000.0, 500_000.0));
    Ok(())
}

#[test]
fn shielding_load_varies_continuously_with_blocker_width() -> Result<(), Box<dyn std::error::Error>>
{
    let plate = TriangleMesh::new(
        vec![Vec3::ZERO, Vec3::Y, Vec3::Z, Vec3::Y + Vec3::Z],
        vec![[0, 1, 2], [1, 3, 2]],
    )?;
    for width in [0.1, 0.25, 0.49, 0.5, 0.51, 0.75, 0.9] {
        let shield = TriangleMesh::new(
            vec![
                Vec3::ZERO,
                Vec3::Y * width,
                Vec3::Z,
                Vec3::Y * width + Vec3::Z,
            ],
            vec![[0, 2, 1], [1, 2, 3]],
        )?;
        let instances = [
            GeometryInstance {
                mesh: &plate,
                local_transform: Transform::identity(),
                owner: GeometryOwner::Body,
            },
            GeometryInstance {
                mesh: &shield,
                local_transform: Transform::new(Vec3::X * 1e-9, Quat::IDENTITY)?,
                owner: GeometryOwner::Body,
            },
        ];
        let geometry = PreparedGeometry::prepare(
            &instances,
            GeometryConfig {
                cell_size: 0.25,
                max_cells: 100_000,
                max_work: 10_000_000,
                open_mesh_policy: OpenMeshPolicy::SurfaceOnly,
                weld_tolerance: None,
                max_patch_work: 100_000,
            },
        )?;
        for sources in [instances, [instances[1], instances[0]]] {
            let patches = geometry.exposed_surface_patches(&sources, Vec3::X, 0.01, 100_000)?;
            let result = evaluate_prepared_mesh(&patches, flow())?;
            let magnitude = 1_000_000.0 * (1.0 - width);
            close(result.force, -Vec3::X * magnitude);
            close(
                result.moment,
                Vec3::new(0.0, -0.5 * magnitude, 0.5 * (1.0 + width) * magnitude),
            );
        }
    }
    Ok(())
}

#[test]
fn rotational_force_and_moment_converge_under_mesh_refinement()
-> Result<(), Box<dyn std::error::Error>> {
    let mut previous_error = f64::INFINITY;
    for subdivisions in [1, 2, 4, 8] {
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        for x in 0..subdivisions {
            for z in 0..subdivisions {
                let start = vertices.len();
                let h = 1.0 / subdivisions as f64;
                let a = Vec3::new(1.0 + x as f64 * h, 0.0, z as f64 * h);
                vertices.extend([
                    a,
                    a + Vec3::X * h,
                    a + Vec3::Z * h,
                    a + (Vec3::X + Vec3::Z) * h,
                ]);
                triangles.extend([
                    [start, start + 2, start + 1],
                    [start + 1, start + 2, start + 3],
                ]);
            }
        }
        let mesh = TriangleMesh::new(vertices, triangles)?;
        let instances = [GeometryInstance {
            mesh: &mesh,
            local_transform: Transform::identity(),
            owner: GeometryOwner::Body,
        }];
        let geometry = PreparedGeometry::prepare(
            &instances,
            GeometryConfig {
                cell_size: 0.25,
                max_cells: 100_000,
                max_work: 10_000_000,
                open_mesh_policy: OpenMeshPolicy::SurfaceOnly,
                weld_tolerance: None,
                max_patch_work: 10_000_000,
            },
        )?;
        let patches = geometry.surface_patches_with_flow(&instances, 10_000_000, |point| {
            Ok(Vec3::Z.cross(point) * 1000.0)
        })?;
        let wrench = evaluate_prepared_mesh(
            &patches,
            PressureFlow {
                linear_velocity: Vec3::ZERO,
                angular_velocity: Vec3::Z * 1000.0,
                ..flow()
            },
        )?;
        // Integrals on x in [1,2], z in [0,1]: x^2, z*x^2 and x^3.
        let force = -Vec3::Y * (7_000_000.0 / 3.0);
        let moment = Vec3::new(7_000_000.0 / 6.0, 0.0, -3_750_000.0);
        let error = (wrench.force - force).length() + (wrench.moment - moment).length();
        assert!(
            error < previous_error * 0.3,
            "subdivisions={subdivisions}, error={error}, previous={previous_error}"
        );
        previous_error = error;
    }
    Ok(())
}
