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
    prepare_with_budget(mesh, direction, 100_000)
}

fn prepare_with_budget(
    mesh: &TriangleMesh,
    direction: Vec3,
    budget: usize,
) -> PreparedSurfacePatches {
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
            max_patch_work: budget,
        },
    )
    .unwrap();
    geometry
        .exposed_surface_patches(&instances, direction, 1e-6, budget)
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
fn signed_cp_on_prepared_mesh_sums_force_and_moment() {
    let patches = plate();
    let state = PressureFlow {
        linear_velocity: Vec3::X * 50.0,
        ..flow()
    };
    for cp in [-0.4, 0.4] {
        let pressure = SurfacePressure::new(cp, 0.0, 0.5).unwrap();
        let total = evaluate_prepared_surface(&patches, state, pressure).unwrap();
        let mut force = Vec3::ZERO;
        let mut moment = Vec3::ZERO;
        for face in patches.iter() {
            let load = PressurePatch::new(face.center, face.normal, face.area)
                .unwrap()
                .evaluate_surface(state, pressure)
                .unwrap();
            force += load.force;
            moment += load.moment;
        }
        close(total.force, force);
        close(total.moment, moment);
        assert!(total.force.x * cp < 0.0);
    }
    assert_eq!(
        evaluate_prepared_surface(
            &patches,
            state,
            SurfacePressure::new(1.0, 0.0, 0.49).unwrap()
        ),
        Err(PressureError::UnsupportedMach)
    );
}

#[test]
fn potential_sphere_collider_front_patches_follow_analytic_cp() {
    use crate::{
        aerodynamics::colliders::{ColliderTessellation, collider_mesh},
        collider::Collider,
        shape::{Shape, Sphere},
    };
    let collider = Collider::new(
        Shape::Sphere(Sphere::new(1.0).unwrap()),
        Transform::identity(),
    );
    let mesh = collider_mesh([&collider], ColliderTessellation::new(16, 9, 1000).unwrap()).unwrap();
    let patches = prepare_with_budget(&mesh, Vec3::X, 1_000_000);
    let state = PressureFlow {
        linear_velocity: Vec3::X * 20.0,
        ..flow()
    };
    assert!(patches.iter().count() > 1);
    for face in patches.iter() {
        let patch = PressurePatch::new(face.center, face.normal, face.area).unwrap();
        let cp = 1.0 - 2.25 * (1.0 - patch.normal().x.powi(2));
        let expected = patch
            .evaluate_surface(state, SurfacePressure::new(cp, 0.0, 0.2).unwrap())
            .unwrap();
        let actual = patch.evaluate_potential_sphere(state).unwrap();
        close(actual.force, expected.force);
        close(actual.moment, expected.moment);
    }
}

#[test]
fn potential_cylinder_accepts_tessellated_sides_and_rejects_caps() {
    use crate::{
        aerodynamics::colliders::{ColliderTessellation, collider_mesh},
        collider::Collider,
        shape::{Cylinder, Shape},
    };
    let collider = Collider::new(
        Shape::Cylinder(Cylinder::new(-Vec3::Z, Vec3::Z, 1.0).unwrap()),
        Transform::identity(),
    );
    let mesh = collider_mesh([&collider], ColliderTessellation::new(16, 8, 1000).unwrap()).unwrap();
    let state = PressureFlow {
        linear_velocity: Vec3::X * 20.0,
        ..flow()
    };
    let mut sides = 0;
    let mut caps = 0;
    for face in mesh.triangles() {
        let [a, b, c] = face.map(|index| mesh.vertices()[index]);
        let cross = (b - a).cross(c - a);
        let patch = PressurePatch::new((a + b + c) / 3.0, cross, cross.length() * 0.5).unwrap();
        if patch.normal().dot(Vec3::Z).abs() > 0.5 {
            assert_eq!(
                patch.evaluate_potential_cylinder(state, Vec3::Z),
                Err(PressureError::InvalidPatch)
            );
            caps += 1;
        } else {
            assert!(patch.evaluate_potential_cylinder(state, Vec3::Z).is_ok());
            sides += 1;
        }
    }
    assert!(sides > 0 && caps > 0);
}

#[test]
fn collider_sphere_and_cylinder_converge_to_newtonian_integrals() {
    use crate::{
        aerodynamics::colliders::{ColliderTessellation, collider_mesh},
        collider::Collider,
        shape::{Cylinder, Shape, Sphere},
    };
    use std::f64::consts::PI;

    // Cp = 2 cos²(theta): Cd = 1 for a sphere and 4/3 for a
    // cross-flow cylinder, using frontal projected area as reference.
    // These are model integrals, not experimental validation.
    for (shape, area, expected_cd) in [
        (Shape::Sphere(Sphere::new(1.0).unwrap()), PI, 1.0),
        (
            Shape::Cylinder(Cylinder::new(-Vec3::Z, Vec3::Z, 1.0).unwrap()),
            4.0,
            4.0 / 3.0,
        ),
    ] {
        let collider = Collider::new(shape, Transform::identity());
        let mut previous_error = f64::INFINITY;
        for segments in [8, 16, 32] {
            let mesh = collider_mesh(
                [&collider],
                ColliderTessellation::new(segments, segments / 2 + 1, 16_384).unwrap(),
            )
            .unwrap();
            let load =
                evaluate_prepared_mesh(&prepare_with_budget(&mesh, Vec3::X, 20_000_000), flow())
                    .unwrap();
            let cd = -load.force.x / (500_000.0 * area);
            let error = (cd - expected_cd).abs();
            assert!(
                error < previous_error,
                "segments={segments}, Cd={cd}, previous error={previous_error}"
            );
            previous_error = error;
        }
        assert!(previous_error < 0.03, "final error={previous_error}");
    }
}

#[test]
fn collider_cylinder_explicit_base_pressure_matches_polygon_area() {
    use crate::{
        aerodynamics::colliders::{ColliderTessellation, collider_mesh},
        collider::Collider,
        shape::{Cylinder, Shape},
    };
    let segments = 16;
    let collider = Collider::new(
        Shape::Cylinder(Cylinder::new(-Vec3::X, Vec3::X, 1.0).unwrap()),
        Transform::identity(),
    );
    let mesh = collider_mesh(
        [&collider],
        ColliderTessellation::new(segments, 8, 1000).unwrap(),
    )
    .unwrap();
    // The aft-facing cap is selected explicitly by reversing the exposure direction.
    // This works for this isolated convex cylinder, not arbitrary wake geometry.
    let patches = prepare_with_budget(&mesh, -Vec3::X, 1_000_000);
    let result = evaluate_prepared_base(
        &patches,
        flow(),
        BasePressure::new(-0.4, 5.0, 10.0).unwrap(),
    )
    .unwrap();
    let polygon_area = segments as f64 * (std::f64::consts::TAU / segments as f64).sin() / 2.0;
    assert!((result.force.x + 500_000.0 * 0.4 * polygon_area).abs() < 1e-7);
    assert!(result.moment.length() < 1e-7);
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

    // A closed component fully inside the hull must not add pressure loads.
    let mut vertices = mesh.vertices().to_vec();
    vertices.extend(mesh.vertices().iter().map(|&v| v * 0.5 + Vec3::splat(0.25)));
    let mut triangles = mesh.triangles().to_vec();
    triangles.extend(mesh.triangles().iter().map(|t| t.map(|index| index + 8)));
    let nested = TriangleMesh::new(vertices, triangles).unwrap();
    let nested_result = evaluate_prepared_mesh(&prepare(&nested, Vec3::X), state).unwrap();
    close(nested_result.force, result.force);
    close(nested_result.moment, result.moment);
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
fn full_base_visibility_rejects_full_and_partial_blockers_for_mesh_and_colliders()
-> Result<(), Box<dyn std::error::Error>> {
    use crate::{
        aerodynamics::colliders::{ColliderTessellation, collider_mesh},
        collider::Collider,
        shape::{Shape, Triangle},
    };
    let vertices = [Vec3::ZERO, Vec3::Y, Vec3::Z];
    for collider_mode in [false, true] {
        let mesh = if collider_mode {
            let collider = Collider::new(
                Shape::Triangle(Triangle::new(vertices[0], vertices[1], vertices[2])?),
                Transform::identity(),
            );
            collider_mesh([&collider], ColliderTessellation::default())?
        } else {
            TriangleMesh::new(vertices.to_vec(), vec![[0, 1, 2]])?
        };
        let selected = [mesh.triangles()[0].map(|i| mesh.vertices()[i])];
        let visible = prepare(&mesh, Vec3::X);
        assert_eq!(
            verify_full_base_visibility(&selected, &visible),
            Ok(()),
            "selected={selected:?}, visible={:?}",
            visible.as_slice()
        );
        assert_eq!(
            verify_full_base_visibility(&[selected[0], selected[0]], &visible),
            Err(PressureError::BaseNotFullyVisible)
        );
        for width in [0.5, 1.0] {
            let shield = TriangleMesh::new(
                vec![Vec3::X, Vec3::X + Vec3::Y * width, Vec3::X + Vec3::Z],
                vec![[0, 2, 1]],
            )?;
            let instances = [
                GeometryInstance {
                    mesh: &mesh,
                    local_transform: Transform::identity(),
                    owner: GeometryOwner::Body,
                },
                GeometryInstance {
                    mesh: &shield,
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
            let visible = geometry.exposed_surface_patches(&instances, Vec3::X, 1e-6, 100_000)?;
            assert_eq!(
                verify_full_base_visibility(&selected, &visible),
                Err(PressureError::BaseNotFullyVisible)
            );
        }
    }
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

#[test]
fn stagnation_at_source_centroid_does_not_erase_rotational_windward_area()
-> Result<(), Box<dyn std::error::Error>> {
    let a = Vec3::new(-2.0, 0.0, 0.0);
    let b = Vec3::new(1.0, 0.0, -1.0);
    let c = Vec3::new(1.0, 0.0, 1.0);
    let mut previous_error = None;
    for subdivisions in [1, 3, 6, 12] {
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        let point = |u: usize, v: usize| {
            a + (b - a) * (u as f64 / subdivisions as f64)
                + (c - a) * (v as f64 / subdivisions as f64)
        };
        for u in 0..subdivisions {
            for v in 0..subdivisions - u {
                let base = vertices.len();
                vertices.extend([point(u, v), point(u + 1, v), point(u, v + 1)]);
                triangles.push([base, base + 2, base + 1]);
                if u + v + 1 < subdivisions {
                    let base = vertices.len();
                    vertices.extend([point(u + 1, v), point(u + 1, v + 1), point(u, v + 1)]);
                    triangles.push([base, base + 2, base + 1]);
                }
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
        let omega = Vec3::Z * 10_000.0;
        let patches =
            geometry.surface_patches_with_flow(&instances, 10_000_000, |p| Ok(omega.cross(p)))?;
        let area: f64 = patches.iter().map(|patch| patch.area).sum();
        assert!(
            (area - 5.0 / 3.0).abs() < 1e-12,
            "subdivisions={subdivisions}, windward_area={area}"
        );
        let result = evaluate_prepared_mesh(
            &patches,
            PressureFlow {
                linear_velocity: Vec3::ZERO,
                angular_velocity: omega,
                sound_speed: 1.0,
                ..flow()
            },
        )?;
        assert!(result.force.y < 0.0 && result.moment.z < 0.0);
        // Positive x has width 2(x+2)/3, x in [0,1]. Integrate x^2 and x^3.
        let force = -Vec3::Y * (1e8 * 11.0 / 18.0);
        let moment = -Vec3::Z * (1e8 * 7.0 / 15.0);
        let error = (result.force - force).length() + (result.moment - moment).length();
        // n=1 clipping and n=3 both have one 1 m-wide active x layer.
        // n=3,6,12 then halve that layer width and test quadratic convergence.
        if subdivisions >= 3 {
            if let Some(previous) = previous_error {
                assert!(
                    error < previous * 0.3,
                    "subdivisions={subdivisions}, error={error}, previous={previous}"
                );
            }
            previous_error = Some(error);
        }
    }
    Ok(())
}
