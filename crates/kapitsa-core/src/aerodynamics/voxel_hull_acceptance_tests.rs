use super::*;
use crate::{
    aerodynamics::{
        colliders::{ColliderTessellation, collider_mesh},
        geometry::{
            GeometryConfig, GeometryError, GeometryInstance, GeometryOwner, OpenMeshPolicy,
            PreparedGeometry,
        },
    },
    collider::Collider,
    shape::{Shape, Sphere},
};

const WORK: usize = 100_000_000;

#[test]
fn staged_off_center_pressure_is_covariant_and_reconstructs_total()
-> Result<(), Box<dyn std::error::Error>> {
    use crate::aerodynamics::geometry::OwnerSlot;
    // Two disjoint unit right triangles normal to the relative flow. Their
    // combined centroid is (0, 7/3, 1/3), and total area is 1 m².
    let mesh =
        crate::shape::TriangleMesh::new(vec![Vec3::ZERO, Vec3::Y, Vec3::Z], vec![[0, 1, 2]])?;
    let owners = [0, 1].map(|i| GeometryOwner::Slot(OwnerSlot::new(i)));
    let instances = [
        GeometryInstance {
            mesh: &mesh,
            local_transform: Transform::identity(),
            owner: owners[0],
        },
        GeometryInstance {
            mesh: &mesh,
            local_transform: Transform::new(Vec3::Y * 4.0, kapitsa_math::Quat::IDENTITY)?,
            owner: owners[1],
        },
    ];
    let geometry = PreparedGeometry::prepare(&instances, config(0.125))?;
    let patches = geometry.exposed_surface_patches(&instances, Vec3::X, 0.01, WORK)?;
    let mut workspace = OwnerPressureWorkspace::new(&owners).ok_or("ordered owners")?;
    workspace.set_pressure_model(MeshPressureModel::ModifiedNewtonianTriangleQuadrature);
    let speed = 4.63 * 340.0;
    // Appendix A pressure coefficient at Mach 4.63 (twice the spherical Cd).
    let force_scale = speed * speed * 0.901_867_771_160_288;
    let force = -Vec3::X * force_scale;
    let com = Vec3::new(0.0, 1.0, 0.0);
    let rotation = kapitsa_math::Quat::from_axis_angle(Vec3::Z, 0.7).ok_or("rotation")?;
    for rotation in [kapitsa_math::Quat::IDENTITY, rotation] {
        let transform = Transform::new(Vec3::new(5.0, -3.0, 2.0), rotation)?;
        let flow = PressureFlow {
            atmosphere: AtmosphericState::new(1.0, Vec3::ZERO)?,
            sound_speed: 340.0,
            transform,
            center_of_mass: com,
            linear_velocity: transform.transform_vector(Vec3::X * speed),
            angular_velocity: Vec3::ZERO,
        };
        workspace.prepare(&patches, flow)?;
        let total = workspace.staged_total()?.ok_or("staged total")?;
        let loads = workspace.staged().ok_or("staged owners")?;
        let tol = 1e-10 * force_scale;
        assert!((total.force - transform.transform_vector(force)).length() < tol);
        let mut reconstructed = Vec3::ZERO;
        for (i, (owner, load)) in loads.iter().enumerate() {
            assert_eq!(*owner, owners[i]);
            let centroid = Vec3::new(0.0, 4.0 * i as f64 + 1.0 / 3.0, 1.0 / 3.0);
            let expected = transform.transform_vector((centroid - com).cross(force * 0.5));
            assert!((load.force - transform.transform_vector(force * 0.5)).length() < tol);
            assert!((load.moment - expected).length() < tol);
            reconstructed += load.moment;
        }
        assert!((total.moment - reconstructed).length() < tol);
        assert!((total.force - loads[0].1.force - loads[1].1.force).length() < tol);
        workspace.prepare(
            &patches,
            PressureFlow {
                atmosphere: AtmosphericState::new(0.0, Vec3::ZERO)?,
                ..flow
            },
        )?;
        let zero = workspace.staged_total()?.ok_or("vacuum total")?;
        assert_eq!(zero.force, Vec3::ZERO);
        assert_eq!(zero.moment, Vec3::ZERO);
        assert!(
            workspace
                .staged()
                .ok_or("vacuum owners")?
                .iter()
                .all(|(_, load)| *load == zero)
        );
    }
    Ok(())
}

#[test]
fn cooked_union_attributes_no_pressure_to_enclosed_slot() -> Result<(), Box<dyn std::error::Error>>
{
    use crate::aerodynamics::{
        geometry::OwnerSlot,
        shape::{CookedShape, CookedSource},
    };
    use std::sync::Arc;

    let tessellation = ColliderTessellation::new(16, 9, 4096)?;
    let [outer, inner] = [1.0, 0.25].map(|radius| {
        let collider = Collider::new(Shape::Sphere(Sphere::new(radius)?), Transform::identity());
        collider_mesh([&collider], tessellation).map(Arc::new)
    });
    let sources = [outer?, inner?]
        .into_iter()
        .zip([0, 1])
        .map(|(mesh, index)| CookedSource {
            mesh,
            local_transform: Transform::identity(),
            slot: OwnerSlot::new(index),
        })
        .collect::<Vec<_>>();
    let flow = PressureFlow {
        atmosphere: AtmosphericState::new(1.0, Vec3::ZERO)?,
        sound_speed: 340.0,
        transform: Transform::identity(),
        center_of_mass: Vec3::ZERO,
        linear_velocity: Vec3::X * (4.63 * 340.0),
        angular_velocity: Vec3::ZERO,
    };
    let evaluate = |sources: &[CookedSource]| -> Result<_, crate::Error> {
        let shape = CookedShape::cook(sources, config(0.125), false)?;
        let instances = shape
            .sources()
            .iter()
            .map(|source| GeometryInstance {
                mesh: &source.mesh,
                local_transform: source.local_transform,
                owner: GeometryOwner::Slot(source.slot),
            })
            .collect::<Vec<_>>();
        let patches = shape
            .geometry()
            .exposed_surface_patches(&instances, Vec3::X, 0.01, WORK)?;
        evaluate_prepared_mesh_by_owner_with_model(
            &patches,
            flow,
            MeshPressureModel::ModifiedNewtonian,
        )
    };
    let single = evaluate(&sources[..1])?;
    let assembly = evaluate(&sources)?;
    assert_eq!(assembly.len(), 1);
    assert_eq!(assembly[0].0, GeometryOwner::Slot(OwnerSlot::new(0)));
    let reference = single[0].1;
    let actual = assembly[0].1;
    let force_scale = reference.force.length();
    assert!(force_scale > 0.0);
    assert!((actual.force - reference.force).length() < 1e-10 * force_scale);
    // Moment scale is the reference force times the 1 m outer radius.
    assert!((actual.moment - reference.moment).length() < 1e-10 * force_scale);
    Ok(())
}

fn config(cell_size: f64) -> GeometryConfig {
    GeometryConfig {
        cell_size,
        max_cells: 1_048_576,
        max_work: WORK,
        open_mesh_policy: OpenMeshPolicy::SurfaceOnly,
        weld_tolerance: Some(1e-10),
        max_patch_work: WORK,
    }
}

fn annulus(reflect_diagonals: bool) -> Result<crate::shape::TriangleMesh, crate::Error> {
    const SIDES: usize = 16;
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    let inner_radius = 1.0 / (std::f64::consts::PI / SIDES as f64).cos();
    for i in 0..SIDES {
        let (sin, cos) =
            (std::f64::consts::TAU * (i % (SIDES / 2)) as f64 / SIDES as f64).sin_cos();
        let (sin, cos) = if i < SIDES / 2 {
            (sin, cos)
        } else {
            (-sin, -cos)
        };
        for (radius, z) in [
            (1.1, -0.025),
            (1.1, 0.025),
            (inner_radius, 0.025),
            (inner_radius, -0.025),
        ] {
            vertices.push(Vec3::new(radius * cos, radius * sin, z));
        }
        for j in 0..4 {
            let a = 4 * i + j;
            let b = 4 * ((i + 1) % SIDES) + j;
            let c = 4 * ((i + 1) % SIDES) + (j + 1) % 4;
            let d = 4 * i + (j + 1) % 4;
            if reflect_diagonals {
                triangles.extend([[a, b, d], [b, c, d]]);
            } else {
                triangles.extend([[a, b, c], [a, c, d]]);
            }
        }
    }
    crate::shape::TriangleMesh::new(vertices, triangles)
}

#[test]
fn rotating_annulus_local_mach_and_visibility() -> Result<(), Box<dyn std::error::Error>> {
    let mesh = annulus(false)?;
    let instances = [GeometryInstance {
        mesh: &mesh,
        local_transform: Transform::identity(),
        owner: GeometryOwner::Body,
    }];
    let flow = PressureFlow {
        atmosphere: AtmosphericState::new(1.0, Vec3::ZERO)?,
        sound_speed: 340.0,
        transform: Transform::identity(),
        center_of_mass: Vec3::ZERO,
        linear_velocity: Vec3::ZERO,
        angular_velocity: Vec3::Z * (4.2 * 340.0),
    };
    let mut voxel_reference: Option<PressureWrench> = None;
    let mut voxel_workspace = OwnerPressureWorkspace::new(&[GeometryOwner::Body]).ok_or("owner")?;
    voxel_workspace.set_pressure_model(MeshPressureModel::ModifiedNewtonian);
    for cell_size in [0.05, 0.025, 0.0125] {
        let geometry = PreparedGeometry::prepare(&instances, config(cell_size))?;
        let patches = geometry.surface_patches_with_flow(&instances, WORK, |point| {
            let velocity = flow.angular_velocity.cross(point);
            let mach = velocity.length() / flow.sound_speed;
            assert!((4.2 - 1e-12..=4.62 + 1e-12).contains(&mach));
            Ok(velocity)
        })?;
        let wrench = evaluate_prepared_mesh_modified_newtonian(&patches, flow)?;
        eprintln!(
            "h={cell_size} force={:?} moment={:?}",
            wrench.force, wrench.moment
        );
        assert!(wrench.moment.z < 0.0);
        // Half-turn symmetry enforces zero in-plane force. The observed
        // transverse moment is a separate unresolved acceptance discrepancy.
        assert!(wrench.force.length() < 1e-10 * wrench.moment.z.abs() / 1.1);
        voxel_workspace.prepare_triangle_quadrature(&patches, flow)?;
        let integrated = voxel_workspace.staged_total()?.ok_or("staged total")?;
        eprintln!("h={cell_size} three-point pressure={integrated:?}");
        assert!(integrated.moment.z < 0.0);
        assert!(integrated.force.length() < 1e-12 * integrated.moment.z.abs() / 1.1);
        if let Some(reference) = voxel_reference {
            // For this fixed triangulation, changing the occupancy grid should
            // not change the integrated load beyond summation roundoff.
            assert!(
                (integrated.moment - reference.moment).length() < 1e-12 * reference.moment.z.abs()
            );
        } else {
            voxel_reference = Some(integrated);
        }
    }
    let reflected = annulus(true)?;
    let instances = [GeometryInstance {
        mesh: &reflected,
        ..instances[0]
    }];
    let geometry = PreparedGeometry::prepare(&instances, config(0.025))?;
    let patches = geometry.surface_patches_with_flow(&instances, WORK, |point| {
        Ok(flow.angular_velocity.cross(point))
    })?;
    let wrench = evaluate_prepared_mesh_modified_newtonian(&patches, flow)?;
    eprintln!("alternate diagonals: {wrench:?}");
    let mut workspace = OwnerPressureWorkspace::new(&[GeometryOwner::Body]).ok_or("owner")?;
    workspace.set_pressure_model(MeshPressureModel::ModifiedNewtonian);
    workspace.prepare_triangle_quadrature(&patches, flow)?;
    let integrated = workspace.staged_total()?.ok_or("staged total")?;
    eprintln!("three-point quadrature: {integrated:?}");
    // Independent composite midpoint integration over the same frozen
    // visibility fragments. No visibility decisions are repeated here.
    let axial_reference = |n: usize| -> Result<f64, crate::Error> {
        let mut sum = 0.0;
        for patch in patches.iter() {
            let [a, b, c] = patch.vertices;
            let area = patch.area / (n * n) as f64;
            for i in 0..n {
                for j in 0..n - i {
                    for offset in [1.0 / 3.0, 2.0 / 3.0] {
                        if offset > 0.5 && i + j + 1 == n {
                            continue;
                        }
                        let point = a
                            + (b - a) * ((i as f64 + offset) / n as f64)
                            + (c - a) * ((j as f64 + offset) / n as f64);
                        sum += PressurePatch::new(point, patch.normal, area)?
                            .evaluate_modified_newtonian(flow)?
                            .moment
                            .z;
                    }
                }
            }
        }
        Ok(sum)
    };
    let [coarse, medium, fine] = [8, 16, 32].map(axial_reference);
    let (coarse, medium, fine) = (coarse?, medium?, fine?);
    eprintln!("frozen-fragment axial refinement: {coarse}, {medium}, {fine}");
    assert!((fine - medium).abs() < 0.3 * (medium - coarse).abs());
    assert!((integrated.moment.z - fine).abs() < 0.01 * (wrench.moment.z - fine).abs());
    assert!(
        integrated.moment.x.hypot(integrated.moment.y)
            < 0.05 * wrench.moment.x.hypot(wrench.moment.y)
    );
    assert!(integrated.moment.z < 0.0);
    assert_eq!(workspace.staged().ok_or("owner wrench")?[0].1, integrated);
    assert!(workspace.commit());
    let invalid = PressureFlow {
        sound_speed: 100.0,
        ..flow
    };
    assert!(
        matches!(workspace.prepare_triangle_quadrature(&patches, invalid), Err(error)
        if matches!(error.downcast_ref::<PressureError>(), Some(PressureError::UnsupportedMach)))
    );
    assert!(workspace.staged().is_none());
    assert_eq!(workspace.published()[0].1, integrated);
    let geometric_moment = patches.iter().fold(Vec3::ZERO, |sum, patch| {
        sum + patch.center.cross(patch.normal) * patch.area
    });
    eprintln!("constant-pressure geometric moment: {geometric_moment:?}");
    // Constant pressure removes the nonlinear flow quadrature; its transverse
    // first area moments must cancel at roundoff relative to the axial term.
    assert!(geometric_moment.x.hypot(geometric_moment.y) < 1e-12 * geometric_moment.z.abs());
    Ok(())
}

#[test]
fn rotating_annulus_visibility_and_pressure_under_mesh_refinement()
-> Result<(), Box<dyn std::error::Error>> {
    let mut mesh = annulus(false)?;
    let mut reference: Option<(f64, f64)> = None;
    let mut coarse_axial: Option<f64> = None;
    let flow = PressureFlow {
        atmosphere: AtmosphericState::new(1.0, Vec3::ZERO)?,
        sound_speed: 340.0,
        transform: Transform::identity(),
        center_of_mass: Vec3::ZERO,
        linear_velocity: Vec3::ZERO,
        angular_velocity: Vec3::Z * (4.2 * 340.0),
    };
    let mut workspace =
        OwnerPressureWorkspace::new(&[GeometryOwner::Body]).ok_or("invalid owner")?;
    workspace.set_pressure_model(MeshPressureModel::ModifiedNewtonian);
    for level in 0..3 {
        let instances = [GeometryInstance {
            mesh: &mesh,
            local_transform: Transform::identity(),
            owner: GeometryOwner::Body,
        }];
        let geometry = PreparedGeometry::prepare(&instances, config(0.025))?;
        let result = geometry.surface_patches_with_flow(&instances, WORK, |point| {
            Ok(Vec3::Z.cross(point) * (4.2 * 340.0))
        });
        {
            let exposed = result?;
            // Zeroth and first area integrals, exact over each planar fragment.
            // No pressure law or pressure quadrature is involved.
            let area: f64 = exposed.iter().map(|patch| patch.area).sum();
            let moment = exposed.iter().fold(Vec3::ZERO, |sum, patch| {
                sum + patch.center.cross(patch.normal) * patch.area
            });
            eprintln!(
                "level={level} fragments={} area={area:.12} moment={moment:?}",
                exposed.len()
            );
            assert!(area > 0.0);
            assert!(moment.x.hypot(moment.y) < 1e-12 * moment.z.abs());
            if let Some((reference_area, reference_axial)) = reference {
                assert!((area - reference_area).abs() < 1e-12 * reference_area);
                assert!((moment.z - reference_axial).abs() < 1e-12 * reference_axial.abs());
            } else {
                reference = Some((area, moment.z));
            }
            workspace.prepare_triangle_quadrature(&exposed, flow)?;
            let wrench = workspace.staged_total()?.ok_or("missing wrench")?;
            eprintln!("level={level} pressure={wrench:?}");
            assert!(wrench.moment.z < 0.0);
            let moment_scale = wrench.moment.z.abs();
            // Half-turn symmetry requires zero net force and zero transverse
            // moment. Scale the force by the torque divided by the outer radius.
            assert!(wrench.force.length() < 1e-12 * moment_scale / 1.1);
            if let Some(coarse) = coarse_axial {
                assert!(wrench.moment.x.hypot(wrench.moment.y) < 1e-12 * moment_scale);
                // Refinement consistency, not a bound on physical model error.
                assert!((wrench.moment.z - coarse).abs() < 1e-4 * coarse.abs());
            } else {
                coarse_axial = Some(wrench.moment.z);
            }
        }
        if level == 2 {
            break;
        }
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        for indices in mesh.triangles() {
            let [a, b, c] = indices.map(|i| mesh.vertices()[i]);
            let i = vertices.len();
            vertices.extend([a, b, c, (a + b) * 0.5, (b + c) * 0.5, (c + a) * 0.5]);
            triangles.extend([
                [i, i + 3, i + 5],
                [i + 3, i + 1, i + 4],
                [i + 5, i + 4, i + 2],
                [i + 3, i + 4, i + 5],
            ]);
        }
        mesh = crate::shape::TriangleMesh::new(vertices, triangles)?;
    }
    Ok(())
}

#[test]
fn annulus_axial_opening_preserves_projected_area_under_refinement()
-> Result<(), Box<dyn std::error::Error>> {
    let mesh = annulus(false)?;
    let instances = [GeometryInstance {
        mesh: &mesh,
        local_transform: Transform::identity(),
        owner: GeometryOwner::Body,
    }];
    // Difference of the exact areas of the outer and inner regular 16-gons.
    let angle = std::f64::consts::PI / 16.0;
    let expected = 8.0 * (2.0 * angle).sin() * (1.1_f64.powi(2) - angle.cos().recip().powi(2));
    for cell_size in [0.05, 0.025, 0.0125] {
        let geometry = PreparedGeometry::prepare(&instances, config(cell_size))?;
        let exposed = geometry.exposed_surface_patches(&instances, Vec3::Z, 0.01, WORK)?;
        for z in [-0.025, 0.0, 0.025] {
            assert!(!geometry.occupied_at(Vec3::new(0.0, 0.0, z))?);
        }
        assert!(
            geometry.occupied_at(Vec3::X * 1.05)?,
            "wall missing at h={cell_size}"
        );
        let projected: f64 = exposed.iter().map(|p| p.area * p.normal.z).sum();
        assert!(
            (projected - expected).abs() < 1e-10 * expected,
            "h={cell_size}: {projected} != {expected}"
        );
    }
    Ok(())
}

#[test]
fn steady_blunt_nose_model_integral_and_budgets() -> Result<(), Box<dyn std::error::Error>> {
    let collider = Collider::new(Shape::Sphere(Sphere::new(1.0)?), Transform::identity());
    // NASA TR R-274, Appendix A: the windward hemisphere integrates to
    // Cd = Cp,max / 2, not a prediction of total drag including base pressure.
    for (mach, expected_cd) in [
        (4.06_f64, 0.896_585_022_503_271),
        (4.63, 0.901_867_771_160_288),
    ] {
        let flow = PressureFlow {
            atmosphere: AtmosphericState::new(1.0, Vec3::ZERO)?,
            sound_speed: 340.0,
            transform: Transform::identity(),
            center_of_mass: Vec3::ZERO,
            linear_velocity: Vec3::X * (340.0 * mach),
            angular_velocity: Vec3::ZERO,
        };
        let mut previous_error = f64::INFINITY;
        for (segments, cell_size) in [(8, 0.25), (16, 0.125), (32, 0.0625)] {
            let mesh = collider_mesh(
                [&collider],
                ColliderTessellation::new(segments, segments / 2 + 1, 16_384)?,
            )?;
            let instances = [GeometryInstance {
                mesh: &mesh,
                local_transform: Transform::identity(),
                owner: GeometryOwner::Body,
            }];
            let config = config(cell_size);
            let geometry = PreparedGeometry::prepare(&instances, config)?;
            let rejected = PreparedGeometry::prepare(
                &instances,
                GeometryConfig {
                    max_cells: 1,
                    ..config
                },
            );
            assert!(
                matches!(rejected, Err(e) if e.downcast_ref::<GeometryError>() == Some(&GeometryError::CellBudgetExceeded))
            );
            let rejected = geometry.exposed_surface_patches(&instances, Vec3::X, 0.01, 1);
            assert!(
                matches!(rejected, Err(e) if e.downcast_ref::<GeometryError>() == Some(&GeometryError::WorkBudgetExceeded))
            );
            let patches = geometry.exposed_surface_patches(&instances, Vec3::X, 0.01, WORK)?;
            let wrench = evaluate_prepared_mesh_modified_newtonian(&patches, flow)?;
            let scale = 0.5 * flow.linear_velocity.length_squared() * std::f64::consts::PI;
            let cd = -wrench.force.x / scale;
            let error = (cd - expected_cd).abs();
            eprintln!("M={mach} segments={segments} h={cell_size} Cd={cd:.9} error={error:.9}");
            assert!(error < previous_error);
            previous_error = error;
            // Roundoff-scale imbalance relative to the 1 m radius and qA.
            assert!((wrench.force + Vec3::X * (cd * scale)).length() / scale < 1e-10);
            assert!(wrench.moment.length() / scale < 1e-10);
        }
        assert!(previous_error < 0.03);
    }
    Ok(())
}
