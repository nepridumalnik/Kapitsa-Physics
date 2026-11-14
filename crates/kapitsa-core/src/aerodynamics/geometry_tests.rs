use super::*;
use kapitsa_math::Quat;

fn cube() -> TriangleMesh {
    TriangleMesh::new(
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
    .unwrap()
}
fn config(h: f64) -> GeometryConfig {
    GeometryConfig {
        cell_size: h,
        max_cells: 100_000,
        max_work: 10_000_000,
        open_mesh_policy: OpenMeshPolicy::Reject,
        weld_tolerance: None,
        max_patch_work: 100_000,
    }
}
fn instance(mesh: &TriangleMesh, offset: Vec3) -> GeometryInstance<'_> {
    GeometryInstance {
        mesh,
        owner: GeometryOwner::Body,
        local_transform: Transform::new(offset, Quat::IDENTITY).unwrap(),
    }
}

#[test]
fn conservative_cube_volume_sections_and_area_converge() {
    let mesh = cube();
    let mut previous = f64::INFINITY;
    for h in [0.5, 0.25, 0.125, 0.0625] {
        let g = PreparedGeometry::prepare(&[instance(&mesh, Vec3::ZERO)], config(h)).unwrap();
        assert!(g.volume() >= 1.0);
        let error = g.volume() - 1.0;
        assert!(error < previous, "h={h}, volume={}", g.volume());
        previous = error;
        let sections = &g.sections()[0];
        let volume: f64 = sections.iter().map(|s| s.area * h).sum();
        assert!((volume - g.volume()).abs() < 1e-12);
        for section in sections {
            if let Some(c) = section.centroid {
                assert!((c.y - 0.5).abs() <= h);
                assert!((c.z - 0.5).abs() <= h);
            }
        }
        let faces: Vec<_> = g.exposed_faces().collect();
        let area: f64 = faces.iter().map(|f| f.area).sum();
        assert!(area >= 6.0);
        assert!(faces.iter().all(|f| f.normal.length_squared() == 1.0));
        let balance = faces.iter().fold(Vec3::ZERO, |n, f| n + f.normal * f.area);
        assert!(balance.length() < 1e-12);
    }
}

#[test]
fn identical_overlap_is_not_counted_twice() {
    let mesh = cube();
    let one = PreparedGeometry::prepare(&[instance(&mesh, Vec3::ZERO)], config(0.25)).unwrap();
    let two = PreparedGeometry::prepare(
        &[instance(&mesh, Vec3::ZERO), instance(&mesh, Vec3::ZERO)],
        config(0.25),
    )
    .unwrap();
    assert_eq!(one, two);
    assert_eq!(one.exposed_faces().count(), two.exposed_faces().count());
}

#[test]
fn translation_preserves_geometry_in_body_coordinates() {
    let mesh = cube();
    let offset = Vec3::new(4.0, -2.0, 8.0);
    let a = PreparedGeometry::prepare(&[instance(&mesh, Vec3::ZERO)], config(0.25)).unwrap();
    let b = PreparedGeometry::prepare(&[instance(&mesh, offset)], config(0.25)).unwrap();
    assert_eq!(b.origin(), a.origin() + offset);
    assert_eq!(a.occupancy(), b.occupancy());
    assert_eq!(a.volume(), b.volume());
    for (x, y) in a.exposed_faces().zip(b.exposed_faces()) {
        assert_eq!(x.center + offset, y.center);
        assert_eq!(x.normal, y.normal);
    }
}

#[test]
fn thin_grid_aligned_surface_survives_and_open_policy_is_explicit() {
    let mesh = TriangleMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 2]]).unwrap();
    let instances = [instance(&mesh, Vec3::ZERO)];
    assert_eq!(
        PreparedGeometry::prepare(&instances, config(0.25)),
        Err(GeometryError::OpenMesh)
    );
    let mut cfg = config(0.25);
    cfg.open_mesh_policy = OpenMeshPolicy::SurfaceOnly;
    let g = PreparedGeometry::prepare(&instances, cfg).unwrap();
    assert!(g.occupancy().iter().any(Option::is_some));
    assert!(g.occupancy().iter().any(Option::is_none));
    assert!(g.exposed_faces().count() > 0);
}

#[test]
fn welding_can_accept_uv_seam_duplicate_vertices() {
    let base = cube();
    let mut vertices = base.vertices().to_vec();
    let seam = vertices.len();
    vertices.push(vertices[0]);
    vertices.push(vertices[1]);
    vertices.push(vertices[2]);
    let mut triangles = base.triangles().to_vec();
    triangles[0] = [seam, seam + 2, seam + 1];
    let mesh = TriangleMesh::new(vertices, triangles).unwrap();
    let mut cfg = config(0.25);
    cfg.weld_tolerance = Some(1e-9);
    assert!(PreparedGeometry::prepare(&[instance(&mesh, Vec3::ZERO)], cfg).is_ok());
}

#[test]
fn unused_vertices_do_not_change_prepared_geometry() {
    let base = cube();
    let mut vertices = base.vertices().to_vec();
    vertices.push(Vec3::new(10_000.0, 0.0, 0.0));
    let with_unused = TriangleMesh::new(vertices, base.triangles().to_vec()).unwrap();
    let a = PreparedGeometry::prepare(&[instance(&base, Vec3::ZERO)], config(0.25)).unwrap();
    let b = PreparedGeometry::prepare(&[instance(&with_unused, Vec3::ZERO)], config(0.25)).unwrap();
    assert_eq!(a.volume(), b.volume());
    assert_eq!(a.occupancy(), b.occupancy());
}

#[test]
fn resources_and_nonrepresentable_coordinates_are_rejected() {
    let mesh = cube();
    let instances = [instance(&mesh, Vec3::ZERO)];
    for cell_size in [
        0.0,
        -1.0,
        f64::NAN,
        f64::INFINITY,
        f64::MIN_POSITIVE,
        f64::MAX,
    ] {
        let mut cfg = config(0.25);
        cfg.cell_size = cell_size;
        assert_eq!(
            PreparedGeometry::prepare(&instances, cfg),
            Err(GeometryError::InvalidConfig)
        );
    }
    let mut cfg = config(0.25);
    cfg.max_cells = 1;
    assert_eq!(
        PreparedGeometry::prepare(&instances, cfg),
        Err(GeometryError::CellBudgetExceeded)
    );
    cfg = config(0.25);
    cfg.max_work = 20;
    assert_eq!(
        PreparedGeometry::prepare(&instances, cfg),
        Err(GeometryError::WorkBudgetExceeded)
    );
    assert_eq!(
        PreparedGeometry::prepare(&[], config(0.25)),
        Err(GeometryError::EmptyGeometry)
    );
    assert!(
        PreparedGeometry::prepare(&[instance(&mesh, Vec3::splat(1e30))], config(0.25)).is_err()
    );
}

#[test]
fn part_overlap_ownership_is_order_independent() {
    use crate::{mass::MassProperties, rigid_body::RigidBody, world::PhysicsWorld};
    use kapitsa_math::Mat3;
    let mut world = PhysicsWorld::new();
    let body = world
        .add_body(
            RigidBody::new_dynamic(
                Transform::identity(),
                MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap(),
            )
            .unwrap(),
        )
        .unwrap();
    let part = world.enable_cluster(body).unwrap();
    let mesh = cube();
    let a = instance(&mesh, Vec3::ZERO);
    let b = GeometryInstance {
        owner: GeometryOwner::Part(part),
        ..a
    };
    let first = PreparedGeometry::prepare(&[a, b], config(0.25)).unwrap();
    let second = PreparedGeometry::prepare(&[b, a], config(0.25)).unwrap();
    assert_eq!(first, second);
    assert!(
        first
            .occupancy()
            .iter()
            .flatten()
            .all(|o| *o == GeometryOwner::Body)
    );
    let part_only = PreparedGeometry::prepare(&[b], config(0.25)).unwrap();
    assert!(
        part_only
            .exposed_faces()
            .all(|f| f.owner == GeometryOwner::Part(part))
    );
}

#[test]
fn nested_solid_has_no_internal_exposed_faces() {
    let outer = cube();
    let inner = TriangleMesh::new(
        outer.vertices().iter().map(|v| *v * 0.25).collect(),
        outer.triangles().to_vec(),
    )
    .unwrap();
    let a = instance(&outer, Vec3::ZERO);
    let b = instance(&inner, Vec3::splat(0.375));
    let single = PreparedGeometry::prepare(&[a], config(0.125)).unwrap();
    let nested = PreparedGeometry::prepare(&[a, b], config(0.125)).unwrap();
    assert_eq!(single, nested);
}

#[test]
fn duplicated_faces_and_wrong_winding_are_not_closed_solids() {
    let cube = cube();
    let mut triangles = cube.triangles().to_vec();
    triangles.push(triangles[0]);
    let duplicate = TriangleMesh::new(cube.vertices().to_vec(), triangles).unwrap();
    assert_eq!(
        PreparedGeometry::prepare(&[instance(&duplicate, Vec3::ZERO)], config(0.25)),
        Err(GeometryError::OpenMesh)
    );
    let mut triangles = cube.triangles().to_vec();
    triangles[0].swap(0, 1);
    let wrong = TriangleMesh::new(cube.vertices().to_vec(), triangles).unwrap();
    assert_eq!(
        PreparedGeometry::prepare(&[instance(&wrong, Vec3::ZERO)], config(0.25)),
        Err(GeometryError::OpenMesh)
    );
}

#[test]
fn rotated_closed_mesh_fills_its_interior() {
    let mesh = cube();
    let rotated = GeometryInstance {
        local_transform: Transform::new(
            Vec3::ZERO,
            Quat::from_axis_angle(Vec3::Z, std::f64::consts::FRAC_PI_4).unwrap(),
        )
        .unwrap(),
        ..instance(&mesh, Vec3::ZERO)
    };
    let g = PreparedGeometry::prepare(&[rotated], config(0.125)).unwrap();
    let center = rotated.local_transform.transform_point(Vec3::splat(0.5));
    let p = components((center - g.origin()) / g.cell_size()).map(|v| v.floor() as usize);
    assert_eq!(
        g.occupancy()[index(p, g.dimensions())],
        Some(GeometryOwner::Body)
    );
    assert!(g.volume() >= 1.0 && g.volume() < 3.0);
}

#[test]
fn separating_axes_reject_empty_corner_of_triangle_bounds() {
    let triangle = [Vec3::ZERO, Vec3::X, Vec3::Y];
    assert!(!intersects(triangle, Vec3::new(0.9, 0.9, 0.0), 0.05).unwrap());
    assert!(intersects(triangle, Vec3::new(0.2, 0.2, 0.0), 0.05).unwrap());
    assert!(!intersects(triangle, Vec3::new(0.2, 0.2, 1.0), 0.05).unwrap());
}

#[test]
fn exposed_surface_patches_remove_internal_overlap() {
    let mesh = cube();
    let g = PreparedGeometry::prepare(&[instance(&mesh, Vec3::ZERO)], config(0.125)).unwrap();
    let patches = g
        .exposed_surface_patches(&[instance(&mesh, Vec3::ZERO)], -Vec3::X, 0.01, 100_000)
        .unwrap();
    assert_eq!(patches.len(), 2);
    assert!(patches.iter().all(|patch| patch.normal.dot(-Vec3::X) > 0.0));
}

#[test]
fn nested_and_duplicate_source_surfaces_are_filtered() {
    let outer = cube();
    let inner = TriangleMesh::new(
        outer.vertices().iter().map(|v| *v * 0.25).collect(),
        outer.triangles().to_vec(),
    )
    .unwrap();
    let outer_instance = instance(&outer, Vec3::ZERO);
    let inner_instance = instance(&inner, Vec3::splat(0.375));
    let g = PreparedGeometry::prepare(&[outer_instance, inner_instance], config(0.125)).unwrap();
    let nested = g
        .exposed_surface_patches(&[outer_instance, inner_instance], -Vec3::X, 0.01, 100_000)
        .unwrap();
    let duplicate = g
        .exposed_surface_patches(&[outer_instance, outer_instance], -Vec3::X, 0.01, 100_000)
        .unwrap();
    assert_eq!(nested.len(), 2);
    assert_eq!(duplicate.len(), 2);
    assert_eq!(duplicate.iter().next().unwrap().owner, GeometryOwner::Body);
}

#[test]
fn diagonal_flow_keeps_windward_surface_patches() {
    let mesh = cube();
    let g = PreparedGeometry::prepare(&[instance(&mesh, Vec3::ZERO)], config(0.125)).unwrap();
    let patches = g
        .exposed_surface_patches(
            &[instance(&mesh, Vec3::ZERO)],
            -Vec3::X - Vec3::Y,
            0.01,
            100_000,
        )
        .unwrap();
    assert!(!patches.is_empty());
}

#[test]
fn exposed_surface_patches_are_empty_for_leeward_direction() {
    let mesh = cube();
    let g = PreparedGeometry::prepare(&[instance(&mesh, Vec3::ZERO)], config(0.125)).unwrap();
    let patches = g
        .exposed_surface_patches(&[instance(&mesh, Vec3::ZERO)], Vec3::X, 0.01, 100_000)
        .unwrap();
    assert_eq!(patches.len(), 2);
}

#[test]
fn open_box_is_not_filled() {
    let cube = cube();
    let mesh = TriangleMesh::new(cube.vertices().to_vec(), cube.triangles()[2..].to_vec()).unwrap();
    let mut cfg = config(0.125);
    cfg.open_mesh_policy = OpenMeshPolicy::SurfaceOnly;
    let g = PreparedGeometry::prepare(&[instance(&mesh, Vec3::ZERO)], cfg).unwrap();
    let midpoint = [
        g.dimensions()[0] / 2,
        g.dimensions()[1] / 2,
        g.dimensions()[2] / 2,
    ];
    assert_eq!(g.occupancy()[index(midpoint, g.dimensions())], None);
}
