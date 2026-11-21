use super::*;
use kapitsa_math::Quat;

fn step_cause(result: &Result<(), crate::Error>) -> Option<&crate::world::StepError> {
    result.as_ref().err().and_then(|error| error.downcast_ref())
}

fn aero_cause(
    result: &Result<(), crate::Error>,
) -> Option<&crate::aerodynamics::AerodynamicsError> {
    match step_cause(result)? {
        crate::world::StepError::Aerodynamics { cause, .. } => cause.downcast_ref(),
        _ => None,
    }
}
fn aero_hull(result: &Result<(), crate::Error>) -> Option<&crate::aerodynamics::hull::HullError> {
    match aero_cause(result)? {
        crate::aerodynamics::AerodynamicsError::Hull(cause) => cause.downcast_ref(),
        _ => None,
    }
}

fn visibility_triangle(a: Vec3, b: Vec3, c: Vec3) -> SurfaceTriangle {
    let normal = (b - a).cross(c - a).try_normalize().unwrap();
    SurfaceTriangle {
        vertices: [a, b, c],
        normal,
        area: (b - a).cross(c - a).length() * 0.5,
        owner: GeometryOwner::Body,
    }
}

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

#[test]
fn oriented_voxelization_preserves_a_sealed_hollow_cavity() -> Result<(), crate::Error> {
    let shell = cube();
    let vertices = shell
        .vertices()
        .iter()
        .copied()
        .chain(
            shell
                .vertices()
                .iter()
                .map(|v| Vec3::splat(0.25) + *v * 0.5),
        )
        .collect();
    let faces = shell
        .triangles()
        .iter()
        .copied()
        .chain(
            shell
                .triangles()
                .iter()
                .map(|&[a, b, c]| [a + 8, c + 8, b + 8]),
        )
        .collect();
    let mesh = TriangleMesh::new(vertices, faces)?;
    let inputs = [instance(&mesh, Vec3::ZERO)];
    let settings = config(0.0625);
    let hollow = PreparedGeometry::prepare_with_oriented_cavities(&inputs, settings)?;
    let solid = PreparedGeometry::prepare(&inputs, settings)?;
    let center = Vec3::splat(0.5);
    assert!(!hollow.occupied_at(center)?);
    assert!(solid.occupied_at(center)?);
    assert!(hollow.occupied_at(Vec3::new(0.125, 0.5, 0.5))?);
    assert!(hollow.volume() < solid.volume());
    let bounded = GeometryConfig {
        max_work: 180_000,
        ..settings
    };
    assert_eq!(
        PreparedGeometry::prepare_with_oriented_cavities(&inputs, bounded)?,
        hollow
    );
    Ok(())
}
fn instance(mesh: &TriangleMesh, offset: Vec3) -> GeometryInstance<'_> {
    GeometryInstance {
        mesh,
        owner: GeometryOwner::Body,
        local_transform: Transform::new(offset, Quat::IDENTITY).unwrap(),
    }
}

#[test]
fn triangle_grid_candidates_are_conservative_for_axis_and_diagonal_sweeps() {
    let triangles = vec![
        visibility_triangle(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.2, 0.0, 0.0),
            Vec3::new(0.0, 0.2, 0.0),
        ),
        visibility_triangle(
            Vec3::new(0.8, 0.8, 0.8),
            Vec3::new(1.0, 0.8, 0.8),
            Vec3::new(0.8, 1.0, 0.8),
        ),
        visibility_triangle(
            Vec3::new(0.4, 0.4, 0.4),
            Vec3::new(0.6, 0.4, 0.4),
            Vec3::new(0.4, 0.6, 0.4),
        ),
    ];
    let grid = TriangleGrid::build(&triangles, 10_000).unwrap();
    assert!(matches!(
        TriangleGrid::build(&triangles, grid.build_work - 1),
        Err(error) if error.downcast_ref::<GeometryError>() == Some(&GeometryError::WorkBudgetExceeded)
    ));
    assert!(TriangleGrid::build(&triangles, grid.build_work).is_ok());
    let mut marks = vec![0; triangles.len()];
    for (stamp, direction) in [
        Vec3::X,
        -Vec3::X,
        Vec3::Y,
        Vec3::Z,
        Vec3::new(1.0, -1.0, 1.0),
    ]
    .into_iter()
    .enumerate()
    {
        let (candidates, _) = grid
            .candidates(
                &triangles[1],
                direction,
                &mut marks,
                stamp as u32 + 1,
                10_000,
            )
            .unwrap();
        assert!(triangles.iter().enumerate().all(|(index, _)| {
            // Every exhaustive swept candidate is allowed; the index must not omit
            // any triangle whose AABB intersects the axis-aligned swept bounds.
            let candidate = &triangles[index];
            let source_min = triangles[1]
                .vertices
                .iter()
                .fold(Vec3::splat(f64::INFINITY), |v, &p| v.min(p));
            let source_max = triangles[1]
                .vertices
                .iter()
                .fold(Vec3::splat(f64::NEG_INFINITY), |v, &p| v.max(p));
            let global_min = triangles
                .iter()
                .flat_map(|t| t.vertices)
                .fold(Vec3::splat(f64::INFINITY), |v, p| v.min(p));
            let global_max = triangles
                .iter()
                .flat_map(|t| t.vertices)
                .fold(Vec3::splat(f64::NEG_INFINITY), |v, p| v.max(p));
            let direction = components(direction);
            let (source_min, source_max, global_min, global_max) = (
                components(source_min),
                components(source_max),
                components(global_min),
                components(global_max),
            );
            let lower: [f64; 3] = std::array::from_fn(|axis| {
                if direction[axis] < 0.0 {
                    global_min[axis]
                } else {
                    source_min[axis]
                }
            });
            let upper: [f64; 3] = std::array::from_fn(|axis| {
                if direction[axis] > 0.0 {
                    global_max[axis]
                } else {
                    source_max[axis]
                }
            });
            let candidate_min = candidate
                .vertices
                .iter()
                .fold(Vec3::splat(f64::INFINITY), |v, &p| v.min(p));
            let candidate_max = candidate
                .vertices
                .iter()
                .fold(Vec3::splat(f64::NEG_INFINITY), |v, &p| v.max(p));
            let (candidate_max, candidate_min, lower, upper) = (
                components(candidate_max),
                components(candidate_min),
                lower,
                upper,
            );
            let intersects = (0..3).all(|axis| {
                candidate_max[axis] >= lower[axis] && candidate_min[axis] <= upper[axis]
            });
            !intersects || candidates.contains(&index)
        }));
    }
}

#[test]
fn sort_work_estimate_checks_power_of_two_boundaries_and_overflow() {
    assert_eq!(sort_work_estimate(0), Some(0));
    assert_eq!(sort_work_estimate(1), Some(2));
    assert_eq!(sort_work_estimate(2), Some(6));
    assert_eq!(sort_work_estimate(3), Some(9));
    assert_eq!(sort_work_estimate(4), Some(16));
    assert_eq!(sort_work_estimate(usize::MAX), None);
}

#[test]
fn triangle_preprocessing_reserves_surface_transform_work() {
    const MAX_TRIANGLES: usize = 65_536;
    for count in 0..=MAX_TRIANGLES {
        let reserved = sort_work_estimate(count)
            .unwrap()
            .checked_mul(TRIANGLE_PREPARATION_WORK)
            .unwrap();
        assert!(reserved >= count * SURFACE_TRIANGLE_WORK);
    }
}

#[test]
fn sort_work_estimate_is_monotone_for_grid_sized_inputs() {
    const MAX_ENTRIES: usize = 65_536;
    let mut previous = 0;
    for len in 0..=MAX_ENTRIES {
        let estimate = sort_work_estimate(len).unwrap();
        assert!(estimate >= previous, "estimate decreased at {len}");
        previous = estimate;
    }
}

#[test]
fn budgeted_sort_charges_swap_before_mutating_values() {
    let mut values = [1, 2];
    let mut work = 0;
    assert_eq!(
        budgeted_sort(&mut values, &mut work, 1, Ord::cmp),
        Err(crate::Error::from(GeometryError::WorkBudgetExceeded))
    );
    assert_eq!(values, [1, 2]);
    assert_eq!(work, 1);
    let mut work = 0;
    budgeted_sort(&mut values, &mut work, 3, Ord::cmp).unwrap();
    assert_eq!(values, [1, 2]);
    assert_eq!(work, 3);
}

#[test]
fn budgeted_sort_counts_comparisons_and_rejects_exhaustion() {
    let source = [4, 1, 3, 1, 2];
    let mut values = source;
    let mut work = 0;
    budgeted_sort(&mut values, &mut work, 1_000, Ord::cmp).unwrap();
    assert_eq!(values, [1, 1, 2, 3, 4]);
    assert!(work > 0);
    let required = work;
    let mut values = source;
    let mut work = 0;
    assert_eq!(
        budgeted_sort(&mut values, &mut work, required - 1, Ord::cmp),
        Err(crate::Error::from(GeometryError::WorkBudgetExceeded))
    );
    let mut values = source;
    let mut work = 0;
    budgeted_sort(&mut values, &mut work, required, Ord::cmp).unwrap();
    assert_eq!(values, [1, 1, 2, 3, 4]);
}

#[test]
fn budgeted_sort_matches_standard_sort_for_varied_inputs() {
    const MAX_LEN: usize = 128;
    const CASES: u64 = 8;
    const MULTIPLIER: u64 = 6_364_136_223_846_793_005;
    const INCREMENT: u64 = 1_442_695_040_888_963_407;
    const VALUE_COUNT: u64 = 17;
    const WORK_LIMIT: usize = 100_000;
    for len in 0..=MAX_LEN {
        for seed in 0..CASES {
            let mut state = seed;
            let input: Vec<_> = (0..len)
                .map(|_| {
                    state = state.wrapping_mul(MULTIPLIER).wrapping_add(INCREMENT);
                    state % VALUE_COUNT
                })
                .collect();
            let mut expected = input.clone();
            expected.sort_unstable();
            let mut actual = input.clone();
            let mut work = 0;
            budgeted_sort(&mut actual, &mut work, WORK_LIMIT, Ord::cmp).unwrap();
            assert_eq!(actual, expected, "length {len}, seed {seed}");
            if work > 0 {
                let mut retry = input.clone();
                let mut charged = 0;
                assert_eq!(
                    budgeted_sort(&mut retry, &mut charged, work - 1, Ord::cmp),
                    Err(crate::Error::from(GeometryError::WorkBudgetExceeded))
                );
                let mut retry = input;
                let mut charged = 0;
                budgeted_sort(&mut retry, &mut charged, work, Ord::cmp).unwrap();
                assert_eq!(retry, expected);
            }
        }
    }
}

#[test]
fn surface_work_charge_rejects_overflow_and_preserves_previous_value() {
    let mut work = 4;
    assert_eq!(charge_surface_work(&mut work, 1, 5), Ok(()));
    assert_eq!(work, 5);
    assert_eq!(
        charge_surface_work(&mut work, 1, 5),
        Err(crate::Error::from(GeometryError::WorkBudgetExceeded))
    );
    assert_eq!(work, 5);
    let mut overflow = usize::MAX;
    assert_eq!(
        charge_surface_work(&mut overflow, 1, usize::MAX),
        Err(crate::Error::from(GeometryError::WorkBudgetExceeded))
    );
    assert_eq!(overflow, usize::MAX);
}

#[test]
fn surface_work_enforces_hard_ceiling_even_with_larger_caller_budget() {
    const HARD_LIMIT: usize = 100_000_000;
    let mut work = HARD_LIMIT - 1;
    assert_eq!(charge_surface_work(&mut work, 1, usize::MAX), Ok(()));
    assert_eq!(work, HARD_LIMIT);
    assert_eq!(
        charge_surface_work(&mut work, 1, usize::MAX),
        Err(crate::Error::from(GeometryError::WorkBudgetExceeded))
    );
    assert_eq!(work, HARD_LIMIT);
    assert_eq!(
        check_surface_work(Some(0), 0),
        Err(crate::Error::from(GeometryError::WorkBudgetExceeded))
    );
    assert_eq!(
        check_surface_work(None, usize::MAX),
        Err(crate::Error::from(GeometryError::WorkBudgetExceeded))
    );
}

#[test]
fn candidate_mark_allocation_requires_work_budget_up_front() {
    const MARK_COUNT: usize = 4;
    const INITIAL_WORK: usize = 3;
    let mut work = INITIAL_WORK;
    assert_eq!(
        allocate_candidate_marks(MARK_COUNT, &mut work, INITIAL_WORK + MARK_COUNT - 1),
        Err(crate::Error::from(GeometryError::WorkBudgetExceeded))
    );
    assert_eq!(work, INITIAL_WORK);
    assert_eq!(
        allocate_candidate_marks(MARK_COUNT, &mut work, INITIAL_WORK + MARK_COUNT).unwrap(),
        vec![0; MARK_COUNT]
    );
    assert_eq!(work, INITIAL_WORK + MARK_COUNT);
}

#[test]
fn grid_build_estimate_reserves_cell_buffers_prefix_scan_and_index_entries() {
    const ENTRIES: usize = 4;
    const CELLS: usize = 3;
    const SCANS: usize = 6;
    let expected = 3 * ENTRIES + 3 * CELLS + 1 + SCANS;
    assert_eq!(
        grid_build_work_estimate(ENTRIES, CELLS, SCANS),
        Some(expected)
    );
    assert_eq!(grid_build_work_estimate(0, usize::MAX, 0), None);
    assert_eq!(grid_build_work_estimate(usize::MAX, 0, 0), None);
    assert_eq!(grid_build_work_estimate(0, 0, usize::MAX), None);
}

#[test]
fn triangle_grid_scatter_preserves_bucket_membership_and_order() -> Result<(), crate::Error> {
    let triangles = [
        visibility_triangle(Vec3::ZERO, Vec3::Y * 0.5, Vec3::Z * 0.5),
        visibility_triangle(Vec3::ZERO, Vec3::new(2.0, 0.0, 2.0), Vec3::Y * 2.0),
        visibility_triangle(
            Vec3::new(2.0, 1.5, 1.5),
            Vec3::new(2.0, 2.0, 1.5),
            Vec3::new(2.0, 1.5, 2.0),
        ),
    ];
    // Four triangle scans, three passes per grid cell and entry, and the
    // extra prefix offset. The middle triangle's AABB spans all eight cells.
    const WORK: usize = 4 * 3 + 3 * 8 + 3 * 10 + 1;
    let grid = TriangleGrid::build(&triangles, WORK)?;
    assert_eq!(grid.build_work, WORK);
    assert_eq!(grid.dimensions, [2, 2, 2]);
    assert_eq!(grid.offsets, [0, 2, 3, 4, 5, 6, 7, 8, 10]);
    assert_eq!(grid.triangles, [0, 1, 1, 1, 1, 1, 1, 1, 1, 2]);
    assert!(matches!(
        TriangleGrid::build(&triangles, WORK - 1),
        Err(error) if error.downcast_ref::<GeometryError>() == Some(&GeometryError::WorkBudgetExceeded)
    ));
    Ok(())
}

#[test]
fn triangle_grid_build_accounts_for_input_scans() {
    const TRIANGLE_COUNT: usize = 2;
    const SCAN_WORK: usize = TRIANGLE_COUNT * 3;
    let triangle = visibility_triangle(Vec3::ZERO, Vec3::Y, Vec3::Z);
    let triangles = [triangle; TRIANGLE_COUNT];
    assert!(matches!(
        TriangleGrid::build(&triangles, SCAN_WORK - 1),
        Err(error) if error.downcast_ref::<GeometryError>() == Some(&GeometryError::WorkBudgetExceeded)
    ));
    let required = TriangleGrid::build(&triangles, 10_000).unwrap().build_work;
    assert!(required > SCAN_WORK);
    assert!(matches!(
        TriangleGrid::build(&triangles, required - 1),
        Err(error) if error.downcast_ref::<GeometryError>() == Some(&GeometryError::WorkBudgetExceeded)
    ));
    assert_eq!(
        TriangleGrid::build(&triangles, required)
            .unwrap()
            .build_work,
        required
    );
}

#[test]
fn triangle_grid_counts_empty_buckets_toward_query_budget() {
    let grid = TriangleGrid {
        origin: Vec3::ZERO,
        cell_size: Vec3::ONE,
        dimensions: [3, 1, 1],
        offsets: vec![0; 4],
        triangles: vec![],
        build_work: 0,
    };
    let source = visibility_triangle(Vec3::ZERO, Vec3::Y * 0.1, Vec3::Z * 0.1);
    assert_eq!(
        grid.candidates(&source, Vec3::X, &mut [], 1, 2)
            .unwrap_err(),
        crate::Error::from(GeometryError::WorkBudgetExceeded)
    );
    assert_eq!(
        grid.candidates(&source, Vec3::X, &mut [], 1, 3).unwrap().1,
        3
    );
}

#[test]
fn triangle_grid_charges_repeated_entries_even_when_deduplicated() {
    const CELL_COUNT: usize = 3;
    const QUERY_WORK: usize = CELL_COUNT * 2;
    let grid = TriangleGrid {
        origin: Vec3::ZERO,
        cell_size: Vec3::ONE,
        dimensions: [CELL_COUNT, 1, 1],
        offsets: (0..=CELL_COUNT).collect(),
        triangles: vec![0; CELL_COUNT],
        build_work: 0,
    };
    let source = visibility_triangle(Vec3::ZERO, Vec3::Y * 0.1, Vec3::Z * 0.1);
    let mut marks = [0];
    assert_eq!(
        grid.candidates(&source, Vec3::X, &mut marks, 1, QUERY_WORK - 1)
            .unwrap_err(),
        crate::Error::from(GeometryError::WorkBudgetExceeded)
    );
    let (candidates, work) = grid
        .candidates(&source, Vec3::X, &mut marks, 2, QUERY_WORK)
        .unwrap();
    assert_eq!(candidates, vec![0]);
    assert_eq!(work, QUERY_WORK);
}

#[test]
fn triangle_grid_budget_stops_before_marking_next_candidate() {
    const FIRST_CANDIDATE_WORK: usize = 2;
    const ALL_CANDIDATES_WORK: usize = 3;
    let grid = TriangleGrid {
        origin: Vec3::ZERO,
        cell_size: Vec3::ONE,
        dimensions: [1, 1, 1],
        offsets: vec![0, 2],
        triangles: vec![0, 1],
        build_work: 0,
    };
    let source = visibility_triangle(Vec3::ZERO, Vec3::Y * 0.1, Vec3::Z * 0.1);
    let mut marks = [0; 2];
    assert_eq!(
        grid.candidates(&source, Vec3::X, &mut marks, 1, FIRST_CANDIDATE_WORK),
        Err(crate::Error::from(GeometryError::WorkBudgetExceeded))
    );
    assert_eq!(marks, [1, 0]);
    let (result, work) = grid
        .candidates(&source, Vec3::X, &mut marks, 2, ALL_CANDIDATES_WORK)
        .unwrap();
    assert_eq!(result, [0, 1]);
    assert_eq!(work, ALL_CANDIDATES_WORK);
}

#[test]
fn triangle_grid_rejects_query_after_work_limit_before_scanning_all_buckets() {
    let triangles = vec![
        visibility_triangle(Vec3::ZERO, Vec3::X * 0.2, Vec3::Y * 0.2),
        visibility_triangle(
            Vec3::new(0.1, 0.1, 0.1),
            Vec3::new(0.3, 0.1, 0.1),
            Vec3::new(0.1, 0.3, 0.1),
        ),
    ];
    let grid = TriangleGrid::build(&triangles, 10_000).unwrap();
    let mut marks = vec![0; triangles.len()];
    assert_eq!(
        grid.candidates(&triangles[0], Vec3::X, &mut marks, 1, 1)
            .unwrap_err(),
        crate::Error::from(GeometryError::WorkBudgetExceeded)
    );
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
        Err(crate::Error::from(GeometryError::OpenMesh))
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
            Err(crate::Error::from(GeometryError::InvalidConfig))
        );
    }
    let mut cfg = config(0.25);
    cfg.max_cells = 1;
    assert_eq!(
        PreparedGeometry::prepare(&instances, cfg),
        Err(crate::Error::from(GeometryError::CellBudgetExceeded))
    );
    cfg = config(0.25);
    cfg.max_work = 20;
    assert_eq!(
        PreparedGeometry::prepare(&instances, cfg),
        Err(crate::Error::from(GeometryError::WorkBudgetExceeded))
    );
    assert_eq!(
        PreparedGeometry::prepare(&[], config(0.25)),
        Err(crate::Error::from(GeometryError::EmptyGeometry))
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
    let first_patches = first
        .exposed_surface_patches(&[a, b], Vec3::X, 0.01, 100_000)
        .unwrap();
    let second_patches = second
        .exposed_surface_patches(&[b, a], Vec3::X, 0.01, 100_000)
        .unwrap();
    assert_eq!(first_patches, second_patches);
    assert_eq!(first_patches.len(), 2);
    assert!(first_patches.iter().all(|p| p.owner == GeometryOwner::Body));
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
        Err(crate::Error::from(GeometryError::OpenMesh))
    );
    let mut triangles = cube.triangles().to_vec();
    triangles[0].swap(0, 1);
    let wrong = TriangleMesh::new(cube.vertices().to_vec(), triangles).unwrap();
    assert_eq!(
        PreparedGeometry::prepare(&[instance(&wrong, Vec3::ZERO)], config(0.25)),
        Err(crate::Error::from(GeometryError::OpenMesh))
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
        .exposed_surface_patches(&[instance(&mesh, Vec3::ZERO)], Vec3::X, 0.01, 100_000)
        .unwrap();
    assert_eq!(patches.len(), 2);
    assert!(patches.iter().all(|patch| patch.normal.dot(Vec3::X) > 0.0));
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
        .exposed_surface_patches(&[outer_instance, inner_instance], Vec3::X, 0.01, 100_000)
        .unwrap();
    let duplicate = g
        .exposed_surface_patches(&[outer_instance, outer_instance], Vec3::X, 0.01, 100_000)
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
            Vec3::X + Vec3::Y,
            0.01,
            100_000,
        )
        .unwrap();
    assert_eq!(patches.len(), 4);
    assert_eq!(patches.iter().filter(|p| p.normal == Vec3::X).count(), 2);
    assert_eq!(patches.iter().filter(|p| p.normal == Vec3::Y).count(), 2);
}

#[test]
fn exposed_surface_patches_keep_only_upstream_windward_direction() {
    let mesh = cube();
    let g = PreparedGeometry::prepare(&[instance(&mesh, Vec3::ZERO)], config(0.125)).unwrap();
    let upstream = g
        .exposed_surface_patches(&[instance(&mesh, Vec3::ZERO)], Vec3::X, 0.01, 100_000)
        .unwrap();
    let downstream = g
        .exposed_surface_patches(&[instance(&mesh, Vec3::ZERO)], -Vec3::X, 0.01, 100_000)
        .unwrap();
    assert_eq!(upstream.len(), 2);
    assert_eq!(downstream.len(), 2);
    assert!(upstream.iter().all(|patch| patch.normal.dot(Vec3::X) > 0.0));
    assert!(
        downstream
            .iter()
            .all(|patch| patch.normal.dot(-Vec3::X) > 0.0)
    );
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

fn surface_geometry(instances: &[GeometryInstance<'_>]) -> PreparedGeometry {
    let mut cfg = config(0.25);
    cfg.open_mesh_policy = OpenMeshPolicy::SurfaceOnly;
    PreparedGeometry::prepare(instances, cfg).unwrap()
}

#[test]
fn upstream_blocker_shadows_but_side_by_side_does_not_and_backfaces_block() {
    let plate = TriangleMesh::new(vec![Vec3::ZERO, Vec3::Y, Vec3::Z], vec![[0, 1, 2]]).unwrap();
    let backface = TriangleMesh::new(vec![Vec3::ZERO, Vec3::Y, Vec3::Z], vec![[0, 2, 1]]).unwrap();
    // Both blockers are much closer than epsilon and the voxel cell size.
    for separation in [1e-9, 0.01, 0.5] {
        let rear = instance(&plate, Vec3::ZERO);
        let front = instance(&plate, Vec3::X * separation);
        let instances = [rear, front];
        let g = surface_geometry(&instances);
        let patches = g
            .exposed_surface_patches(&instances, Vec3::X, 0.1, 100_000)
            .unwrap();
        assert_eq!(patches.len(), 1);
        assert_eq!(patches.as_slice()[0].center.x, separation);
        let instances = [rear, instance(&backface, Vec3::X * separation)];
        assert!(
            g.exposed_surface_patches(&instances, Vec3::X, 0.1, 100_000)
                .unwrap()
                .is_empty()
        );
    }
    let instances = [
        instance(&plate, Vec3::ZERO),
        instance(&plate, Vec3::Y * 2.0),
    ];
    let patches = surface_geometry(&instances)
        .exposed_surface_patches(&instances, Vec3::X, 0.1, 100_000)
        .unwrap();
    assert_eq!(patches.len(), 2);
}

#[test]
fn equal_centroid_and_area_distinct_triangles_are_not_deduplicated() {
    let a = TriangleMesh::new(
        vec![
            Vec3::new(0.0, -1.0, -1.0),
            Vec3::new(0.0, 1.0, -1.0),
            Vec3::new(0.0, 0.0, 2.0),
        ],
        vec![[0, 1, 2]],
    )
    .unwrap();
    let b = TriangleMesh::new(a.vertices().iter().map(|v| -*v).collect(), vec![[0, 1, 2]]).unwrap();
    let instances = [instance(&a, Vec3::ZERO), instance(&b, Vec3::ZERO)];
    let patches = surface_geometry(&instances)
        .exposed_surface_patches(&instances, Vec3::X, 10.0, 100_000)
        .unwrap();
    // Each source has area 3; their intersection has area 2.
    let area: f64 = patches.iter().map(|patch| patch.area).sum();
    assert!((area - 4.0).abs() < 1e-12);
    let first_moment = patches
        .iter()
        .fold(Vec3::ZERO, |sum, patch| sum + patch.center * patch.area);
    assert!(first_moment.length() < 1e-12);
}

#[test]
fn canonical_dedup_handles_vertex_permutations_without_epsilon_aliasing() {
    let a = TriangleMesh::new(
        vec![Vec3::ZERO, Vec3::Y * 0.01, Vec3::Z * 0.01],
        vec![[0, 1, 2]],
    )
    .unwrap();
    let b = TriangleMesh::new(a.vertices().to_vec(), vec![[2, 0, 1]]).unwrap();
    let instances = [
        instance(&a, Vec3::ZERO),
        instance(&b, Vec3::ZERO),
        instance(&a, Vec3::Y * 0.02),
    ];
    let g = surface_geometry(&instances);
    let patches = g
        .exposed_surface_patches(&instances, Vec3::X, 1.0, 100_000)
        .unwrap();
    assert_eq!(patches.len(), 2);
}

#[test]
fn patch_budget_is_checked_before_transform_and_allocation() {
    let mesh = cube();
    let instances = [instance(&mesh, Vec3::ZERO)];
    let g = surface_geometry(&instances);
    let invalid_transform = [instance(&mesh, Vec3::splat(f64::MAX))];
    for sources in [&instances[..], &invalid_transform[..]] {
        assert_eq!(
            PreparedGeometry::surface_triangles(sources, 1),
            Err(crate::Error::from(GeometryError::WorkBudgetExceeded))
        );
        assert_eq!(
            g.exposed_surface_patches(sources, Vec3::X, 0.01, 1),
            Err(crate::Error::from(GeometryError::WorkBudgetExceeded))
        );
    }
    // Exact conservative preflight: 3T² + 8T (T = 12).
    assert_eq!(
        g.exposed_surface_patches(&instances, Vec3::X, 0.01, 527),
        Err(crate::Error::from(GeometryError::WorkBudgetExceeded))
    );
    assert_eq!(
        g.exposed_surface_patches(&instances, Vec3::X, 0.01, 100_000)
            .unwrap()
            .len(),
        2
    );
}

#[test]
fn surface_triangle_budget_includes_instance_visits() {
    let mesh = cube();
    let instances = [instance(&mesh, Vec3::ZERO), instance(&mesh, Vec3::X)];
    let work = 2 + 4 * 2 * 12;
    assert_eq!(
        PreparedGeometry::surface_triangles(&instances, work - 1),
        Err(crate::Error::from(GeometryError::WorkBudgetExceeded))
    );
    assert_eq!(
        PreparedGeometry::surface_triangles(&instances, work)
            .unwrap()
            .len(),
        24
    );
}

#[test]
fn patch_budget_includes_unshadowed_partition_output() -> Result<(), Box<dyn std::error::Error>> {
    let mesh = TriangleMesh::new(vec![Vec3::ZERO, Vec3::Y, Vec3::Z], vec![[0, 1, 2]])?;
    let instances = [instance(&mesh, Vec3::ZERO)];
    let geometry = surface_geometry(&instances);
    assert_eq!(
        geometry.exposed_surface_patches(&instances, Vec3::X, 0.01, 100),
        Err(crate::Error::from(GeometryError::WorkBudgetExceeded))
    );
    assert_eq!(
        geometry
            .exposed_surface_patches(&instances, Vec3::X, 0.01, 1_000)?
            .len(),
        1
    );
    Ok(())
}

#[test]
fn patch_frame_conversion_is_reserved_before_return() {
    use std::cell::Cell;

    const MAX_WORK: usize = 10_000;
    let mesh = TriangleMesh::new(vec![Vec3::ZERO, Vec3::Y, Vec3::Z], vec![[0, 1, 2]]).unwrap();
    let instances = [instance(&mesh, Vec3::ZERO)];
    let geometry = surface_geometry(&instances);
    let calls = Cell::new(0);
    let evaluate = |budget| {
        calls.set(0);
        geometry.surface_patches_with_flow(&instances, budget, |_| {
            calls.set(calls.get() + 1);
            Ok(Vec3::X)
        })
    };
    let (mut low, mut high) = (0, MAX_WORK);
    while high - low > 1 {
        let middle = low + (high - low) / 2;
        if evaluate(middle).is_ok() {
            high = middle;
        } else {
            low = middle;
        }
    }
    assert_eq!(evaluate(high).unwrap().len(), 1);
    let successful_calls = calls.get();
    assert_eq!(
        evaluate(low),
        Err(crate::Error::from(GeometryError::WorkBudgetExceeded))
    );
    assert_eq!(calls.get(), successful_calls);
}

#[test]
fn unshadowed_patch_replacement_has_budget_boundary() {
    let mesh = TriangleMesh::new(vec![Vec3::ZERO, Vec3::Y, Vec3::Z], vec![[0, 1, 2]]).unwrap();
    let instances = [instance(&mesh, Vec3::ZERO)];
    let geometry = surface_geometry(&instances);
    let evaluate = |budget| geometry.surface_patches_with_flow(&instances, budget, |_| Ok(Vec3::X));
    let mut low = 0;
    let mut high = 10_000;
    assert!(evaluate(high).is_ok());
    while high - low > 1 {
        let middle = low + (high - low) / 2;
        if evaluate(middle).is_ok() {
            high = middle
        } else {
            low = middle
        }
    }
    // Includes one unit for replacing the partition and one for frame conversion.
    const REQUIRED_WORK: usize = 117;
    assert_eq!(high, REQUIRED_WORK);
    assert_eq!(
        evaluate(low),
        Err(crate::Error::from(GeometryError::WorkBudgetExceeded))
    );
    assert_eq!(evaluate(high).unwrap().len(), 1);
}

#[test]
fn unrelated_surface_preserves_partially_clipped_patch_area() {
    const MAX_WORK: usize = 100_000;
    const EPSILON: f64 = 0.01;
    const SEPARATION: f64 = 4.0;
    const AREA_TOLERANCE: f64 = 1e-12;
    // Two right triangles of area 1/2 overlap in a similar triangle of leg 3/4.
    const EXPECTED_AREA: f64 = 1.0 - 0.5 * 0.75 * 0.75;
    let plate = TriangleMesh::new(vec![Vec3::ZERO, Vec3::Y, Vec3::Z], vec![[0, 1, 2]]).unwrap();
    let base = [
        instance(&plate, Vec3::ZERO),
        instance(&plate, Vec3::X + Vec3::Y * 0.25),
    ];
    let extended = [base[0], base[1], instance(&plate, Vec3::Y * SEPARATION)];
    let geometry = surface_geometry(&base);
    let total_area = |instances: &[GeometryInstance<'_>]| {
        geometry
            .exposed_surface_patches(instances, Vec3::X, EPSILON, MAX_WORK)
            .unwrap()
            .iter()
            .filter(|patch| patch.center.y < SEPARATION / 2.0)
            .map(|patch| patch.area)
            .sum::<f64>()
    };
    assert!((total_area(&base) - EXPECTED_AREA).abs() < AREA_TOLERANCE);
    assert!((total_area(&extended) - EXPECTED_AREA).abs() < AREA_TOLERANCE);
}

#[test]
fn partially_clipped_patches_respect_exact_budget_boundary() {
    const MAX_WORK: usize = 100_000;
    const EPSILON: f64 = 0.01;
    let plate = TriangleMesh::new(vec![Vec3::ZERO, Vec3::Y, Vec3::Z], vec![[0, 1, 2]]).unwrap();
    let instances = [
        instance(&plate, Vec3::ZERO),
        instance(&plate, Vec3::X + Vec3::Y * 0.25),
    ];
    let geometry = surface_geometry(&instances);
    let evaluate = |budget| geometry.exposed_surface_patches(&instances, Vec3::X, EPSILON, budget);
    let expected = evaluate(MAX_WORK).unwrap();
    assert!(
        expected.len() > 2,
        "fixture must partition the rear triangle"
    );
    let (mut low, mut high) = (0, MAX_WORK);
    while high - low > 1 {
        let middle = low + (high - low) / 2;
        match evaluate(middle) {
            Ok(_) => high = middle,
            Err(error)
                if error.downcast_ref::<GeometryError>()
                    == Some(&GeometryError::WorkBudgetExceeded) =>
            {
                low = middle
            }
            Err(error) => panic!("unexpected error: {error:?}"),
        }
    }
    assert!(matches!(
        evaluate(low),
        Err(error) if error.downcast_ref::<GeometryError>() == Some(&GeometryError::WorkBudgetExceeded)
    ));
    assert_eq!(evaluate(high).unwrap().as_slice(), expected.as_slice());
}

#[test]
fn wide_shadow_projection_obeys_end_to_end_budget() -> Result<(), crate::Error> {
    const EPSILON: f64 = 0.01;
    let x = -2.0_f64.powi(-500);
    let mesh = TriangleMesh::new(
        vec![
            Vec3::ZERO,
            Vec3::Y,
            Vec3::Z,
            Vec3::new(x, 0.25, 0.25),
            Vec3::new(x, 0.50, 0.25),
            Vec3::new(x, 0.25, 0.50),
        ],
        vec![[0, 1, 2], [3, 4, 5]],
    )?;
    let instances = [instance(&mesh, Vec3::ZERO)];
    let geometry = surface_geometry(&instances);
    let evaluate =
        |budget| geometry.exposed_surface_patches(&instances, Vec3::X + Vec3::Y, EPSILON, budget);
    let (mut lower, mut upper) = (0, MAX_SURFACE_WORK);
    let expected = evaluate(upper)?;
    assert_eq!(expected.len(), 1);
    assert_eq!(expected.iter().map(|patch| patch.area).sum::<f64>(), 0.5);
    while upper - lower > 1 {
        let mid = lower + (upper - lower) / 2;
        match evaluate(mid) {
            Ok(patches) => {
                assert_eq!(patches, expected);
                upper = mid;
            }
            Err(error)
                if error.downcast_ref::<GeometryError>()
                    == Some(&GeometryError::WorkBudgetExceeded) =>
            {
                lower = mid;
            }
            Err(error) => return Err(error),
        }
    }
    assert_eq!(evaluate(upper)?, expected);
    assert_eq!(
        evaluate(upper - 1),
        Err(GeometryError::WorkBudgetExceeded.into())
    );
    // Three edge planes each charge a reference sign and three coefficients.
    const EXACT_DETERMINANTS: usize = 12;
    const WORK_PER_DETERMINANT: usize = 1_224;
    const ORDINARY_WORK: usize = 382;
    assert_eq!(
        upper,
        EXACT_DETERMINANTS * WORK_PER_DETERMINANT + ORDINARY_WORK
    );
    Ok(())
}

#[test]
fn multiple_shadow_budget_exhaustion_never_returns_partial_patches() {
    const MAX_WORK: usize = 100_000;
    const EPSILON: f64 = 0.01;
    let plate = TriangleMesh::new(vec![Vec3::ZERO, Vec3::Y, Vec3::Z], vec![[0, 1, 2]]).unwrap();
    let instances = [
        instance(&plate, Vec3::ZERO),
        instance(&plate, Vec3::X + Vec3::Y * 0.25),
        instance(&plate, Vec3::X * 2.0 + Vec3::Z * 0.25),
    ];
    let geometry = surface_geometry(&instances);
    let evaluate = |budget| geometry.exposed_surface_patches(&instances, Vec3::X, EPSILON, budget);
    let expected = evaluate(MAX_WORK).unwrap();
    assert!(expected.len() > instances.len());
    let (mut low, mut high) = (0, MAX_WORK);
    while high - low > 1 {
        let middle = low + (high - low) / 2;
        match evaluate(middle) {
            Ok(result) => {
                assert_eq!(result.as_slice(), expected.as_slice());
                high = middle;
            }
            Err(error)
                if error.downcast_ref::<GeometryError>()
                    == Some(&GeometryError::WorkBudgetExceeded) =>
            {
                low = middle
            }
            Err(error) => panic!("unexpected error: {error:?}"),
        }
    }
    assert!(matches!(
        evaluate(low),
        Err(error) if error.downcast_ref::<GeometryError>() == Some(&GeometryError::WorkBudgetExceeded)
    ));
    assert_eq!(evaluate(high).unwrap().as_slice(), expected.as_slice());
}

#[test]
fn multiple_shadow_clipping_is_independent_of_instance_order() {
    const MAX_WORK: usize = 100_000;
    const EPSILON: f64 = 0.01;
    let plate = TriangleMesh::new(vec![Vec3::ZERO, Vec3::Y, Vec3::Z], vec![[0, 1, 2]]).unwrap();
    let instances = [
        instance(&plate, Vec3::ZERO),
        instance(&plate, Vec3::X + Vec3::Y * 0.25),
        instance(&plate, Vec3::X * 2.0 + Vec3::Z * 0.25),
    ];
    let geometry = surface_geometry(&instances);
    let expected = geometry
        .exposed_surface_patches(&instances, Vec3::X, EPSILON, MAX_WORK)
        .unwrap();
    let reversed = [instances[2], instances[1], instances[0]];
    let result = geometry
        .exposed_surface_patches(&reversed, Vec3::X, EPSILON, MAX_WORK)
        .unwrap();
    assert_eq!(result.as_slice(), expected.as_slice());
}

#[test]
fn translated_rotated_source_instances_preserve_diagonal_visibility() {
    let mesh = cube();
    let base = [instance(&mesh, Vec3::ZERO)];
    let transform = Transform::new(
        Vec3::new(3.0, -4.0, 7.0),
        Quat::from_axis_angle(Vec3::Z, 0.7).unwrap(),
    )
    .unwrap();
    let moved = [GeometryInstance {
        local_transform: transform,
        ..base[0]
    }];
    let original = surface_geometry(&base)
        .exposed_surface_patches(&base, Vec3::X + Vec3::Y, 0.01, 100_000)
        .unwrap();
    let transformed = surface_geometry(&moved)
        .exposed_surface_patches(
            &moved,
            transform.transform_vector(Vec3::X + Vec3::Y),
            0.01,
            100_000,
        )
        .unwrap();
    assert_eq!(original.len(), 4);
    assert_eq!(transformed.len(), 4);
    for patch in original.iter() {
        assert!(transformed.iter().any(|p| {
            (p.center - transform.transform_point(patch.center)).length() < 1e-12
                && (p.normal - transform.transform_vector(patch.normal)).length() < 1e-12
        }));
    }
}

#[test]
fn diagonal_cube_pressure_is_covariant_over_many_instance_rotations()
-> Result<(), Box<dyn std::error::Error>> {
    use crate::aerodynamics::{
        AtmosphericState,
        pressure::{PressureFlow, evaluate_prepared_mesh},
    };
    let mesh = cube();
    for step in 1..=200 {
        let angle = step as f64 * 0.013;
        let rotation = kapitsa_math::Quat::from_axis_angle(Vec3::new(1.0, 2.0, 3.0), angle)
            .ok_or("rotation")?;
        let transform = Transform::new(Vec3::new(2.3, -4.1, 0.7), rotation)?;
        let instances = [GeometryInstance {
            mesh: &mesh,
            local_transform: transform,
            owner: GeometryOwner::Body,
        }];
        let geometry = PreparedGeometry::prepare(&instances, config(0.25))?;
        let direction = transform.transform_vector(Vec3::ONE);
        let patches = geometry.exposed_surface_patches(&instances, direction, 0.01, 100_000)?;
        let wrench = evaluate_prepared_mesh(
            &patches,
            PressureFlow {
                atmosphere: AtmosphericState::new(1.0, Vec3::ZERO)?,
                sound_speed: 100.0,
                transform: Transform::identity(),
                center_of_mass: transform.transform_point(Vec3::splat(0.5)),
                linear_velocity: direction.try_normalize().ok_or("direction")? * 1000.0,
                angular_velocity: Vec3::ZERO,
            },
        )?;
        let force = -direction * (1_000_000.0 / 3.0);
        assert!(
            (wrench.force - force).length() < 1e-7,
            "angle={angle}, force={:?}, expected={force:?}, patches={}",
            wrench.force,
            patches.len()
        );
        assert!(
            wrench.moment.length() < 1e-7,
            "angle={angle}, moment={:?}",
            wrench.moment
        );
    }
    Ok(())
}

#[test]
fn rotated_coplanar_retriangulation_preserves_union_pressure()
-> Result<(), Box<dyn std::error::Error>> {
    use crate::aerodynamics::{
        AtmosphericState,
        pressure::{PressureFlow, evaluate_prepared_mesh},
    };
    let vertices = vec![Vec3::ZERO, Vec3::Y, Vec3::Z, Vec3::Y + Vec3::Z];
    let first = TriangleMesh::new(vertices.clone(), vec![[0, 1, 2], [1, 3, 2]])?;
    let second = TriangleMesh::new(vertices, vec![[0, 1, 3], [0, 3, 2]])?;
    for baked in [false, true] {
        for step in 1..=200 {
            let angle = step as f64 * 0.013;
            let rotation = kapitsa_math::Quat::from_axis_angle(Vec3::new(1.0, 2.0, 3.0), angle)
                .ok_or("rotation")?;
            let transform = Transform::new(Vec3::new(2.3, -4.1, 0.7), rotation)?;
            let baked_first = TriangleMesh::new(
                first
                    .vertices()
                    .iter()
                    .map(|point| transform.transform_point(*point))
                    .collect(),
                first.triangles().to_vec(),
            )?;
            let baked_second = TriangleMesh::new(
                second
                    .vertices()
                    .iter()
                    .map(|point| transform.transform_point(*point))
                    .collect(),
                second.triangles().to_vec(),
            )?;
            let placement = if baked {
                Transform::identity()
            } else {
                transform
            };
            let instances = [
                GeometryInstance {
                    mesh: if baked { &baked_first } else { &first },
                    local_transform: placement,
                    owner: GeometryOwner::Body,
                },
                GeometryInstance {
                    mesh: if baked { &baked_second } else { &second },
                    local_transform: placement,
                    owner: GeometryOwner::Body,
                },
            ];
            let mut settings = config(0.25);
            settings.open_mesh_policy = OpenMeshPolicy::SurfaceOnly;
            let geometry = PreparedGeometry::prepare(&instances, settings)?;
            let direction = transform.transform_vector(Vec3::X);
            let patches = geometry.exposed_surface_patches(&instances, direction, 0.01, 100_000)?;
            let wrench = evaluate_prepared_mesh(
                &patches,
                PressureFlow {
                    atmosphere: AtmosphericState::new(1.0, Vec3::ZERO)?,
                    sound_speed: 100.0,
                    transform: Transform::identity(),
                    center_of_mass: transform.transform_point((Vec3::Y + Vec3::Z) * 0.5),
                    linear_velocity: direction * 1000.0,
                    angular_velocity: Vec3::ZERO,
                },
            )?;
            assert!(
                (wrench.force + direction * 1_000_000.0).length() < 1e-7,
                "baked={baked}, angle={angle}, force={:?}, patches={}",
                wrench.force,
                patches.len()
            );
            assert!(
                wrench.moment.length() < 1e-7,
                "angle={angle}, moment={:?}",
                wrench.moment
            );
        }
    }
    Ok(())
}

#[test]
fn voxel_sections_configure_wave_drag_with_checked_axis() -> Result<(), Box<dyn std::error::Error>>
{
    use crate::aerodynamics::{
        AtmosphericState,
        hull::{HullError, HullSection},
        pressure::PressureFlow,
        wave,
    };
    let mesh = cube();
    let geometry = PreparedGeometry::prepare(&[instance(&mesh, Vec3::ZERO)], config(0.125))?;
    let section = HullSection::new(
        Transform::identity(),
        Vec3::X,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        None,
    )?;
    assert!(matches!(
        section.clone().with_voxel_wave_drag(&geometry, 1, 1),
        Err(error) if error.downcast_ref::<HullError>() == Some(&HullError::InvalidParameters)
    ));
    assert!(matches!(
        section.clone().with_voxel_wave_drag(&geometry, 3, 1),
        Err(error) if error.downcast_ref::<HullError>() == Some(&HullError::InvalidParameters)
    ));
    let configured = section.with_voxel_wave_drag(&geometry, 0, 1)?;
    let slopes = wave::area_slopes_from_sections(&geometry.sections()[0], geometry.cell_size(), 1)?;
    let expected = wave::wave_drag(&slopes, 1.0, 680.0, 340.0)?;
    let result = configured.evaluate(PressureFlow {
        atmosphere: AtmosphericState::new(1.0, Vec3::ZERO)?,
        sound_speed: 340.0,
        transform: Transform::identity(),
        center_of_mass: Vec3::ZERO,
        linear_velocity: Vec3::X * 680.0,
        angular_velocity: Vec3::ZERO,
    })?;
    assert!(expected > 0.0);
    assert!((result.force.x + expected).abs() < expected * 1e-12);
    Ok(())
}

#[test]
fn voxel_wave_drag_refines_on_slender_sears_haack_mesh() -> Result<(), Box<dyn std::error::Error>> {
    use crate::aerodynamics::wave::{area_slopes_from_sections, wave_drag};
    use std::f64::consts::PI;
    let axial = 48_usize;
    let radial = 32_usize;
    let mut vertices = vec![Vec3::ZERO];
    for i in 1..axial {
        let x = i as f64 / axial as f64;
        let radius = 0.1 * (4.0 * x * (1.0 - x)).powf(0.75);
        for j in 0..radial {
            let angle = 2.0 * PI * j as f64 / radial as f64;
            vertices.push(Vec3::new(x, radius * angle.cos(), radius * angle.sin()));
        }
    }
    let tail = vertices.len();
    vertices.push(Vec3::X);
    let mut triangles = Vec::new();
    for j in 0..radial {
        let next = (j + 1) % radial;
        triangles.push([0, 1 + next, 1 + j]);
        for i in 0..axial - 2 {
            let a = 1 + i * radial + j;
            let b = 1 + i * radial + next;
            let c = a + radial;
            let d = b + radial;
            triangles.push([a, b, c]);
            triangles.push([b, d, c]);
        }
        let a = 1 + (axial - 2) * radial + j;
        let b = 1 + (axial - 2) * radial + next;
        triangles.push([a, b, tail]);
    }
    let mesh = TriangleMesh::new(vertices, triangles)?;
    let volume = PI * 0.01 * 8.0 * 3.0 * PI / 128.0;
    let theoretical = 256.0 * volume * volume / PI;
    let mut measured = Vec::new();
    for (h, radius) in [(0.05, 1), (0.025, 2), (0.0125, 4), (0.00625, 8)] {
        let mut settings = config(h);
        settings.max_work = 100_000_000;
        settings.max_cells = 1_000_000;
        let geometry = PreparedGeometry::prepare(&[instance(&mesh, Vec3::ZERO)], settings)?;
        let slopes = area_slopes_from_sections(&geometry.sections()[0], h, radius)?;
        measured.push(wave_drag(&slopes, 1.0, 2.0, 1.0)?);
    }
    let errors: Vec<_> = measured
        .iter()
        .map(|value| (value - theoretical).abs())
        .collect();
    assert!(
        errors[3] < errors[2]
            && errors[2] < errors[1]
            && errors[1] < errors[0]
            && errors[2] < theoretical * 0.2
            && errors[3] < theoretical * 0.02,
        "expected={theoretical}, measured={measured:?}"
    );
    Ok(())
}

#[test]
fn preflight_counts_triangle_bounds_and_enforces_budget() -> Result<(), Box<dyn std::error::Error>>
{
    let mesh = cube();
    let instances = [instance(&mesh, Vec3::ZERO)];
    let mut settings = config(0.05);
    settings.max_work = 200_000;
    let bounded = PreparedGeometry::prepare(&instances, settings)?;
    settings.max_work = 10_000_000;
    let reference = PreparedGeometry::prepare(&instances, settings)?;
    assert_eq!(bounded, reference);
    settings.max_work = 100_000;
    assert!(matches!(
        PreparedGeometry::prepare(&instances, settings),
        Err(error) if error.downcast_ref::<GeometryError>() == Some(&GeometryError::WorkBudgetExceeded)
    ));
    Ok(())
}

#[test]
fn occupancy_query_validates_points_and_grid_boundaries() -> Result<(), Box<dyn std::error::Error>>
{
    let mesh = cube();
    let geometry = PreparedGeometry::prepare(&[instance(&mesh, Vec3::ZERO)], config(0.125))?;
    assert!(geometry.occupied_at(Vec3::splat(0.5))?);
    assert!(!geometry.occupied_at(Vec3::splat(10.0))?);
    assert_eq!(
        geometry.occupied_at(Vec3::new(f64::NAN, 0.0, 0.0)),
        Err(crate::Error::from(GeometryError::InvalidGeometry))
    );
    let upper = geometry.origin()
        + Vec3::new(
            geometry.dimensions()[0] as f64 * geometry.cell_size(),
            geometry.dimensions()[1] as f64 * geometry.cell_size(),
            geometry.dimensions()[2] as f64 * geometry.cell_size(),
        );
    assert!(!geometry.occupied_at(upper)?);
    Ok(())
}

#[test]
fn wing_volume_visibility_resolves_full_partial_and_clear_footprints()
-> Result<(), Box<dyn std::error::Error>> {
    use crate::aerodynamics::wing::{FiniteWing, WingError, WingSurface};
    let mesh = cube();
    let geometry = PreparedGeometry::prepare(&[instance(&mesh, Vec3::ZERO)], config(0.125))?;
    let model = FiniteWing::new(4.0, 0.8, 0.02, 0.0, 0.2)?;
    let inside = WingSurface::new(model, Transform::identity(), 0.0625)?
        .with_rectangular_footprint(Vec3::new(0.5, 0.5, 0.5), 0.25, 0.25)?;
    let visibility = inside
        .volume_visibility(&geometry, Transform::identity(), 1_000_000)?
        .ok_or("visibility")?;
    assert_eq!(visibility.fraction, 0.0);
    let partial = WingSurface::new(model, Transform::identity(), 4.0)?.with_rectangular_footprint(
        Vec3::new(0.0, 0.0, 0.5),
        2.0,
        2.0,
    )?;
    let visibility = partial
        .volume_visibility(&geometry, Transform::identity(), 1_000_000)?
        .ok_or("visibility")?;
    assert!(visibility.fraction > 0.5 && visibility.fraction < 0.9);
    assert!(visibility.offset.x < 0.0 && visibility.offset.y < 0.0);
    let rotation = Quat::from_axis_angle(Vec3::Z, std::f64::consts::FRAC_PI_2).ok_or("rotation")?;
    let part_transform = Transform::new(Vec3::ZERO, rotation)?;
    let rotated = partial
        .volume_visibility(&geometry, part_transform, 1_000_000)?
        .ok_or("visibility")?;
    assert_eq!(rotated.fraction, visibility.fraction);
    assert!((part_transform.transform_vector(rotated.offset) - visibility.offset).length() < 1e-12);
    let flow = crate::aerodynamics::pressure::PressureFlow {
        atmosphere: crate::aerodynamics::AtmosphericState::new(1.2, Vec3::ZERO)?,
        sound_speed: 340.0,
        transform: Transform::identity(),
        center_of_mass: Vec3::ZERO,
        linear_velocity: Vec3::new(50.0, 0.0, -5.0),
        angular_velocity: Vec3::ZERO,
    };
    let full = partial.evaluate(flow)?;
    let exposed = partial.evaluate_with_visibility(flow, Some(visibility))?;
    assert!((exposed.force - full.force * visibility.fraction).length() < 1e-10);
    assert!((exposed.moment - visibility.offset.cross(exposed.force)).length() < 1e-10);
    assert!(matches!(
        partial.volume_visibility(&geometry, Transform::identity(), 1),
        Err(error) if error.downcast_ref::<WingError>() == Some(&WingError::ShieldingBudgetExceeded)
    ));
    let outside = WingSurface::new(model, Transform::identity(), 0.0625)?
        .with_rectangular_footprint(Vec3::new(3.0, 3.0, 0.5), 0.25, 0.25)?;
    let visibility = outside
        .volume_visibility(&geometry, Transform::identity(), 1_000_000)?
        .ok_or("visibility")?;
    assert_eq!(visibility.fraction, 1.0);
    assert_eq!(visibility.offset, Vec3::ZERO);
    Ok(())
}

#[test]
fn voxel_slender_sections_apply_and_reject_rotation_in_world_steps()
-> Result<(), Box<dyn std::error::Error>> {
    use crate::{
        aerodynamics::{
            Aerodynamics, AerodynamicsTask, Atmosphere, DensityProfile, SphericalAtmosphere,
            hull::HullError, slender::hull_sections_from_voxels,
        },
        execution::SequentialExecutor,
        mass::MassProperties,
        rigid_body::RigidBody,
        world::PhysicsWorld,
    };
    use kapitsa_math::Mat3;
    let mesh = cube();
    let geometry = PreparedGeometry::prepare(&[instance(&mesh, Vec3::ZERO)], config(0.125))?;
    let sections = hull_sections_from_voxels(&geometry, 0, Vec3::new(0.0, 0.5, 0.5), 1, 0.2)?;
    assert!(!sections.is_empty());
    let mut expected = None;
    for clustered in [false, true] {
        for sharded in [false, true] {
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(Atmosphere::Sphere(
                SphericalAtmosphere::builder()
                    .reference_radius(1_000_000.0)
                    .reference_density(1.0)
                    .height(1_000_000.0)
                    .profile(DensityProfile::Exponential {
                        scale_height: 1_000_000.0,
                    })
                    .build()?,
            ));
            let mass =
                MassProperties::new(100.0, Vec3::new(0.5, 0.5, 0.5), Mat3::IDENTITY * 100.0)?;
            let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
            body.set_linear_velocity(Vec3::new(10.0, 0.0, -1.0))?;
            body.set_aerodynamics(Aerodynamics::Hull(sections.clone()));
            let id = world.add_body(body)?;
            if clustered {
                world.enable_cluster(id)?;
            }
            AerodynamicsTask.execute(&world)?;
            if sharded {
                world.step_with_shards(
                    1e-4,
                    &mut [Vec::new(), Vec::new()],
                    &SequentialExecutor,
                    1,
                )?;
            } else {
                world.step(1e-4)?;
            }
            let motion = world.body(id).ok_or("body")?.motion();
            assert!(motion.state().angular_velocity().length() > 0.0);
            if let Some(reference) = expected {
                assert_eq!(motion, reference);
            } else {
                expected = Some(motion);
            }
            let failed = if sharded {
                world.step_with_shards(1e-4, &mut [Vec::new()], &SequentialExecutor, 1)
            } else {
                world.step(1e-4)
            };
            assert!(matches!(
                aero_hull(&failed),
                Some(HullError::UnsupportedSlenderRotation)
            ));
            assert_eq!(world.body(id).ok_or("body")?.motion(), motion);
        }
    }
    Ok(())
}

#[test]
fn quasi_steady_voxel_sections_advance_rotating_body() -> Result<(), Box<dyn std::error::Error>> {
    use crate::{
        aerodynamics::{
            Aerodynamics, Atmosphere, DensityProfile, SphericalAtmosphere,
            slender::hull_sections_from_voxels_quasi_steady,
        },
        execution::SequentialExecutor,
        mass::MassProperties,
        rigid_body::RigidBody,
        world::PhysicsWorld,
    };
    use kapitsa_math::Mat3;
    let mesh = cube();
    let geometry = PreparedGeometry::prepare(&[instance(&mesh, Vec3::ZERO)], config(0.125))?;
    let sections =
        hull_sections_from_voxels_quasi_steady(&geometry, 0, Vec3::new(0.0, 0.5, 0.5), 1, 0.2)?;
    let mut expected = None;
    for clustered in [false, true] {
        for sharded in [false, true] {
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(Atmosphere::Sphere(
                SphericalAtmosphere::builder()
                    .reference_radius(1_000_000.0)
                    .reference_density(1.0)
                    .height(1_000_000.0)
                    .profile(DensityProfile::Exponential {
                        scale_height: 1_000_000.0,
                    })
                    .build()?,
            ));
            let mass =
                MassProperties::new(100.0, Vec3::new(0.5, 0.5, 0.5), Mat3::IDENTITY * 100.0)?;
            let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
            body.set_linear_velocity(Vec3::X * 10.0)?;
            body.set_angular_velocity(Vec3::Y * 0.01)?;
            body.set_aerodynamics(Aerodynamics::Hull(sections.clone()));
            let id = world.add_body(body)?;
            if clustered {
                world.enable_cluster(id)?;
            }
            for _ in 0..3 {
                if sharded {
                    world.step_with_shards(
                        1e-4,
                        &mut [Vec::new(), Vec::new()],
                        &SequentialExecutor,
                        1,
                    )?;
                } else {
                    world.step(1e-4)?;
                }
            }
            let motion = world.body(id).ok_or("body")?.motion();
            if let Some(reference) = expected {
                assert_eq!(motion, reference);
            } else {
                expected = Some(motion);
            }
        }
    }
    Ok(())
}
