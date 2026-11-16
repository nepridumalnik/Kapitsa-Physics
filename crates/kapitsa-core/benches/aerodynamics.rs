//! Deterministic scalar kernels; run with `cargo bench -p kapitsa-core --bench aerodynamics`.
use std::{hint::black_box, time::Instant};

use kapitsa_core::{
    aerodynamics::{
        AtmosphericState,
        cache::{GeometryCacheKey, PreparedGeometryCache},
        geometry::{
            GeometryConfig, GeometryInstance, GeometryOwner, OpenMeshPolicy, PreparedGeometry,
        },
        pressure::{PressureFlow, PressureWrench, evaluate_prepared_mesh},
        wing::{FiniteWing, WingSurface},
    },
    rigid_body::RigidBody,
    shape::TriangleMesh,
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Quat, Vec3};

const SAMPLES: usize = 7;

// Each tile has a duplicated rear square and a half-overlapping upstream square.
// All normals are +X. Visibility must remove duplicates and half of the rear
// square, retaining area 1.5 (rather than the unfiltered area 3) per tile.
fn plates(tiles: usize) -> TriangleMesh {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for tile in 0..tiles {
        for (x, shift) in [(0.0, 0.0), (0.0, 0.0), (1.0, 0.5)] {
            let y = tile as f64 * 3.0 + shift;
            let base = vertices.len();
            vertices.extend([
                Vec3::new(x, y, 0.0),
                Vec3::new(x, y + 1.0, 0.0),
                Vec3::new(x, y + 1.0, 1.0),
                Vec3::new(x, y, 1.0),
            ]);
            triangles.extend([[base, base + 1, base + 2], [base, base + 2, base + 3]]);
        }
    }
    TriangleMesh::new(vertices, triangles).unwrap()
}

fn measure(label: &str, iterations: usize, mut operation: impl FnMut()) {
    // Untimed warmup, then the same deterministic workload in every sample.
    for _ in 0..3 {
        operation();
    }
    let mut samples = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        let start = Instant::now();
        for _ in 0..iterations {
            operation();
        }
        samples.push(start.elapsed().as_secs_f64() * 1e9 / iterations as f64);
    }
    samples.sort_by(f64::total_cmp);
    println!(
        "{label:28} iterations={iterations:6} median={:12.2} ns/op range={:.2}..{:.2}",
        samples[SAMPLES / 2],
        samples[0],
        samples[SAMPLES - 1]
    );
}

fn wing_load(wings: &[WingSurface], flow: PressureFlow) -> PressureWrench {
    let mut total = PressureWrench {
        force: Vec3::ZERO,
        moment: Vec3::ZERO,
    };
    for wing in wings {
        let load = wing.evaluate(flow).unwrap();
        total.force += load.force;
        total.moment += load.moment;
    }
    total
}

fn main() {
    println!(
        "kapitsa-core scalar aerodynamic baseline; samples={SAMPLES}; median and min/max; 3 warmup operations"
    );
    let mut world = PhysicsWorld::new();
    let body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    for (label, tiles, preparation_iterations) in [("small", 1, 100), ("moderate", 16, 10)] {
        let mesh = plates(tiles);
        let instances = [GeometryInstance {
            mesh: &mesh,
            local_transform: Transform::identity(),
            owner: GeometryOwner::Body,
        }];
        let config = GeometryConfig {
            cell_size: 0.25,
            max_cells: 100_000,
            max_work: 100_000_000,
            max_patch_work: 100_000_000,
            open_mesh_policy: OpenMeshPolicy::SurfaceOnly,
            weld_tolerance: None,
        };
        // All correctness checks are outside timed loops. No timing thresholds.
        let prepared = PreparedGeometry::prepare(&instances, config).unwrap();
        assert_eq!(
            prepared,
            PreparedGeometry::prepare(&instances, config).unwrap()
        );
        // Two separated X planes, each touching a 5x5 cell footprint;
        // the coincident rear duplicate must not add occupied volume.
        assert_eq!(
            prepared.occupancy().iter().filter(|p| p.is_some()).count(),
            50 * tiles
        );
        assert_eq!(prepared.volume(), 50.0 * tiles as f64 * 0.25_f64.powi(3));
        let patches = prepared
            .exposed_surface_patches(&instances, Vec3::X, 0.01, config.max_patch_work)
            .unwrap();
        assert_eq!(
            patches,
            prepared
                .exposed_surface_patches(&instances, Vec3::X, 0.01, config.max_patch_work)
                .unwrap()
        );
        let area: f64 = patches.iter().map(|p| p.area).sum();
        assert!((area - 1.5 * tiles as f64).abs() < 1e-10);
        assert!(patches.iter().all(|p| p.normal == Vec3::X));
        let flow = PressureFlow {
            atmosphere: AtmosphericState::new(1.0, Vec3::ZERO).unwrap(),
            sound_speed: 340.0,
            transform: Transform::identity(),
            center_of_mass: Vec3::ZERO,
            linear_velocity: Vec3::X * 2000.0,
            angular_velocity: Vec3::X * 0.1,
        };
        // Rotation about X changes local transverse flow, but Newtonian normal
        // pressure on these X-normal plates remains rho * vx^2 per unit area.
        let pressure = evaluate_prepared_mesh(&patches, flow).unwrap();
        assert!((pressure.force.x + 4_000_000.0 * area).abs() < 1e-6);
        assert_eq!(pressure.force.y, 0.0);
        assert_eq!(pressure.force.z, 0.0);
        let expected_moment = patches.iter().fold(Vec3::ZERO, |sum, p| {
            sum + p.center.cross(-Vec3::X * (4_000_000.0 * p.area))
        });
        assert!((pressure.moment - expected_moment).length() < 1e-5);
        let mut key = GeometryCacheKey {
            body,
            geometry_version: 1,
            config_version: 1,
        };
        let mut cache = PreparedGeometryCache::default();
        assert_eq!(
            cache.prepare_instances(key, &instances, config).unwrap(),
            &prepared
        );
        let address = cache.get().unwrap() as *const _;
        assert_eq!(
            cache.prepare_instances(key, &instances, config).unwrap() as *const _,
            address
        );
        key.geometry_version += 1;
        cache.request(key, config);
        assert!(cache.get().is_none());
        assert_eq!(
            cache.prepare_instances(key, &instances, config).unwrap(),
            &prepared
        );

        let model = FiniteWing::new(8.0, 0.8, 0.02, 0.0, 0.2).unwrap();
        let wings: Vec<_> = (0..tiles * 2)
            .map(|i| {
                let y = (i as f64 - (tiles * 2 - 1) as f64 / 2.0) * 0.5;
                WingSurface::new(
                    model,
                    Transform::new(Vec3::Y * y, Quat::IDENTITY).unwrap(),
                    1.0,
                )
                .unwrap()
            })
            .collect();
        let wing_flow = PressureFlow {
            linear_velocity: Vec3::new(60.0, 0.0, -3.0),
            angular_velocity: Vec3::Y * 0.1,
            ..flow
        };
        let wing_result = wing_load(&wings, wing_flow);
        let one = model
            .force(wing_flow.linear_velocity, 1.0, 340.0, 1.0)
            .unwrap();
        assert!((wing_result.force - one * wings.len() as f64).length() < 1e-8);
        assert!(wing_result.force.x < 0.0 && wing_result.force.z > 0.0);
        assert!(wing_result.moment.length() < 1e-8);
        println!(
            "\n{label}: tiles={tiles} triangles={} grid={:?} cells={} occupied={} patches={} exposed_area={area} wings={}",
            mesh.triangles().len(),
            prepared.dimensions(),
            prepared.occupancy().len(),
            prepared.occupancy().iter().filter(|p| p.is_some()).count(),
            patches.len(),
            wings.len()
        );
        measure("voxel preparation", preparation_iterations, || {
            black_box(PreparedGeometry::prepare(black_box(&instances), black_box(config)).unwrap());
        });
        measure("partial visibility", preparation_iterations, || {
            black_box(
                prepared
                    .exposed_surface_patches(
                        black_box(&instances),
                        black_box(Vec3::X),
                        black_box(0.01),
                        black_box(config.max_patch_work),
                    )
                    .unwrap(),
            );
        });
        measure("cache hit", 100_000, || {
            black_box(
                cache
                    .prepare_instances(black_box(key), black_box(&instances), black_box(config))
                    .unwrap(),
            );
        });
        measure("cache miss + preparation", preparation_iterations, || {
            key.geometry_version += 1;
            black_box(
                cache
                    .prepare_instances(black_box(key), black_box(&instances), black_box(config))
                    .unwrap(),
            );
        });
        measure("prepared pressure", 10_000, || {
            black_box(evaluate_prepared_mesh(black_box(&patches), black_box(flow)).unwrap());
        });
        measure("finite-wing surfaces", 10_000, || {
            black_box(wing_load(black_box(&wings), black_box(wing_flow)));
        });
    }
}
