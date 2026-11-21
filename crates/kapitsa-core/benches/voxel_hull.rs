//! Run with `cargo bench -p kapitsa-core --bench voxel_hull`.
use std::{
    alloc::{GlobalAlloc, Layout, System},
    cell::Cell,
    error::Error,
    hint::black_box,
    sync::Arc,
    time::Instant,
};

use kapitsa_core::{
    aerodynamics::{
        AtmosphericState,
        colliders::{ColliderTessellation, collider_mesh},
        geometry::{GeometryConfig, GeometryError, GeometryInstance, GeometryOwner, OwnerSlot},
        pressure::{MeshPressureModel, OwnerPressureWorkspace, PressureFlow},
        shape::{CookedShape, CookedSource, assembly_pressure_config},
    },
    collider::Collider,
    shape::{Cuboid, Shape},
    transform::Transform,
};
use kapitsa_math::Vec3;

#[derive(Clone, Copy, Default)]
struct Heap {
    requests: usize,
    live: isize,
    peak: isize,
}

thread_local! {
    static HEAP: Cell<Option<Heap>> = const { Cell::new(None) };
}

fn account(requests: usize, delta: isize) {
    HEAP.with(|cell| {
        if let Some(mut heap) = cell.get() {
            heap.requests += requests;
            heap.live += delta;
            heap.peak = heap.peak.max(heap.live);
            cell.set(Some(heap));
        }
    });
}

struct TrackingAllocator;

// SAFETY: All pointers and layouts are forwarded unchanged to System. The
// thread-local counter uses only stack/static storage and never allocates.
unsafe impl GlobalAlloc for TrackingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let ptr = unsafe { System.alloc(layout) };
        if !ptr.is_null() {
            account(1, layout.size() as isize);
        }
        ptr
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) };
        account(0, -(layout.size() as isize));
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        let ptr = unsafe { System.realloc(ptr, layout, size) };
        if !ptr.is_null() {
            account(1, size as isize - layout.size() as isize);
        }
        ptr
    }
}

#[global_allocator]
static ALLOCATOR: TrackingAllocator = TrackingAllocator;

fn measure<T>(label: &str, batches: usize, iterations: usize, mut run: impl FnMut() -> T) -> T {
    drop(black_box(run())); // warm up before sampling
    let mut times = Vec::with_capacity(batches);
    for _ in 0..batches {
        let start = Instant::now();
        for _ in 0..iterations {
            drop(black_box(run()));
        }
        times.push(start.elapsed().as_secs_f64() * 1e6 / iterations as f64);
    }
    times.sort_by(f64::total_cmp);
    // Measure heap separately so timings do not include counter updates.
    HEAP.set(Some(Heap::default()));
    let result = black_box(run());
    let heap = HEAP.replace(None).unwrap_or_default();
    println!(
        "{label}: median={:.3} us/op, requests={}, peak={} B, live at return={} B",
        times[batches / 2],
        heap.requests,
        heap.peak,
        heap.live
    );
    result
}

fn main() -> Result<(), Box<dyn Error>> {
    const BATCHES: usize = 7;
    let sources = [1.0, 0.25]
        .into_iter()
        .enumerate()
        .map(|(i, edge)| -> Result<_, kapitsa_core::Error> {
            let cuboid = Cuboid::new(edge, edge, edge)?;
            let collider = Collider::new(Shape::Cuboid(cuboid), Transform::identity());
            Ok(CookedSource {
                mesh: Arc::new(collider_mesh([&collider], ColliderTessellation::default())?),
                local_transform: Transform::identity(),
                slot: OwnerSlot::new(i as u32),
            })
        })
        .collect::<Result<Vec<_>, _>>()?;
    let config = assembly_pressure_config();
    let flow = PressureFlow {
        atmosphere: AtmosphericState::new(1.0, Vec3::ZERO)?,
        sound_speed: 340.0,
        transform: Transform::identity(),
        center_of_mass: Vec3::ZERO,
        linear_velocity: Vec3::X * (4.63 * 340.0),
        angular_velocity: Vec3::ZERO,
    };
    for (name, cell_size, cook_iterations) in [
        ("representative", 0.0625, 20),
        ("near cell ceiling", 1.0 / 98.5, 1),
    ] {
        println!("{name}: h={cell_size} m");
        let config = GeometryConfig {
            cell_size,
            ..config
        };
        let cook = |config| CookedShape::cook(black_box(&sources), config, false);
        let shape = measure("cook", BATCHES, cook_iterations, || cook(config))?;
        let geometry = shape.geometry();
        let cells = geometry.occupancy().len();
        let instances: Vec<_> = shape
            .sources()
            .iter()
            .map(|source| GeometryInstance {
                mesh: &source.mesh,
                local_transform: source.local_transform,
                owner: GeometryOwner::Slot(source.slot),
            })
            .collect();
        let owners: Vec<_> = instances.iter().map(|source| source.owner).collect();
        let mut workspace = measure("pressure workspace setup", BATCHES, 500, || {
            OwnerPressureWorkspace::new(black_box(&owners))
        })
        .ok_or("ordered owners")?;
        workspace.set_pressure_model(MeshPressureModel::ModifiedNewtonianTriangleQuadrature);
        let exposure =
            |budget| geometry.exposed_surface_patches(black_box(&instances), Vec3::X, 0.01, budget);
        let patches = exposure(config.max_patch_work)?;
        let pressure_only = measure("pressure on frozen patches", BATCHES, 500, || {
            workspace.prepare(black_box(&patches), black_box(flow))?;
            workspace.staged_total()
        })?;
        let mut evaluate = |budget| -> Result<_, kapitsa_core::Error> {
            let patches = exposure(budget)?;
            workspace.prepare(&patches, black_box(flow))?;
            workspace.staged_total()
        };
        let total = measure("exposure + pressure", BATCHES, 500, || {
            evaluate(config.max_patch_work)
        })?
        .ok_or("missing staged load")?;
        assert!(total.force.x < 0.0);
        assert_eq!(pressure_only, Some(total));
        let triangles: usize = instances
            .iter()
            .map(|source| source.mesh.triangles().len())
            .sum();
        println!(
            "{} cells {:?}, {triangles} source triangles, {} exposed patches",
            cells,
            geometry.dimensions(),
            patches.len()
        );

        if name == "near cell ceiling" {
            let rejected = measure("cell budget rejection", BATCHES, 500, || {
                cook(GeometryConfig {
                    max_cells: cells - 1,
                    ..config
                })
            });
            assert!(
                matches!(rejected, Err(ref e) if e.downcast_ref::<GeometryError>() == Some(&GeometryError::CellBudgetExceeded))
            );

            // Probe this deterministic fixture's exact work threshold without
            // exposing a benchmark-only counter through the public API.
            let (mut low, mut high) = (0, config.max_patch_work);
            while high - low > 1 {
                let mid = low + (high - low) / 2;
                match exposure(mid) {
                    Ok(_) => high = mid,
                    Err(e)
                        if e.downcast_ref::<GeometryError>()
                            == Some(&GeometryError::WorkBudgetExceeded) =>
                    {
                        low = mid
                    }
                    Err(e) => return Err(e.into()),
                }
            }
            println!("first passing patch work limit={high}");
            let boundary = measure("exposure + pressure at work limit", BATCHES, 500, || {
                evaluate(high)
            })?;
            assert_eq!(boundary, Some(total));
            let rejected = measure("patch work budget rejection", BATCHES, 500, || {
                evaluate(low)
            });
            assert!(
                matches!(rejected, Err(ref e) if e.downcast_ref::<GeometryError>() == Some(&GeometryError::WorkBudgetExceeded))
            );
        }
    }
    Ok(())
}
