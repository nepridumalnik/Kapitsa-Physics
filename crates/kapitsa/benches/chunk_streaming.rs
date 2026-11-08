#[path = "chunk_streaming/support.rs"]
mod support;

use kapitsa::execution::{RayonPool, StepExecutor};
use std::{fmt::Write, process::Command};
use support::{BenchResult, Config};

fn main() -> BenchResult<()> {
    if std::env::args().any(|arg| arg == "--help") {
        println!(
            "See docs/chunk-streaming-benchmark.md. Options: --prepare-workers 1,2,4,8 --physics-workers 1,2,4,8 --active-chunks 4,64,1024,16384 --meshes-per-chunk 1 --grid 8 --seconds 60 --dt 0.02 --repeats 3 --scenarios overflight,shuttle,boundary,landing,ccd,bulk --mode throughput,realtime --cache cold,warm --batch-sizes 1,4,16,64,256,1024 --flight-batch 16 --in-flight 64 --bulk-iterations 20 --output target/report.md"
        );
        return Ok(());
    }
    let config = Config::parse(std::env::args().skip(1))?;
    let mut report = String::new();
    writeln!(
        report,
        "# Kapitsa chunk streaming benchmark\n\nOS: {}; arch: {}; logical CPUs: {}; CPU: {}.\n\n```text\n{}\ncommit: {}\nconfig: {config:#?}\n```",
        std::env::consts::OS,
        std::env::consts::ARCH,
        std::thread::available_parallelism()?.get(),
        std::env::var("PROCESSOR_IDENTIFIER").unwrap_or_else(|_| "unavailable".into()),
        command("rustc", &["-Vv"]),
        command("git", &["rev-parse", "HEAD"])
    )?;
    writeln!(
        report,
        "\nGeometry: deterministic flat tangent patches, {} triangles per mesh, {} meshes per chunk. Rocket: radius 0.5 m spherical physical proxy. Fixed dt: {} s; flight frames: {}.\n\nReal-time deadlines include streaming/check work. Throughput has no pacing. Cold rebuilds evicted geometry; warm prewarms the nominal route and caches removed geometry. Startup/prewarm is outside frame statistics. Queues are bounded by --in-flight; results publish at frame boundaries. One-step workload lookahead is used. Bulk has no pacing/physics and is run once per preparation-worker setting, independent of physics workers.\n\nTracked memory counts only completed TriangleMesh/BVH buffer capacity, excluding world/query/contact indices, queued/in-flight allocations, metadata, process RSS and allocator overhead. Allocation counts and process peak memory: unavailable. Current means end of measurement before teardown. Contact error: penetration > 0.01 m while the pre-step downward ray finds terrain. Coverage misses count missing downward-ray terrain, independently of penetration. BVH IDs verify actual reuse; successful build count is process-wide instrumentation. A completed workload is not a correctness PASS; inspect counters. Repetitions are reported individually; no measurements are fabricated for missing phases.\n",
        2 * config.grid * config.grid,
        config.meshes,
        config.dt,
        config.frames()
    )?;
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(1)
        .build()?;
    let max_workers = config
        .physics_workers
        .iter()
        .copied()
        .max()
        .ok_or("no physics workers")?;
    let physics = RayonPool::new(runtime.handle(), max_workers)?;
    let mut executor = physics.executor()?;
    let mut failed = false;
    for &prepare_workers in &config.prepare_workers {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(prepare_workers)
            .build()?;
        for &count in &config.active_chunks {
            for cache in &config.caches {
                for repeat in 1..=config.repeats {
                    for scenario in &config.scenarios {
                        if scenario == "bulk" {
                            for api in ["collider", "chunk", "batch"] {
                                let sizes: &[usize] = if api == "batch" {
                                    &config.batch_sizes
                                } else {
                                    &[1]
                                };
                                for &size in sizes {
                                    let label = format!(
                                        "bulk api={api} chunks={count} batch={size} prep={prepare_workers} physics=N/A cache={cache} repeat={repeat}"
                                    );
                                    println!("{label}");
                                    let stats = support::bulk(
                                        &config,
                                        count,
                                        size,
                                        api,
                                        cache == "warm",
                                        &pool,
                                    );
                                    failed |= stats.error.is_some()
                                        || stats.reuse_errors > 0
                                        || stats.contact_errors > 0
                                        || stats.stale_ids > 0;
                                    support::report_run(&mut report, &label, &stats)?;
                                }
                            }
                        } else {
                            for &workers in &config.physics_workers {
                                for mode in &config.modes {
                                    executor.set_parallelism(workers);
                                    let label = format!(
                                        "{scenario} chunks={count} prep={prepare_workers} physics={workers} cache={cache} mode={mode} repeat={repeat}"
                                    );
                                    println!("{label}");
                                    let stats = support::flight(
                                        &config,
                                        support::FlightRun {
                                            scenario,
                                            count,
                                            warm: cache == "warm",
                                            realtime: mode == "realtime",
                                            physics_workers: workers,
                                        },
                                        &pool,
                                        &executor,
                                    );
                                    failed |= stats.error.is_some()
                                        || stats.reuse_errors > 0
                                        || stats.contact_errors > 0
                                        || stats.stale_ids > 0;
                                    support::report_run(&mut report, &label, &stats)?;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    if let Some(parent) = config.output.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(&config.output, report)?;
    println!(
        "Report: {}",
        std::fs::canonicalize(&config.output)?.display()
    );
    if failed {
        return Err("one or more workloads failed; see the generated report".into());
    }
    Ok(())
}

fn command(program: &str, args: &[&str]) -> String {
    match Command::new(program).args(args).output() {
        Ok(output) if output.status.success() => {
            String::from_utf8_lossy(&output.stdout).trim().to_owned()
        }
        Ok(output) => format!("unavailable (exit {})", output.status),
        Err(error) => format!("unavailable ({error})"),
    }
}
