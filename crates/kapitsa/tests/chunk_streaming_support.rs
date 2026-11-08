#![cfg(feature = "bench-internals")]

// Check shared parsing/geometry/publication/report code without benchmark workloads.
#[allow(dead_code)]
#[path = "../benches/chunk_streaming/support.rs"]
mod support;

use kapitsa_math::Vec3;
use std::{collections::HashSet, time::Duration};
use support::{BenchResult, Config, Stats};

#[test]
fn configuration_rejects_invalid_or_unbounded_workloads() {
    for args in [
        vec!["--prepare-workers", "0"],
        vec!["--physics-workers", "257"],
        vec!["--grid", "0"],
        vec!["--seconds", "NaN"],
        vec!["--mode", "unknown"],
        vec!["--cache", ""],
        vec!["--dt", "100"],
        vec!["--unknown", "1"],
    ] {
        assert!(Config::parse(args.into_iter().map(String::from)).is_err());
    }
}

#[test]
fn worker_axes_and_matrix_parameters_are_independent() -> BenchResult<()> {
    let config = Config::parse(
        [
            "--bench",
            "--prepare-workers",
            "1,8",
            "--physics-workers",
            "2,4",
            "--active-chunks",
            "4,16384",
            "--batch-sizes",
            "1,4,16,64,256,1024",
            "--seconds",
            "1",
            "--dt",
            "0.02",
        ]
        .map(String::from),
    )?;
    assert_eq!(config.prepare_workers, [1, 8]);
    assert_eq!(config.physics_workers, [2, 4]);
    assert_eq!(config.frames(), 50);
    assert_eq!(config.batch_sizes, [1, 4, 16, 64, 256, 1024]);
    Ok(())
}

#[test]
fn workload_routes_repeat_and_working_sets_are_unique() {
    assert_eq!(
        support::position("shuttle", 0, 0.02),
        support::position("shuttle", 400, 0.02)
    );
    assert_eq!(support::position("shuttle", 200, 0.02).x, 256.0);
    for count in [1, 4, 16, 31, 64, 1024] {
        let keys = support::working_set(Vec3::new(-0.5, 2.0, 0.1), count);
        assert_eq!(keys.len(), count);
        assert_eq!(keys.iter().copied().collect::<HashSet<_>>().len(), count);
        assert_eq!(keys.first(), Some(&(-1, 0)));
    }
}

#[test]
fn generated_meshes_tile_one_chunk_and_keep_preparation_identity() -> BenchResult<()> {
    let prepared = support::prepare((-2, 3), 2, 2)?;
    assert_eq!(prepared.chunk.colliders().len(), 2);
    assert_eq!(prepared.chunk.local_bounds().min(), Vec3::ZERO);
    assert_eq!(
        prepared.chunk.local_bounds().max(),
        Vec3::new(32.0, 0.0, 32.0)
    );
    assert_eq!(
        prepared.chunk.transform().translation(),
        Vec3::new(-64.0, 0.0, 96.0)
    );
    assert!(prepared.bytes > 0);
    assert_eq!(prepared.serials, support::serials(&prepared.chunk)?);
    Ok(())
}

#[test]
fn report_percentiles_use_nearest_rank_and_show_failures() -> BenchResult<()> {
    let samples: Vec<_> = (1..=100).map(Duration::from_millis).collect();
    assert_eq!(
        support::quantile(&samples, 50),
        Some(Duration::from_millis(50))
    );
    assert_eq!(
        support::quantile(&samples, 95),
        Some(Duration::from_millis(95))
    );
    assert_eq!(
        support::quantile(&samples, 99),
        Some(Duration::from_millis(99))
    );
    assert_eq!(support::quantile(&[], 99), None);
    let mut stats = Stats {
        error: Some("test failure".into()),
        ..Stats::default()
    };
    stats.samples.insert("physics step", samples);
    let mut text = String::new();
    support::report_run(&mut text, "fixture", &stats)?;
    assert!(text.contains("Status: test failure"));
    assert!(text.contains("50.000000 | 95.000000 | 99.000000"));
    Ok(())
}
