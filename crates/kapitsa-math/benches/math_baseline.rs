use std::{hint::black_box, time::Instant};

use kapitsa_math::{Mat3, Quat, Vec3};

const ITERATIONS: u64 = 1_000_000;
const SAMPLES: usize = 7;

fn main() {
    let Some(rotation) = Quat::from_axis_angle(Vec3::new(1.0, 2.0, 3.0), 0.001) else {
        eprintln!("could not construct benchmark rotation");
        std::process::exit(1);
    };
    let Some(rotation_matrix) = rotation.to_mat3() else {
        eprintln!("could not construct benchmark matrix");
        std::process::exit(1);
    };
    let spd = Mat3::from_rows([[4.0, 1.0, 1.0], [1.0, 3.0, 0.0], [1.0, 0.0, 2.0]]);
    let rhs = Vec3::new(1.5, -2.0, 0.25);

    println!("kapitsa-math deterministic microbenchmarks");
    println!("iterations={ITERATIONS}, samples={SAMPLES}, statistic=median with min/max range");

    measure("Vec3 arithmetic", |iterations| {
        let direction = Vec3::new(0.25, -0.5, 0.75);
        let mut value = Vec3::new(1.0, 2.0, 3.0);
        let mut checksum = 0.0;
        for _ in 0..iterations {
            value = (value + direction).cross(direction) * 0.5 + direction;
            checksum += value.dot(direction);
        }
        black_box((value, checksum));
    });

    measure("Vec3 normalization", |iterations| {
        let direction = Vec3::new(0.25, -0.5, 0.75);
        let mut value = Vec3::new(1.0, 2.0, 3.0);
        let mut checksum = 0.0;
        for _ in 0..iterations {
            if let Some(normalized) = (value + direction).try_normalize() {
                value = normalized + direction * 0.001;
                checksum += value.x;
            }
        }
        black_box((value, checksum));
    });

    measure("Mat3 multiply", |iterations| {
        let mut value = rotation_matrix;
        let mut checksum = 0.0;
        for _ in 0..iterations {
            value = value * rotation_matrix;
            checksum += (value * Vec3::X).y;
        }
        black_box((value, checksum));
    });

    measure("Mat3 SPD solve", |iterations| {
        let mut value = rhs;
        let mut checksum = 0.0;
        for _ in 0..iterations {
            if let Ok(solution) = spd.solve_spd(value) {
                value = solution;
                checksum += value.x;
            }
        }
        black_box((value, checksum));
    });

    measure("Quat vector rotation", |iterations| {
        let mut value = Vec3::new(1.0, 2.0, 3.0);
        let mut checksum = 0.0;
        for _ in 0..iterations {
            if let Some(rotated) = rotation.rotate_vector(value) {
                value = rotated;
                checksum += value.y;
            }
        }
        black_box((value, checksum));
    });

    measure("Quat angular integration", |iterations| {
        let angular_velocity = Vec3::new(0.1, -0.2, 0.3);
        let mut orientation = Quat::IDENTITY;
        let mut checksum = 0.0;
        for _ in 0..iterations {
            if let Some(next) =
                orientation.integrate_world_angular_velocity(angular_velocity, 1.0 / 600.0)
            {
                orientation = next;
                checksum += orientation.w;
            }
        }
        black_box((orientation, checksum));
    });
}

fn measure(label: &str, mut kernel: impl FnMut(u64)) {
    let mut nanoseconds_per_iteration = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        let start = Instant::now();
        kernel(black_box(ITERATIONS));
        nanoseconds_per_iteration.push(start.elapsed().as_secs_f64() * 1.0e9 / ITERATIONS as f64);
    }

    nanoseconds_per_iteration.sort_by(f64::total_cmp);
    let minimum = nanoseconds_per_iteration[0];
    let median = nanoseconds_per_iteration[SAMPLES / 2];
    let maximum = nanoseconds_per_iteration[SAMPLES - 1];
    println!("{label:24} median={median:8.2} ns/iter  range={minimum:.2}..{maximum:.2}");
}
