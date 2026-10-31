use std::{
    collections::HashMap,
    error::Error,
    hint::black_box,
    mem::size_of,
    time::{Duration, Instant},
};

use kapitsa_core::{
    collider::Collider,
    collision::{CollisionError, CollisionPair, ContactInfo},
    rigid_body::RigidBody,
    shape::{Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};

const SIZES: [usize; 4] = [100, 1_000, 5_000, 20_000];
const WARMUPS: usize = 1;
const SAMPLES: usize = 5;

#[derive(Clone, Copy)]
struct Interaction {
    pair: CollisionPair,
    sensor: bool,
    transient: bool,
    contact: Option<ContactInfo>,
    impact: bool,
}

struct Sample {
    elapsed: Duration,
    retained_bytes: usize,
    interaction_count: usize,
}

fn main() -> Result<(), Box<dyn Error>> {
    let max_pairs = *SIZES.last().ok_or("missing benchmark size")?;
    let pairs = build_pairs(max_pairs)?;
    println!(
        "Steady-state interaction upsert benchmark: {WARMUPS} warm-up, {SAMPLES} samples; containers reserve M entries before timing; each M includes M persistent sensor interactions followed by M duplicate CCD sensor interactions"
    );
    println!(
        "Retained bytes estimate vector and HashMap entry payload capacity; excludes HashMap control bytes and allocator overhead."
    );

    for count in SIZES {
        let workload = pairs.get(..count).ok_or("insufficient benchmark pairs")?;
        let linear = measure(workload, upsert_linear)?;
        let indexed = measure(workload, upsert_indexed)?;
        println!(
            "M={count:>5} inputs={:>6} | linear={}/{}/{} ms {} B | indexed={}/{}/{} ms {} B | speedup={:.1}x",
            count * 2,
            millis(min(&linear).elapsed),
            millis(median(&linear).elapsed),
            millis(max(&linear).elapsed),
            median(&linear).retained_bytes,
            millis(min(&indexed).elapsed),
            millis(median(&indexed).elapsed),
            millis(max(&indexed).elapsed),
            median(&indexed).retained_bytes,
            median(&linear).elapsed.as_secs_f64() / median(&indexed).elapsed.as_secs_f64(),
        );
        black_box((
            median(&linear).interaction_count,
            median(&indexed).interaction_count,
        ));
    }
    Ok(())
}

fn build_pairs(count: usize) -> Result<Vec<CollisionPair>, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let mut colliders = Vec::with_capacity(count * 2);
    for _ in 0..count * 2 {
        colliders.push(world.add_collider(
            body,
            Collider::new(Shape::Sphere(Sphere::new(0.5)?), Transform::identity()),
        )?);
    }
    colliders
        .as_chunks::<2>()
        .0
        .iter()
        .map(|pair| CollisionPair::new(pair[0], pair[1]).ok_or_else(|| "duplicate pair IDs".into()))
        .collect()
}

fn measure(
    pairs: &[CollisionPair],
    upsert: fn(&[CollisionPair]) -> Result<Sample, CollisionError>,
) -> Result<Vec<Sample>, Box<dyn Error>> {
    for _ in 0..WARMUPS {
        black_box(upsert(black_box(pairs))?);
    }
    let mut samples = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        samples.push(upsert(black_box(pairs))?);
    }
    samples.sort_unstable_by_key(|sample| sample.elapsed);
    Ok(samples)
}

fn upsert_linear(pairs: &[CollisionPair]) -> Result<Sample, CollisionError> {
    let mut interactions = Vec::<Interaction>::with_capacity(pairs.len());
    let started = Instant::now();
    for &pair in pairs {
        upsert_linear_one(&mut interactions, interaction(pair, false))?;
    }
    for &pair in pairs {
        upsert_linear_one(&mut interactions, interaction(pair, true))?;
    }
    let elapsed = started.elapsed();
    black_box(&interactions);
    Ok(Sample {
        elapsed,
        retained_bytes: interactions.capacity() * size_of::<Interaction>(),
        interaction_count: interactions.len(),
    })
}

fn upsert_linear_one(
    interactions: &mut Vec<Interaction>,
    incoming: Interaction,
) -> Result<(), CollisionError> {
    if let Some(existing) = interactions
        .iter_mut()
        .find(|existing| existing.pair == incoming.pair)
    {
        merge(existing, incoming)?;
    } else {
        interactions
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        interactions.push(incoming);
    }
    Ok(())
}

fn upsert_indexed(pairs: &[CollisionPair]) -> Result<Sample, CollisionError> {
    let mut interactions = Vec::<Interaction>::with_capacity(pairs.len());
    let mut indices = HashMap::<CollisionPair, usize>::with_capacity(pairs.len());
    let started = Instant::now();
    for &pair in pairs {
        upsert_indexed_one(&mut interactions, &mut indices, interaction(pair, false))?;
    }
    for &pair in pairs {
        upsert_indexed_one(&mut interactions, &mut indices, interaction(pair, true))?;
    }
    let elapsed = started.elapsed();
    black_box((&interactions, &indices));
    Ok(Sample {
        elapsed,
        retained_bytes: interactions.capacity() * size_of::<Interaction>()
            + indices.capacity() * size_of::<(CollisionPair, usize)>(),
        interaction_count: interactions.len(),
    })
}

fn upsert_indexed_one(
    interactions: &mut Vec<Interaction>,
    indices: &mut HashMap<CollisionPair, usize>,
    incoming: Interaction,
) -> Result<(), CollisionError> {
    if let Some(&index) = indices.get(&incoming.pair) {
        let existing = interactions
            .get_mut(index)
            .ok_or(CollisionError::InvalidContact)?;
        merge(existing, incoming)?;
        return Ok(());
    }
    interactions
        .try_reserve(1)
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    indices
        .try_reserve(1)
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    let index = interactions.len();
    interactions.push(incoming);
    indices.insert(incoming.pair, index);
    Ok(())
}

fn merge(existing: &mut Interaction, incoming: Interaction) -> Result<(), CollisionError> {
    if existing.sensor != incoming.sensor {
        return Err(CollisionError::InvalidContact);
    }
    existing.transient &= incoming.transient;
    if incoming.contact.is_some() {
        existing.contact = incoming.contact;
    }
    existing.impact |= incoming.impact;
    Ok(())
}

fn interaction(pair: CollisionPair, transient: bool) -> Interaction {
    Interaction {
        pair,
        sensor: true,
        transient,
        contact: None,
        impact: false,
    }
}

fn min(samples: &[Sample]) -> &Sample {
    samples.first().expect("samples are collected")
}

fn median(samples: &[Sample]) -> &Sample {
    samples
        .get(samples.len() / 2)
        .expect("samples are collected")
}

fn max(samples: &[Sample]) -> &Sample {
    samples.last().expect("samples are collected")
}

fn millis(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1_000.0
}
