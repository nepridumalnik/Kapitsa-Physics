#[cfg(feature = "parallel")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    use kapitsa::{PhysicsSystem, PhysicsWorld};
    use std::sync::Arc;

    let pool = Arc::new(rayon::ThreadPoolBuilder::new().num_threads(2).build()?);
    let mut physics = PhysicsSystem::builder(PhysicsWorld::new())
        .rayon_pool(pool)
        .build()?;
    println!("{:?}", physics.update(1.0 / 120.0)?.diagnostics());
    Ok(())
}

#[cfg(not(feature = "parallel"))]
fn main() {
    eprintln!("Run this example with --features parallel");
}
