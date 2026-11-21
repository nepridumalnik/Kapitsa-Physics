use kapitsa::{PhysicsSystem, PhysicsWorld};

fn main() -> Result<(), kapitsa::Error> {
    let mut system = PhysicsSystem::builder(PhysicsWorld::new()).build()?;
    for dt in [1.0 / 120.0, 1.0 / 240.0] {
        let results = system.update(dt)?;
        println!("{:?}", results.diagnostics());
    }
    Ok(())
}
