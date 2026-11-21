use kapitsa_core::{
    Error,
    execution::SequentialExecutor,
    world::{PhysicsWorld, StepResults},
};

#[cfg(feature = "parallel")]
use {super::RayonExecutor, kapitsa_core::ErrorKind, std::sync::Arc};

/// A physics world advanced explicitly by its host application.
pub struct PhysicsSystem {
    world: PhysicsWorld,
    #[cfg(feature = "parallel")]
    executor: Option<RayonExecutor>,
}

/// Configures a host-driven physics system.
pub struct PhysicsSystemBuilder {
    world: PhysicsWorld,
    #[cfg(feature = "parallel")]
    pool: PoolSelection,
}

#[cfg(feature = "parallel")]
enum PoolSelection {
    Sequential,
    Owned(std::num::NonZeroUsize),
    Shared(Arc<rayon::ThreadPool>),
}

impl PhysicsSystemBuilder {
    /// Builds a private Rayon pool with the requested positive worker count.
    #[cfg(feature = "parallel")]
    pub fn workers(mut self, count: std::num::NonZeroUsize) -> Self {
        self.pool = PoolSelection::Owned(count);
        self
    }

    /// Shares an existing Rayon pool without taking exclusive ownership.
    #[cfg(feature = "parallel")]
    pub fn rayon_pool(mut self, pool: Arc<rayon::ThreadPool>) -> Self {
        self.pool = PoolSelection::Shared(pool);
        self
    }

    /// Builds the system without starting a scheduler or a Tokio runtime.
    pub fn build(self) -> Result<PhysicsSystem, Error> {
        #[cfg(feature = "parallel")]
        let executor = match self.pool {
            PoolSelection::Sequential => None,
            PoolSelection::Shared(pool) => Some(RayonExecutor::from_pool(pool)),
            PoolSelection::Owned(count) => {
                let pool = rayon::ThreadPoolBuilder::new()
                    .num_threads(count.get())
                    .build()
                    .map_err(|cause| Error::from_source(ErrorKind::Runtime, cause))?;
                Some(RayonExecutor::from_pool(Arc::new(pool)))
            }
        };
        Ok(PhysicsSystem {
            world: self.world,
            #[cfg(feature = "parallel")]
            executor,
        })
    }
}

impl PhysicsSystem {
    /// Configures a system that takes ownership of `world`.
    pub fn builder(world: PhysicsWorld) -> PhysicsSystemBuilder {
        PhysicsSystemBuilder {
            world,
            #[cfg(feature = "parallel")]
            pool: PoolSelection::Sequential,
        }
    }

    /// Borrows the world and its latest committed state.
    pub fn world(&self) -> &PhysicsWorld {
        &self.world
    }

    /// Allows direct world edits between updates.
    pub fn world_mut(&mut self) -> &mut PhysicsWorld {
        &mut self.world
    }

    /// Completes exactly one blocking step of `dt` seconds.
    /// Returns a borrowed view only after the step has committed successfully.
    pub fn update(&mut self, dt: f64) -> Result<StepResults<'_>, Error> {
        #[cfg(feature = "parallel")]
        if let Some(executor) = &self.executor {
            self.world
                .step_with_executor(dt, executor, executor.worker_count())?;
            return Ok(self.world.committed_step_results());
        }
        self.world.step_with_executor(dt, &SequentialExecutor, 1)?;
        Ok(self.world.committed_step_results())
    }
}

#[cfg(all(test, feature = "parallel"))]
#[test]
fn private_pool_runs_scoped_work_on_multiple_threads_and_is_released()
-> Result<(), Box<dyn std::error::Error>> {
    use kapitsa_core::execution::StepExecutor;
    use std::{num::NonZeroUsize, sync::Barrier, thread};

    let mut system = PhysicsSystem::builder(PhysicsWorld::new())
        .workers(NonZeroUsize::new(2).ok_or("positive worker count required")?)
        .build()?;
    let executor = system.executor.as_ref().ok_or("missing private executor")?;
    let pool = executor.pool_weak();
    let rendezvous = Barrier::new(2);
    let identify_worker = || {
        rendezvous.wait();
        (thread::current().id(), rayon::current_thread_index())
    };
    let (left, right) = executor.join(identify_worker, identify_worker);
    assert_ne!(left.0, right.0);
    assert!(left.1.is_some() && right.1.is_some());
    system.update(0.01)?;
    drop(system);
    assert!(pool.upgrade().is_none());
    Ok(())
}
