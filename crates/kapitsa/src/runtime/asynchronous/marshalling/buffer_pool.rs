use std::sync::Mutex;

use super::command::{PhysicsCommand, PhysicsCommandBuffer};

const MAX_POOLED_BUFFERS: usize = 16;
const MAX_POOL_MEMORY_BYTES: usize = 256 * 1024;

struct PoolState {
    buffers: Vec<Vec<PhysicsCommand>>,
    retained_bytes: usize,
}

impl Default for PoolState {
    fn default() -> Self {
        let buffers = Vec::with_capacity(MAX_POOLED_BUFFERS);
        let retained_bytes = buffers.capacity() * std::mem::size_of::<Vec<PhysicsCommand>>();
        Self {
            buffers,
            retained_bytes,
        }
    }
}

#[derive(Default)]
pub(crate) struct CommandBufferPool {
    state: Mutex<PoolState>,
}

const POOL_FIXED_BYTES: usize =
    std::mem::size_of::<CommandBufferPool>() + 2 * std::mem::size_of::<usize>();
const MAX_RETAINED_BUFFER_BYTES: usize = MAX_POOL_MEMORY_BYTES.saturating_sub(POOL_FIXED_BYTES);

impl CommandBufferPool {
    pub(crate) fn take(&self) -> Vec<PhysicsCommand> {
        let Ok(mut state) = self.state.try_lock() else {
            return Vec::new();
        };
        let Some(commands) = state.buffers.pop() else {
            return Vec::new();
        };
        state.retained_bytes -= command_buffer_bytes(commands.capacity());
        commands
    }

    pub(crate) fn recycle(&self, mut buffer: PhysicsCommandBuffer) {
        buffer.commands.clear();
        let command_bytes = command_buffer_bytes(buffer.commands.capacity());
        if command_bytes == 0 {
            return;
        }

        let Ok(mut state) = self.state.try_lock() else {
            return;
        };
        if state.buffers.len() == MAX_POOLED_BUFFERS
            || state.retained_bytes.saturating_add(command_bytes) > MAX_RETAINED_BUFFER_BYTES
        {
            return;
        }

        state.retained_bytes += command_bytes;
        state.buffers.push(std::mem::take(&mut buffer.commands));
    }

    #[cfg(test)]
    pub(crate) fn stats(&self) -> (usize, usize) {
        let Ok(state) = self.state.try_lock() else {
            return (0, 0);
        };
        (state.buffers.len(), state.retained_bytes)
    }

    #[cfg(test)]
    pub(crate) fn fixed_memory_bytes() -> usize {
        POOL_FIXED_BYTES
    }
}

fn command_buffer_bytes(capacity: usize) -> usize {
    capacity.saturating_mul(std::mem::size_of::<PhysicsCommand>())
}

#[cfg(test)]
mod tests {
    use super::{
        CommandBufferPool, MAX_POOL_MEMORY_BYTES, MAX_POOLED_BUFFERS, MAX_RETAINED_BUFFER_BYTES,
    };
    use crate::runtime::asynchronous::marshalling::PhysicsCommandBuffer;
    use kapitsa_core::{
        mass::MassProperties, rigid_body::RigidBody, transform::Transform, world::PhysicsWorld,
    };
    use kapitsa_math::{Mat3, Vec3};

    #[test]
    fn recycle_clears_commands_and_preserves_capacity() {
        let pool = CommandBufferPool::default();
        let list_bytes = pool.stats().1;
        let mut buffer = PhysicsCommandBuffer::with_capacity(8);
        let mut world = PhysicsWorld::new();
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass");
        let body = world
            .add_body(RigidBody::new_dynamic(Transform::identity(), mass).expect("body"))
            .expect("body ID");
        buffer.set_linear_velocity(body, Vec3::X);
        let capacity = buffer.commands.capacity();

        pool.recycle(buffer);

        assert_eq!(
            pool.stats(),
            (
                1,
                list_bytes + capacity * std::mem::size_of::<super::PhysicsCommand>()
            )
        );
        let commands = pool.take();
        assert!(commands.is_empty());
        assert_eq!(commands.capacity(), capacity);
        assert_eq!(pool.stats(), (0, list_bytes));

        let mut reused = PhysicsCommandBuffer { commands };
        reused.set_linear_velocity(body, Vec3::Y);
        pool.recycle(reused);
        let commands = pool.take();
        assert!(commands.is_empty());
        assert_eq!(commands.capacity(), capacity);
        assert_eq!(pool.stats(), (0, list_bytes));
    }

    #[test]
    fn pool_bounds_buffer_count_and_retained_capacity_bytes() {
        let pool = CommandBufferPool::default();
        for _ in 0..MAX_POOLED_BUFFERS + 1 {
            pool.recycle(PhysicsCommandBuffer::with_capacity(1));
        }
        assert_eq!(pool.stats().0, MAX_POOLED_BUFFERS);

        let pool = CommandBufferPool::default();
        let list_bytes = pool.stats().1;
        let available_bytes = MAX_RETAINED_BUFFER_BYTES - list_bytes;
        let oversized = available_bytes / std::mem::size_of::<super::PhysicsCommand>() + 1;
        pool.recycle(PhysicsCommandBuffer::with_capacity(oversized));
        assert_eq!(pool.stats().0, 0);

        let pool = CommandBufferPool::default();
        let list_bytes = pool.stats().1;
        let available_bytes = MAX_RETAINED_BUFFER_BYTES - list_bytes;
        let capacity = available_bytes / (2 * std::mem::size_of::<super::PhysicsCommand>());
        pool.recycle(PhysicsCommandBuffer::with_capacity(capacity));
        pool.recycle(PhysicsCommandBuffer::with_capacity(capacity));
        pool.recycle(PhysicsCommandBuffer::with_capacity(capacity));
        assert!(pool.stats().1 + CommandBufferPool::fixed_memory_bytes() <= MAX_POOL_MEMORY_BYTES);
        assert_eq!(pool.stats().0, 2);
    }

    #[test]
    fn lock_contention_uses_nonblocking_fallbacks() {
        let pool = CommandBufferPool::default();
        let _guard = pool.state.try_lock().expect("pool lock available");

        let commands = pool.take();
        assert!(commands.is_empty());
        pool.recycle(PhysicsCommandBuffer::with_capacity(4));
    }
}
