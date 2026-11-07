use super::{CollisionError, SolverBody, SolverBodyAccess};
use std::marker::PhantomData;

/// Exclusive access to the island body array while one conflict-free color runs.
pub(super) struct SharedSolverBodies<'a> {
    pointer: *mut SolverBody,
    length: usize,
    _borrow: PhantomData<&'a mut [SolverBody]>,
}

// Safety: every dynamic body appears in at most one constraint in a color and
// coloring is validated unconditionally before this wrapper is used. Static
// and kinematic bodies are only read. The executor joins all callbacks before
// the wrapper or the color changes.
unsafe impl Send for SharedSolverBodies<'_> {}
unsafe impl Sync for SharedSolverBodies<'_> {}

impl<'a> SharedSolverBodies<'a> {
    pub(super) fn new(bodies: &'a mut [SolverBody]) -> Self {
        Self {
            pointer: bodies.as_mut_ptr(),
            length: bodies.len(),
            _borrow: PhantomData,
        }
    }

    /// # Safety
    /// `indices` must identify one constraint in a validated color. No other
    /// concurrent task in that color may reference either dynamic endpoint.
    pub(super) unsafe fn pair(
        &self,
        indices: [usize; 2],
    ) -> Result<ColoredSolverBodyPair<'_>, CollisionError> {
        if indices[0] == indices[1] || indices.iter().any(|index| *index >= self.length) {
            return Err(CollisionError::InvalidContact);
        }
        // SAFETY: the caller processes only constraints in one validated color.
        // That guarantees distinct dynamic endpoints; read-only endpoints may
        // be shared between tasks. The pair is local to one task and never escapes it.
        unsafe {
            let body_a = self.pointer.add(indices[0]);
            let body_b = self.pointer.add(indices[1]);
            Ok(ColoredSolverBodyPair {
                bodies: [colored_body_ref(body_a), colored_body_ref(body_b)],
            })
        }
    }
}

enum ColoredSolverBodyRef<'a> {
    ReadOnly(&'a SolverBody),
    Dynamic(&'a mut SolverBody),
}

pub(super) struct ColoredSolverBodyPair<'a> {
    bodies: [ColoredSolverBodyRef<'a>; 2],
}

/// # Safety
/// The pointer must identify a live body held by `SharedSolverBodies`, and
/// mutable access must be unique for the active color.
unsafe fn colored_body_ref<'a>(pointer: *mut SolverBody) -> ColoredSolverBodyRef<'a> {
    // SAFETY: pointer comes from the bounded body array held by SharedSolverBodies.
    let is_dynamic =
        unsafe { (*pointer).body().physics().body_type() == crate::rigid_body::BodyType::Dynamic };
    if is_dynamic {
        // SAFETY: the color ownership invariant gives this task sole access to
        // this dynamic body until the executor finishes the current color.
        ColoredSolverBodyRef::Dynamic(unsafe { &mut *pointer })
    } else {
        // SAFETY: nondynamic bodies are never mutated by the solver.
        ColoredSolverBodyRef::ReadOnly(unsafe { &*pointer })
    }
}

impl SolverBodyAccess for ColoredSolverBodyPair<'_> {
    fn len(&self) -> usize {
        self.bodies.len()
    }

    fn get(&self, index: usize) -> Option<&SolverBody> {
        match self.bodies.get(index)? {
            ColoredSolverBodyRef::ReadOnly(body) => Some(body),
            ColoredSolverBodyRef::Dynamic(body) => Some(body),
        }
    }

    fn get_dynamic_mut(&mut self, index: usize) -> Option<&mut SolverBody> {
        match self.bodies.get_mut(index)? {
            ColoredSolverBodyRef::ReadOnly(_) => None,
            ColoredSolverBodyRef::Dynamic(body) => Some(body),
        }
    }
}
