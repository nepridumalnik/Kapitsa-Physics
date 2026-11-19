use super::{CollisionError, SolverBody, SolverBodyAccess};

/// Index-backed ownership for one complete color. Dynamic entries are
/// consumed once; read-only entries can be borrowed repeatedly.
pub(super) struct BodyBinding<'a> {
    slots: Vec<BodySlot<'a>>,
}

enum BodySlot<'a> {
    Dynamic(Option<&'a mut SolverBody>),
    ReadOnly(&'a SolverBody),
}

pub(super) enum BoundBodyRef<'a> {
    Dynamic(&'a mut SolverBody),
    ReadOnly(&'a SolverBody),
}

pub(super) struct BoundPair<'a>(pub(super) [BoundBodyRef<'a>; 2]);

impl SolverBodyAccess for BoundPair<'_> {
    fn len(&self) -> usize {
        2
    }
    fn get(&self, index: usize) -> Option<&SolverBody> {
        match self.0.get(index)? {
            BoundBodyRef::Dynamic(body) => Some(body),
            BoundBodyRef::ReadOnly(body) => Some(body),
        }
    }
    fn get_dynamic_mut(&mut self, index: usize) -> Option<&mut SolverBody> {
        match self.0.get_mut(index)? {
            BoundBodyRef::Dynamic(body) => Some(body),
            BoundBodyRef::ReadOnly(_) => None,
        }
    }
}

impl BoundBodyRef<'_> {
    pub(super) fn reborrow(&mut self) -> BoundBodyRef<'_> {
        match self {
            Self::Dynamic(body) => BoundBodyRef::Dynamic(body),
            Self::ReadOnly(body) => BoundBodyRef::ReadOnly(body),
        }
    }
}

impl<'a> BodyBinding<'a> {
    pub(super) fn bind(
        bodies: &'a mut [SolverBody],
        indices: &[usize],
    ) -> Result<Self, crate::Error> {
        if indices.windows(2).any(|pair| pair[0] >= pair[1])
            || indices.last().is_some_and(|index| *index >= bodies.len())
        {
            return Err(crate::Error::from(CollisionError::InvalidContact));
        }
        let mut remaining = bodies;
        let mut base = 0;
        let mut slots = Vec::with_capacity(indices.len());
        for &index in indices {
            let (prefix, tail) = remaining.split_at_mut(index - base + 1);
            let body = prefix
                .last_mut()
                .ok_or_else(|| crate::Error::from(CollisionError::InvalidContact))?;
            slots.push(
                if body.body().physics().body_type() == crate::rigid_body::BodyType::Dynamic {
                    BodySlot::Dynamic(Some(body))
                } else {
                    BodySlot::ReadOnly(&*body)
                },
            );
            remaining = tail;
            base = index + 1;
        }
        Ok(Self { slots })
    }

    pub(super) fn pair_at(
        &mut self,
        [a, b]: [usize; 2],
    ) -> Result<[BoundBodyRef<'a>; 2], crate::Error> {
        if a == b || a >= self.slots.len() || b >= self.slots.len() {
            return Err(crate::Error::from(CollisionError::InvalidContact));
        }
        let (lo, hi, reversed) = if a < b { (a, b, false) } else { (b, a, true) };
        let (left, right) = self.slots.split_at_mut(hi);
        let first = take_body(&mut left[lo])?;
        let second = take_body(&mut right[0])?;
        Ok(if reversed {
            [second, first]
        } else {
            [first, second]
        })
    }
}

fn take_body<'a>(slot: &mut BodySlot<'a>) -> Result<BoundBodyRef<'a>, crate::Error> {
    match slot {
        BodySlot::Dynamic(body) => {
            Ok(BoundBodyRef::Dynamic(body.take().ok_or_else(|| {
                crate::Error::from(CollisionError::InvalidContact)
            })?))
        }
        BodySlot::ReadOnly(body) => Ok(BoundBodyRef::ReadOnly(body)),
    }
}
