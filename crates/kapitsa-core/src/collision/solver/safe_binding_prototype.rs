//! Safe ownership prototype for sparse color bindings.

enum Slot<'a, T> {
    Dynamic(Option<&'a mut T>),
    ReadOnly(&'a T),
}

struct Binding<'a, T> {
    slots: Vec<Slot<'a, T>>,
}

impl<'a, T> Binding<'a, T> {
    fn bind(
        values: &'a mut [T],
        indices: &[usize],
        mut is_dynamic: impl FnMut(&T) -> bool,
    ) -> Option<Self> {
        if indices.windows(2).any(|pair| pair[0] >= pair[1])
            || indices.last().is_some_and(|index| *index >= values.len())
        {
            return None;
        }
        let mut remaining = values;
        let mut base = 0;
        let mut slots = Vec::with_capacity(indices.len());
        for &index in indices {
            let (prefix, tail) = remaining.split_at_mut(index - base + 1);
            let value = prefix.last_mut()?;
            slots.push(if is_dynamic(value) {
                Slot::Dynamic(Some(value))
            } else {
                Slot::ReadOnly(&*value)
            });
            remaining = tail;
            base = index + 1;
        }
        Some(Self { slots })
    }

    fn pair(&mut self, indices: [usize; 2], all: &[usize]) -> Option<[Ref<'a, T>; 2]> {
        let a = all.binary_search(&indices[0]).ok()?;
        let b = all.binary_search(&indices[1]).ok()?;
        if a == b {
            return None;
        }
        let (lo, hi, reversed) = if a < b { (a, b, false) } else { (b, a, true) };
        let (left, right) = self.slots.split_at_mut(hi);
        let first = take_ref(&mut left[lo])?;
        let second = take_ref(&mut right[0])?;
        Some(if reversed {
            [second, first]
        } else {
            [first, second]
        })
    }
}

enum Ref<'a, T> {
    Dynamic(&'a mut T),
    ReadOnly(&'a T),
}

fn take_ref<'a, T>(slot: &mut Slot<'a, T>) -> Option<Ref<'a, T>> {
    match slot {
        Slot::Dynamic(value) => Some(Ref::Dynamic(value.take()?)),
        Slot::ReadOnly(value) => Some(Ref::ReadOnly(*value)),
    }
}

#[test]
fn binds_arbitrary_indices_and_reuses_read_only_endpoints() {
    let mut values = [0, 1, 2, 3, 4, 5];
    let indices = [0, 2, 4, 5];
    let mut binding = Binding::bind(&mut values, &indices, |value| *value != 2).unwrap();
    let mut first = binding.pair([4, 0], &indices).unwrap();
    if let Ref::Dynamic(value) = &mut first[0] {
        **value = 40;
    }
    if let Ref::Dynamic(value) = &mut first[1] {
        **value = 0;
    }
    let second = binding.pair([2, 5], &indices).unwrap();
    assert!(matches!(second[0], Ref::ReadOnly(_)));
    assert!(matches!(second[1], Ref::Dynamic(_)));
    assert_eq!(values, [0, 1, 2, 3, 40, 5]);
}

#[test]
fn rejects_duplicates_unsorted_and_out_of_bounds() {
    let mut values = [0, 1, 2];
    assert!(Binding::bind(&mut values, &[1, 1], |_| true).is_none());
    assert!(Binding::bind(&mut values, &[2, 1], |_| true).is_none());
    assert!(Binding::bind(&mut values, &[0, 3], |_| true).is_none());
}

#[test]
fn rejects_duplicate_dynamic_pair_but_allows_repeated_read_only() {
    let mut values = [0, 1, 2, 3];
    let indices = [0, 1, 2, 3];
    let mut binding = Binding::bind(&mut values, &indices, |value| *value != 1).unwrap();
    assert!(binding.pair([0, 0], &indices).is_none());
    assert!(binding.pair([1, 2], &indices).is_some());
    assert!(binding.pair([1, 3], &indices).is_some());
}
