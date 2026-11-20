# Surface visibility

## ADDED Requirements

### Requirement: Preserve representable numeric intersections

The surface-visibility intersection calculations SHALL handle finite endpoint
values whose direct difference overflows when the intersection remains
representable. Crossing fractions SHALL use a ratio-preserving rescaling in
that case, and coordinate interpolation SHALL avoid an overflowing endpoint
difference where the alternative weighted form is representable. Genuine
non-finite results and precision loss SHALL remain errors rather than being
silently converted into a missed intersection.

#### Scenario: Finite distances have an overflowing difference

- **WHEN** opposite-signed finite distances have a difference outside the
  finite `f64` range
- **THEN** the crossing fraction is computed after a common rescaling that
  preserves the ratio, and a representable fraction is retained

#### Scenario: Endpoint coordinate subtraction overflows

- **WHEN** an intersection coordinate's endpoint subtraction is non-finite
  but the weighted endpoint interpolation is finite
- **THEN** the weighted interpolation form is used and the representable
  intersection is retained

### Requirement: Check area consistency per source partition

The clipping result SHALL check conservation of the source triangle's local
barycentric area after partition reconstruction. Its tolerance SHALL be a
heuristic based on work charged for the current source partition, including
the initial triangle contribution, rather than on unrelated prior sources or
global preprocessing work. The tolerance SHALL be documented as heuristic and
not as a proven numerical error bound.

#### Scenario: A source triangle is split into clipping pieces

- **WHEN** the reconstructed piece fractions are summed
- **THEN** the sum is compared with that source's initial visible fraction
  using the source-local heuristic allowance

#### Scenario: The reconstructed area is inconsistent

- **WHEN** the source-local area check fails or produces a non-finite result
- **THEN** surface preparation returns a precision error instead of emitting
  an inconsistent partition

### Requirement: Reuse clipping scratch storage

The clipping loop SHALL reuse the allocations for its two polygon lists and
the vertex-distance scratch buffer across clipping planes within each source
partition where the existing ownership flow permits. Reuse SHALL NOT weaken
fallible allocation handling or work accounting.

#### Scenario: Multiple clipping planes process one source partition

- **WHEN** a clipping plane is applied after a prior plane
- **THEN** polygon buffers are exchanged and the distance buffer is cleared
  and reused rather than recreated for every polygon or plane

### Requirement: Charge and propagate the remaining hard work budget

Surface preparation SHALL enforce the shared hard cap and pass the remaining
budget to operations that need to charge work, including grid construction and
candidate queries. Work units SHALL be treated as heuristic accounting
weights for bounded effort, not as an exact bound on processor instructions,
elapsed time, allocator overhead, or bytes allocated. No exact byte budget is
claimed.

#### Scenario: A downstream operation receives the remaining budget

- **WHEN** earlier preparation work has consumed part of the hard cap
- **THEN** downstream grid or query work receives only the remaining budget,
  and charged work beyond the cap is rejected

#### Scenario: Allocation overhead is considered

- **WHEN** vectors grow or temporary structures coexist
- **THEN** their logical element work may be charged, while the specification
  explicitly does not represent allocator overhead or exact bytes

#### Scenario: Work accounting is incomplete

- **WHEN** an intermediate operation is not yet fully covered by the hard
  work audit
- **THEN** the remaining hard-work audit is tracked as outstanding work and
  is not presented as a final completeness or validation claim
