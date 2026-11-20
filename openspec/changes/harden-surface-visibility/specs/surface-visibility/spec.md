# Surface visibility

## ADDED Requirements

### Requirement: Scale surface predicates independently by axis

Visibility predicates SHALL use independent power-of-two scales for each
axis and preserve a common positive determinant factor within a frame.
Barycentric area calculations SHALL undo both in-plane scales.

#### Scenario: Synthetic barycentric area spans different axis scales

- **WHEN** `triangle_fraction_preserves_area_across_axis_scales` evaluates
  the triangle with vertices `(0, 0, 0)`, `(2^-500, 0, 0)`, and `(0, 1, 0)`
- **THEN** its barycentric area fraction is `2^-500` after reversing the two
  in-plane scales

#### Scenario: Synthetic thin source remains covered by its shadow

- **WHEN** `shadow_covers_thin_source_across_axis_scales` constructs a source
  with vertices `(0, 0, 0)`, `(0, 2^-500, 0)`, and `(0, 0, 1)`, a blocker
  translated by `(1, 0, 0)`, and visibility direction `(1, 0, 0)`
- **THEN** shadow construction succeeds and covers the source-affine point
  `(0.25, 0.25, 0)` despite the different coordinate-axis scales

#### Scenario: A direction or projected edge predicate exceeds its span

- **WHEN** a direction or projected edge predicate exceeds its supported
  exponent span
- **THEN** preparation precharges exact fallback instead of rejecting the
  frame span alone

#### Scenario: An aligned plane bypasses the 3D span limit

- **WHEN** a coordinate-aligned blocker plane has a wide within-axis span but
  its signed separation along the aligned normal axis is finite
- **THEN** axial depth evaluation does not reject it for the 3D predicate's
  exponent-span limit

### Requirement: Omit axial depth only from projected edge tests

For exactly axial flow, projected edge tests SHALL omit only that axial
coordinate. Aligned blocker depth SHALL use finite signed axial separation.
Overflowing separation or other planes SHALL use the general depth predicate:
try the 3D frame, then precharged exact fallback.

#### Scenario: Axial shadow uses signed separation for blocker depth

- **WHEN** a coordinate-aligned blocker is separated from the source by
  `1e-100` along the flow axis
- **THEN** the flow-facing signed separation covers the source and the
  opposite sign is rejected

#### Scenario: Wide within-axis span remains a valid axial blocker

- **WHEN** an axial-plane blocker has a source vertex offset by `1e-100` on a
  non-axial coordinate and the blocker is one unit downstream
- **THEN** the blocker covers the source-affine point `(0.25, 0.25, 0)` and
  the opposite flow direction rejects it

#### Scenario: Synthetic axial separation overflows

- **WHEN** `axial_depth_accepts_overflowing_separation` uses blocker vertices
  `[s, s + Y, s + Z]`, source vertices `[-s, -s + Y, -s + Z]`, and
  `s = X*f64::MAX`
- **THEN** the overflowing separation selects the general depth predicate;
  flow `X` covers affine point `(0.25, 0.25, 0)`, while `-X` yields no shadow
- **AND** this is a synthetic assertion, not execution evidence or independent
  physical validation

### Requirement: Fall back to exact orientation predicates

Frame-span rejection or direction scaling overflow, subnormalization or loss
to zero SHALL trigger precharged exact work. Direction and edge references
SHALL use sign only; three edge coefficients SHALL share a separate scale.
Nonzero coefficients rounding to zero SHALL return `PrecisionLoss`.

#### Scenario: Large determinant terms cancel

- **WHEN** a blocker at `t = (2^100, 2^100, 2^100)` has X/Y edges of `2^50`
  and source points `t + Z*2^50`, `t - Z*2^50`, and `t`
- **THEN** exact depth coefficients are `[-1, 1, 0]` despite cancellation

#### Scenario: Smallest subnormal is preserved or rejected on rounding

- **WHEN** blocker vertices are `[0, X, Y]` and `m` is the smallest positive
  binary64 subnormal
- **THEN** sources `[Z, Z*m, 0]` give `[-1, -m, 0]`, while
  `[2*Z, Z*m, 0]` returns `PrecisionLoss` on ties-to-even rounding to zero

#### Scenario: Inclined fallback precharge is rejected or accepted

- **WHEN** source vertices are `[0, Y + X*1e-100, Z]`, blocker vertices are
  `[X, 2*X + Y, X + Z]`, and flow is `X`
- **THEN** rejected precharge returns `WorkBudgetExceeded` before exact work;
  accepted precharge recovers coverage at affine point `(0.25, 0.25, 0)`

#### Scenario: Direction and edge frames use exact fallback

- **WHEN** direction rescaling produces a subnormal component, or a direction,
  depth, or projected-edge frame exceeds its exponent span
- **THEN** preparation charges the affected exact predicate before exact work;
  direction and edge-reference orientation use only their sign, and the three
  edge coefficients are scaled independently of that reference

### Requirement: Project exactly from original endpoints

Exact projection SHALL expand `det(end-start, point-start, direction)` from
original endpoints and ray direction before subtraction. Overflowing edges or
offsets SHALL select precharged fallback. The fast path SHALL retain rounded
floating-point edges and offsets. Non-finite original vertices or direction
SHALL return `ArithmeticOverflow`.

#### Scenario: Exact projection expands before subtraction

- **WHEN** exact projection evaluates original endpoints `s`, `e`, point `p`
  and ray direction `d`
- **THEN** it accumulates `det(e,p,d) - det(s,p,d) - det(e,s,d)` exactly,
  without first forming rounded differences; reference signs and commonly
  scaled coefficients use this expansion
- **AND** an overflowing intermediate edge, opposite offset or source offset
  bypasses the fast projected-edge frame, not the finite-input check

#### Scenario: Synthetic rounded subtraction erases displacement

- **WHEN** `projected_determinants_preserve_displacement_lost_by_subtraction`
  uses `s = X*2^-500`, `e = X + Y`, `p = 2*e`, and direction `Z`
- **THEN** `(e-s).cross(p-s)` rounds to zero, but exact projected sign is `-1`
  and coefficients for `[p, e, 0]` are `[-1, 0, 1]`
- **AND** this is a synthetic assertion, not execution evidence or independent
  physical validation

#### Scenario: Synthetic endpoint difference overflows

- **WHEN** `projected_determinants_accept_overflowing_endpoint_difference`
  uses `s = X*f64::MAX`, `e = -s`, direction `Z`, and points `[Y, -Y*0.5, 0]`
- **THEN** `e-s` is non-finite but exact coefficients succeed: the first is
  negative, the second is minus half the first, and the third is zero
- **AND** this is a synthetic assertion, not execution evidence or independent
  physical validation

### Requirement: Recover absolute-scale triangle areas

Windward and partition triangles SHALL use exact absolute-scale area fallback
when the 2D frame exceeds its span, precharging the regular surface-work budget.
Representable subnormal fractions SHALL be preserved; nonzero fractions rounding
to zero SHALL return `PrecisionLoss`.

#### Scenario: Wide triangle area preserves absolute scale and precharges work

- **WHEN** `wide_triangle_area_preserves_absolute_scale_and_precharges_work`
  evaluates `[0, X*m, X + Y]`, where `m` is the smallest positive binary64
  subnormal
- **THEN** the fraction is `m` in either winding and the charge is exactly
  `exact_orientation::WORK`
- **AND** replacing the last vertex with `X + Y*0.5` returns `PrecisionLoss`,
  while rejected precharge returns `WorkBudgetExceeded` before exact evaluation
- **AND** these are synthetic regression assertions, not test execution evidence
  or independent physical validation

### Requirement: Preserve representable numeric intersections

For finite endpoints with overflowing differences, crossings SHALL use
ratio-preserving rescaling and coordinates SHALL use weighted interpolation
when it preserves a representable intersection. Genuine non-finite results
and precision loss SHALL remain errors, not missed intersections.

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

Reconstructed pieces SHALL conserve source-local barycentric area. The check
SHALL use a documented heuristic tolerance, not a proven error bound, based
on current partition work including the initial triangle, not prior sources
or global preprocessing.

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

### Requirement: Reserve split outputs from measured vertex counts

For each clipping polygon of `n` vertices, preparation SHALL charge
`1 + 16*n` before evaluating distances or growing scratch. A split SHALL
reserve each side once using retained vertex and strict crossing counts.
Boundary vertices SHALL be retained on both sides. The unsplit polygon
SHALL reuse its vertex buffer. Reservation failure SHALL return
`AllocationFailed`.

#### Scenario: Split reservations include boundary vertices and the closing edge

- **WHEN** cyclic vertex distances are `[1, 0, -1]`
- **THEN** `p = 1`, `m = 1` and `c = 1`, with the crossing on the closing edge
- **AND** each output requests exactly three slots once before emission,
  retaining the zero-distance vertex and adding the crossing point

#### Scenario: A polygon does not require splitting

- **WHEN** all distances are nonnegative, all are nonpositive, or all are zero
- **THEN** the polygon is moved onward without allocating a new vertex buffer
- **AND** distance scratch may still be reserved and reused after the charge

### Requirement: Construct stable grid buckets with linear work

Grid construction SHALL preflight entry counts and use per-cell counts and
prefix offsets to populate the final index in input triangle order. Work
SHALL scale linearly with source triangles, cells and triangle-to-cell
entries. Construction SHALL budget `4*T + 3*C + 3*E + 1` units for those
counts.

#### Scenario: Synthetic grid preserves exact bucket membership and order

- **WHEN** `triangle_grid_scatter_preserves_bucket_membership_and_order`
  constructs its three synthetic triangles, with the middle triangle's AABB
  spanning all eight cells
- **THEN** grid dimensions are `[2, 2, 2]`, prefix offsets are
  `[0, 2, 3, 4, 5, 6, 7, 8, 10]`, and the final triangle index is
  `[0, 1, 1, 1, 1, 1, 1, 1, 1, 2]`
- **AND** construction succeeds with reported build work exactly
  `4*3 + 3*8 + 3*10 + 1 = 67` at budget 67, while budget 66 returns
  `WorkBudgetExceeded`
- **AND** these are synthetic layout and accounting assertions, not test
  execution evidence, experimental validation or exact allocator-byte bounds

#### Scenario: Patch replacement reflects revised grid accounting

- **WHEN** `unshadowed_patch_replacement_has_budget_boundary` checks its
  synthetic preparation budget boundary
- **THEN** its required work is 117 units, including one unit for partition
  replacement and one for frame conversion, rather than the prior 116

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
