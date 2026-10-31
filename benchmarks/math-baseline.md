# kapitsa-math baseline

Recorded 2026-10-31 on Windows 11 Pro with an AMD Ryzen 9 7900X3D, using
`rustc 1.98.0` and `cargo 1.98.0`.

Run with:

```powershell
cargo bench -p kapitsa-math --bench math_baseline
```

Each deterministic kernel ran for 1,000,000 iterations per sample. The table
shows the median of seven samples and the minimum-to-maximum range. Results are
in nanoseconds per kernel iteration; they are a local comparison baseline, not
a cross-machine performance target.

| Kernel | Median (ns/iteration) | Range (ns/iteration) |
| --- | ---: | ---: |
| Vec3 arithmetic | 3.81 | 3.74–3.86 |
| Vec3 normalization | 29.39 | 29.32–29.57 |
| Mat3 multiply | 11.14 | 11.10–11.32 |
| Mat3 SPD solve | 57.71 | 57.15–59.37 |
| Quat vector rotation | 29.22 | 29.12–29.26 |
| Quat angular integration | 75.55 | 75.09–75.86 |
