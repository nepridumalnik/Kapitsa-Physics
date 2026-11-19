# kapitsa-math

`kapitsa-math` contains the `f64` mathematical primitives used by the Kapitsa physics
engine: 2D and 3D vectors, a 3×3 matrix with a symmetric Cholesky decomposition, an
affine transform, and unit quaternions. The crate is `unsafe`-free and has no
dependencies.

## Role in the workspace

- `kapitsa-math` — this crate; shared math types in SI-friendly `f64`.
- `kapitsa-core` — synchronous physics built on these types.
- `kapitsa` — asynchronous simulation runtime and the public engine API.

## Documentation

- [Repository README](https://github.com/nepridumalnik/Kapitsa-Physics#readme)
- [Collision system](https://github.com/nepridumalnik/Kapitsa-Physics/blob/main/docs/collisions.md)
- [API documentation](https://docs.rs/kapitsa-math)

## License

MIT. See [LICENSE.md](LICENSE.md).