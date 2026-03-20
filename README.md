# matter-rs

A Rust port of [Matter.js](https://github.com/liabru/matter-js), the 2D rigid body physics engine.

## Why?

Matter.js is embedded in [Phaser](https://phaser.io/) as its default physics engine. By porting it to Rust, we get:

- **Authoritative game server** — run the same physics on the server as the client, preventing desync
- **WASM build** — drop-in replacement for Matter.js in the browser with better performance
- **Bit-exact compatibility** — verified against Matter.js output (IEEE 754 f64 precision)

## Demo

[Live demo](https://gsenden.codeberg.page/matter-rs/) — interactive physics scenes running in the browser via WASM.

## Project structure

```
matter-rs/
├── src/            # Physics engine library
├── wasm/           # WASM binding (thin wrapper)
└── testdata/       # Reference data generated from Matter.js
```

## Verification

Every function is tested against Matter.js output:

1. `testdata/generate.js` runs scenarios in Matter.js and stores input + output as JSON
2. Rust tests read the JSON and compare with floating-point precision (relative epsilon 1e-14)
3. `node testdata/generate.js` regenerates all reference data

## Progress

- [x] **Geometry** — Vec2, Vertices, Bounds, Axes
- [x] **Body** — Body struct, Verlet integration, set_position/angle/velocity, apply_force, scale
- [x] **Composite** — Body/constraint container with recursive traversal
- [x] **Collision** — SAT detection, contacts, pairs, broadphase detector
- [x] **Constraint** — Distance/spring constraints, Gauss-Siedel solver
- [x] **Engine** — Simulation loop, resolver, collision events
- [x] **Factory** — rectangle, circle, polygon, trapezoid builders
- [x] **WASM** — wasm-bindgen bindings (103KB, TypeScript types)

See also: [matter-rs-server](https://codeberg.org/gsenden/matter-rs-server) — multiplayer game server using this engine.

## Building

```sh
cargo build --workspace
cargo test --workspace
```

## Regenerating test data

```sh
cd testdata
npm install
node generate.js
```

## Acknowledgements

This project is a port of [Matter.js](https://github.com/liabru/matter-js) by Liam Brummitt, licensed under the MIT License.

## License

Licensed under either of

- [Apache License, Version 2.0](LICENSE-APACHE)
- [MIT License](LICENSE-MIT)

at your option.
