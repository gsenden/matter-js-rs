# matter-rs

A Rust port of [Matter.js](https://github.com/liabru/matter-js), the 2D rigid body physics engine.

## Why?

Matter.js is embedded in [Phaser](https://phaser.io/) as its default physics engine. By porting it to Rust, we get:

- **Authoritative game server** — run the same physics on the server as the client, preventing desync
- **WASM build** — drop-in replacement for Matter.js in the browser with better performance
- **Bit-exact compatibility** — verified against Matter.js output (IEEE 754 f64 precision)

## Project structure

```
matter-rs/
├── crates/
│   ├── core/       # Physics engine — pure domain, no I/O
│   ├── server/     # Authoritative game server (hexagonal architecture)
│   └── wasm/       # WASM binding (thin wrapper)
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
- [ ] **Composite** — Body/constraint container
- [ ] **Collision** — SAT detection, contacts, pairs, detector
- [ ] **Constraint** — Distance/spring constraints
- [ ] **Engine** — Simulation loop, solver, events
- [ ] **Factory** — rectangle, circle, polygon builders
- [ ] **WASM** — wasm-bindgen bindings
- [ ] **Server** — Authoritative game server with WebSocket

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

## License

[EUPL-1.2](LICENSE)
