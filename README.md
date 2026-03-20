# matter-rs

A Rust port of [Matter.js](https://github.com/liabru/matter-js), the 2D rigid body physics engine.

## Why?

- **Pure Rust** — use Matter.js physics in any Rust project, no JavaScript runtime needed
- **WASM build** — drop-in replacement for Matter.js in the browser
- **Bit-exact compatibility** — verified against Matter.js output (IEEE 754 f64 precision)

## Compatibility

| matter-rs | Matter.js |
|-----------|-----------|
| 0.1.x     | 0.20.x    |

## Demo

[Live demo](https://gsenden.codeberg.page/matter-rs/) — interactive physics scenes running in the browser via WASM.

## Features

- **Geometry** — Vec2, Vertices, Bounds, Axes
- **Body** — Verlet integration, forces, collisions, static bodies
- **Composite** — Body/constraint container with recursive traversal
- **Collision** — SAT detection, contacts, pairs, broadphase (AABB sweep)
- **Constraint** — Distance/spring constraints, Gauss-Siedel solver
- **Engine** — Full simulation loop, resolver, collision events
- **Factory** — Rectangle, circle, polygon, trapezoid builders
- **WASM** — wasm-bindgen bindings with TypeScript types

## Project structure

```
matter-rs/
├── src/            # Physics engine library
├── wasm/           # WASM binding (thin wrapper)
└── testdata/       # Reference data generated from Matter.js
```

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

## Related projects

- [matter-rs-server](https://codeberg.org/gsenden/matter-rs-server) — multiplayer game server with Dioxus frontend, using this engine

## Acknowledgements

This project is a port of [Matter.js](https://github.com/liabru/matter-js) by Liam Brummitt, licensed under the MIT License.

## License

Licensed under either of

- [Apache License, Version 2.0](LICENSE-APACHE)
- [MIT License](LICENSE-MIT)

at your option.
