use wasm_bindgen::prelude::*;
use matter_core::body::BodyHandle;
use matter_core::constraint::{Constraint, ConstraintHandle, ConstraintOptions};
use matter_core::engine::{Engine, Gravity, PhysicsEvent};
use matter_core::factory::Bodies;
use matter_core::geometry::Vec2;
use serde::Serialize;

#[wasm_bindgen]
pub struct PhysicsEngine {
    engine: Engine,
    next_body_id: usize,
    next_constraint_id: usize,
}

#[derive(Serialize)]
struct BodyState {
    id: usize,
    x: f64,
    y: f64,
    angle: f64,
    vx: f64,
    vy: f64,
    speed: f64,
    #[serde(rename = "angularVelocity")]
    angular_velocity: f64,
}

#[derive(Serialize)]
struct UpdateResult {
    bodies: Vec<BodyState>,
    events: Vec<EventData>,
}

#[derive(Serialize)]
struct EventData {
    #[serde(rename = "type")]
    event_type: String,
    pairs: Vec<[usize; 2]>,
}

impl Default for PhysicsEngine {
    fn default() -> PhysicsEngine {
        PhysicsEngine::new()
    }
}

#[wasm_bindgen]
impl PhysicsEngine {
    #[wasm_bindgen(constructor)]
    pub fn new() -> PhysicsEngine {
        PhysicsEngine {
            engine: Engine::new(),
            next_body_id: 0,
            next_constraint_id: 0,
        }
    }

    #[wasm_bindgen(js_name = setGravity)]
    pub fn set_gravity(&mut self, x: f64, y: f64, scale: f64) {
        self.engine.gravity = Gravity { x, y, scale };
    }

    #[wasm_bindgen(js_name = addRectangle)]
    pub fn add_rectangle(&mut self, x: f64, y: f64, width: f64, height: f64, is_static: bool) -> usize {
        let id = self.next_body_id;
        self.next_body_id += 1;
        let mut body = Bodies::rectangle(BodyHandle(id), x, y, width, height);
        if is_static {
            body.set_static(true);
        }
        self.engine.add_body(body);
        id
    }

    #[wasm_bindgen(js_name = addCircle)]
    pub fn add_circle(&mut self, x: f64, y: f64, radius: f64, is_static: bool) -> usize {
        let id = self.next_body_id;
        self.next_body_id += 1;
        let mut body = Bodies::circle(BodyHandle(id), x, y, radius);
        if is_static {
            body.set_static(true);
        }
        self.engine.add_body(body);
        id
    }

    #[wasm_bindgen(js_name = addPolygon)]
    pub fn add_polygon(&mut self, x: f64, y: f64, sides: usize, radius: f64, is_static: bool) -> usize {
        let id = self.next_body_id;
        self.next_body_id += 1;
        let mut body = Bodies::polygon(BodyHandle(id), x, y, sides, radius);
        if is_static {
            body.set_static(true);
        }
        self.engine.add_body(body);
        id
    }

    #[wasm_bindgen(js_name = addConstraint)]
    pub fn add_constraint(&mut self, body_a: usize, body_b: usize, stiffness: f64) -> usize {
        let id = self.next_constraint_id;
        self.next_constraint_id += 1;
        let constraint = Constraint::new(
            ConstraintHandle(id),
            ConstraintOptions {
                body_a: Some(BodyHandle(body_a)),
                body_b: Some(BodyHandle(body_b)),
                point_a: None,
                point_b: None,
                length: None,
                stiffness: Some(stiffness),
            },
            &self.engine.bodies,
        );
        self.engine.add_constraint(constraint);
        id
    }

    #[wasm_bindgen(js_name = addPinConstraint)]
    pub fn add_pin_constraint(&mut self, body_a: usize, world_x: f64, world_y: f64, stiffness: f64) -> usize {
        let id = self.next_constraint_id;
        self.next_constraint_id += 1;
        let constraint = Constraint::new(
            ConstraintHandle(id),
            ConstraintOptions {
                body_a: Some(BodyHandle(body_a)),
                body_b: None,
                point_a: None,
                point_b: Some(Vec2 { x: world_x, y: world_y }),
                length: None,
                stiffness: Some(stiffness),
            },
            &self.engine.bodies,
        );
        self.engine.add_constraint(constraint);
        id
    }

    #[wasm_bindgen(js_name = setVelocity)]
    pub fn set_velocity(&mut self, body_id: usize, vx: f64, vy: f64) {
        if let Some(body) = self.engine.bodies.get_mut(body_id) {
            body.set_velocity(Vec2 { x: vx, y: vy });
        }
    }

    #[wasm_bindgen(js_name = setPosition)]
    pub fn set_position(&mut self, body_id: usize, x: f64, y: f64) {
        if let Some(body) = self.engine.bodies.get_mut(body_id) {
            body.set_position(Vec2 { x, y }, false);
        }
    }

    #[wasm_bindgen(js_name = applyForce)]
    pub fn apply_force(&mut self, body_id: usize, px: f64, py: f64, fx: f64, fy: f64) {
        if let Some(body) = self.engine.bodies.get_mut(body_id) {
            body.apply_force(&Vec2 { x: px, y: py }, &Vec2 { x: fx, y: fy });
        }
    }

    pub fn update(&mut self, delta: f64) -> JsValue {
        let events = self.engine.update(delta);

        let body_states: Vec<BodyState> = self.engine.bodies.iter().map(|b| BodyState {
            id: b.id,
            x: b.position.x,
            y: b.position.y,
            angle: b.angle,
            vx: b.velocity.x,
            vy: b.velocity.y,
            speed: b.speed,
            angular_velocity: b.angular_velocity,
        }).collect();

        let event_data: Vec<EventData> = events.into_iter().map(|e| match e {
            PhysicsEvent::CollisionStart { pairs } => EventData {
                event_type: "collisionStart".to_string(),
                pairs: pairs.into_iter().map(|(a, b)| [a, b]).collect(),
            },
            PhysicsEvent::CollisionActive { pairs } => EventData {
                event_type: "collisionActive".to_string(),
                pairs: pairs.into_iter().map(|(a, b)| [a, b]).collect(),
            },
            PhysicsEvent::CollisionEnd { pairs } => EventData {
                event_type: "collisionEnd".to_string(),
                pairs: pairs.into_iter().map(|(a, b)| [a, b]).collect(),
            },
        }).collect();

        let result = UpdateResult {
            bodies: body_states,
            events: event_data,
        };

        serde_wasm_bindgen::to_value(&result).unwrap_or(JsValue::NULL)
    }

    #[wasm_bindgen(js_name = getBodyCount)]
    pub fn get_body_count(&self) -> usize {
        self.engine.bodies.len()
    }
}
