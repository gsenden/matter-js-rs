use crate::geometry::{Vec2, Vertices, Bounds, Axes};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BodyHandle(pub usize);

#[derive(Debug)]
pub struct CollisionFilter {
    pub category: u32,
    pub mask: u32,
    pub group: i32,
}

#[derive(Debug)]
pub struct ConstraintImpulse {
    pub x: f64,
    pub y: f64,
    pub angle: f64,
}

#[derive(Debug)]
pub struct Body {
    pub id: usize,
    pub handle: BodyHandle,

    // Position & motion
    pub position: Vec2,
    pub position_prev: Vec2,
    pub velocity: Vec2,
    pub force: Vec2,
    pub torque: f64,
    pub position_impulse: Vec2,
    pub constraint_impulse: ConstraintImpulse,

    // Rotation
    pub angle: f64,
    pub angle_prev: f64,
    pub angular_velocity: f64,
    pub speed: f64,
    pub angular_speed: f64,

    // Mass
    pub mass: f64,
    pub inverse_mass: f64,
    pub inertia: f64,
    pub inverse_inertia: f64,
    pub density: f64,
    pub area: f64,

    // Flags
    pub is_static: bool,
    pub is_sensor: bool,
    pub is_sleeping: bool,

    // Sleep
    pub motion: f64,
    pub sleep_threshold: u32,
    pub total_contacts: usize,

    // Time
    pub time_scale: f64,
    pub delta_time: f64,

    // Material
    pub friction: f64,
    pub friction_static: f64,
    pub friction_air: f64,
    pub restitution: f64,
    pub slop: f64,

    // Collision
    pub collision_filter: CollisionFilter,

    // Geometry
    pub vertices: Vec<Vec2>,
    pub axes: Vec<Vec2>,
    pub bounds: Bounds,
    pub circle_radius: f64,

    // Hierarchy
    pub parts: Vec<BodyHandle>,
    pub parent: BodyHandle,
}

const BASE_DELTA: f64 = 1000.0 / 60.0;
const TIME_CORRECTION: bool = true;
const INERTIA_SCALE: f64 = 4.0;

impl Body {
    pub fn new(handle: BodyHandle) -> Body {
        let mut vertices = vec![
            Vec2 { x: 0.0, y: 0.0 },
            Vec2 { x: 40.0, y: 0.0 },
            Vec2 { x: 40.0, y: 40.0 },
            Vec2 { x: 0.0, y: 40.0 },
        ];
        let area = Vertices::area(&vertices, false);
        let density = 0.001;
        let mass = density * area;

        // Centre vertices around origin (like Matter.js Body.create)
        let centre = Vertices::centre(&vertices);
        Vertices::translate(&mut vertices, &centre, -1.0);

        // Calculate inertia with vertices at origin
        let inertia = INERTIA_SCALE * Vertices::inertia(&vertices, mass);

        // Translate vertices to body position (0,0 for default)
        let position = Vec2 { x: 0.0, y: 0.0 };
        Vertices::translate(&mut vertices, &position, 1.0);

        let axes = Axes::from_vertices(&vertices);
        let bounds = Bounds::create(&vertices);

        Body {
            id: handle.0,
            handle,

            position: Vec2 { x: 0.0, y: 0.0 },
            position_prev: Vec2 { x: 0.0, y: 0.0 },
            velocity: Vec2 { x: 0.0, y: 0.0 },
            force: Vec2 { x: 0.0, y: 0.0 },
            torque: 0.0,
            position_impulse: Vec2 { x: 0.0, y: 0.0 },
            constraint_impulse: ConstraintImpulse { x: 0.0, y: 0.0, angle: 0.0 },

            angle: 0.0,
            angle_prev: 0.0,
            angular_velocity: 0.0,
            speed: 0.0,
            angular_speed: 0.0,

            mass,
            inverse_mass: 1.0 / mass,
            inertia,
            inverse_inertia: 1.0 / inertia,
            density,
            area,

            is_static: false,
            is_sensor: false,
            is_sleeping: false,

            motion: 0.0,
            sleep_threshold: 60,
            total_contacts: 0,

            time_scale: 1.0,
            delta_time: 1000.0 / 60.0,

            friction: 0.1,
            friction_static: 0.5,
            friction_air: 0.01,
            restitution: 0.0,
            slop: 0.05,

            collision_filter: CollisionFilter {
                category: 0x0001,
                mask: 0xFFFFFFFF,
                group: 0,
            },

            vertices,
            axes,
            bounds,
            circle_radius: 0.0,

            parts: vec![handle],
            parent: handle,
        }
    }

    pub fn set_velocity(&mut self, velocity: Vec2) {
        let time_scale = self.delta_time / BASE_DELTA;
        self.position_prev.x = self.position.x - velocity.x * time_scale;
        self.position_prev.y = self.position.y - velocity.y * time_scale;
        self.velocity.x = (self.position.x - self.position_prev.x) / time_scale;
        self.velocity.y = (self.position.y - self.position_prev.y) / time_scale;
        self.speed = Vec2::magnitude(&self.velocity);
    }

    pub fn update(&mut self, delta_time: f64) {
        let delta_time = delta_time * self.time_scale;
        let delta_time_squared = delta_time * delta_time;
        let correction = if TIME_CORRECTION {
            if self.delta_time != 0.0 { delta_time / self.delta_time } else { 1.0 }
        } else {
            1.0
        };

        let friction_air = 1.0 - self.friction_air * (delta_time / BASE_DELTA);
        let velocity_prev_x = (self.position.x - self.position_prev.x) * correction;
        let velocity_prev_y = (self.position.y - self.position_prev.y) * correction;

        // Update velocity with Verlet integration
        self.velocity.x = (velocity_prev_x * friction_air) + (self.force.x / self.mass) * delta_time_squared;
        self.velocity.y = (velocity_prev_y * friction_air) + (self.force.y / self.mass) * delta_time_squared;

        self.position_prev.x = self.position.x;
        self.position_prev.y = self.position.y;
        self.position.x += self.velocity.x;
        self.position.y += self.velocity.y;
        self.delta_time = delta_time;

        // Update angular velocity with Verlet integration
        self.angular_velocity = ((self.angle - self.angle_prev) * friction_air * correction)
            + (self.torque / self.inertia) * delta_time_squared;
        self.angle_prev = self.angle;
        self.angle += self.angular_velocity;

        // Transform body geometry
        Vertices::translate(&mut self.vertices, &self.velocity, 1.0);

        if self.angular_velocity != 0.0 {
            Vertices::rotate(&mut self.vertices, self.angular_velocity, &self.position);
            Axes::rotate(&mut self.axes, self.angular_velocity);
        }

        Bounds::update(&mut self.bounds, &self.vertices, &self.velocity);
    }

    pub fn update_velocities(&mut self) {
        let time_scale = BASE_DELTA / self.delta_time;

        self.velocity.x = (self.position.x - self.position_prev.x) * time_scale;
        self.velocity.y = (self.position.y - self.position_prev.y) * time_scale;
        self.speed = Vec2::magnitude(&self.velocity);

        self.angular_velocity = (self.angle - self.angle_prev) * time_scale;
        self.angular_speed = self.angular_velocity.abs();
    }

    pub fn apply_force(&mut self, position: &Vec2, force: &Vec2) {
        let offset_x = position.x - self.position.x;
        let offset_y = position.y - self.position.y;
        self.force.x += force.x;
        self.force.y += force.y;
        self.torque += offset_x * force.y - offset_y * force.x;
    }

    pub fn set_static(&mut self, is_static: bool) {
        if is_static {
            self.restitution = 0.0;
            self.friction = 1.0;
            self.mass = f64::INFINITY;
            self.inertia = f64::INFINITY;
            self.density = f64::INFINITY;
            self.inverse_mass = 0.0;
            self.inverse_inertia = 0.0;

            self.position_prev.x = self.position.x;
            self.position_prev.y = self.position.y;
            self.angle_prev = self.angle;
            self.angular_velocity = 0.0;
            self.speed = 0.0;
            self.angular_speed = 0.0;
            self.motion = 0.0;
        }
        self.is_static = is_static;
    }

    pub fn set_position(&mut self, position: Vec2, update_velocity: bool) {
        let delta = Vec2 {
            x: position.x - self.position.x,
            y: position.y - self.position.y,
        };

        if update_velocity {
            self.position_prev.x = self.position.x;
            self.position_prev.y = self.position.y;
            self.velocity.x = delta.x;
            self.velocity.y = delta.y;
            self.speed = Vec2::magnitude(&delta);
        } else {
            self.position_prev.x += delta.x;
            self.position_prev.y += delta.y;
        }

        self.position.x += delta.x;
        self.position.y += delta.y;
        Vertices::translate(&mut self.vertices, &delta, 1.0);
        Bounds::update(&mut self.bounds, &self.vertices, &self.velocity);
    }

    pub fn set_angle(&mut self, angle: f64, update_velocity: bool) {
        let delta = angle - self.angle;

        if update_velocity {
            self.angle_prev = self.angle;
            self.angular_velocity = delta;
            self.angular_speed = delta.abs();
        } else {
            self.angle_prev += delta;
        }

        self.angle += delta;
        Vertices::rotate(&mut self.vertices, delta, &self.position);
        Axes::rotate(&mut self.axes, delta);
        Bounds::update(&mut self.bounds, &self.vertices, &self.velocity);
    }

    pub fn translate(&mut self, translation: Vec2, update_velocity: bool) {
        let new_pos = Vec2 {
            x: self.position.x + translation.x,
            y: self.position.y + translation.y,
        };
        self.set_position(new_pos, update_velocity);
    }

    pub fn rotate(&mut self, rotation: f64, point: Option<Vec2>, update_velocity: bool) {
        match point {
            None => {
                self.set_angle(self.angle + rotation, update_velocity);
            }
            Some(point) => {
                let cos = rotation.cos();
                let sin = rotation.sin();
                let dx = self.position.x - point.x;
                let dy = self.position.y - point.y;

                self.set_position(
                    Vec2 {
                        x: point.x + (dx * cos - dy * sin),
                        y: point.y + (dx * sin + dy * cos),
                    },
                    update_velocity,
                );
                self.set_angle(self.angle + rotation, update_velocity);
            }
        }
    }

    pub fn set_mass(&mut self, mass: f64) {
        let moment = self.inertia / (self.mass / 6.0);
        self.inertia = moment * (mass / 6.0);
        self.inverse_inertia = 1.0 / self.inertia;
        self.mass = mass;
        self.inverse_mass = 1.0 / self.mass;
        self.density = self.mass / self.area;
    }

    pub fn set_inertia(&mut self, inertia: f64) {
        self.inertia = inertia;
        self.inverse_inertia = 1.0 / self.inertia;
    }

    pub fn scale(&mut self, scale_x: f64, scale_y: f64, point: Option<Vec2>) {
        let point = point.unwrap_or(self.position);

        Vertices::scale(&mut self.vertices, scale_x, scale_y, &point);
        self.axes = Axes::from_vertices(&self.vertices);
        self.area = Vertices::area(&self.vertices, false);
        self.set_mass(self.density * self.area);

        // Update inertia (requires vertices at origin)
        Vertices::translate(&mut self.vertices, &self.position, -1.0);
        self.set_inertia(INERTIA_SCALE * Vertices::inertia(&self.vertices, self.mass));
        Vertices::translate(&mut self.vertices, &self.position, 1.0);

        // Scale position
        self.position.x = point.x + (self.position.x - point.x) * scale_x;
        self.position.y = point.y + (self.position.y - point.y) * scale_y;

        Bounds::update(&mut self.bounds, &self.vertices, &self.velocity);

        // Handle circles
        if self.circle_radius != 0.0 {
            if scale_x == scale_y {
                self.circle_radius *= scale_x;
            } else {
                self.circle_radius = 0.0;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    const EPSILON: f64 = 1e-14;

    fn load_testdata() -> Value {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../testdata/body.json");
        let content = std::fs::read_to_string(path).expect("Failed to read body.json");
        serde_json::from_str(&content).expect("Failed to parse body.json")
    }

    fn assert_f64_eq(actual: f64, expected: f64) {
        let diff = (actual - expected).abs();
        let magnitude = actual.abs().max(expected.abs());
        let tolerance = if magnitude > 1.0 { EPSILON * magnitude } else { EPSILON };
        assert!(
            diff < tolerance,
            "f64 differs: actual {actual} expected {expected} (diff {diff})"
        );
    }

    fn assert_vec2_eq(actual: &crate::geometry::Vec2, expected: &Value) {
        assert_f64_eq(actual.x, expected["x"].as_f64().unwrap());
        assert_f64_eq(actual.y, expected["y"].as_f64().unwrap());
    }

    #[test]
    fn compound_body_parent_parts_relationship() {
        let data = load_testdata();
        let expected = &data["body_compound"];

        // Create parts
        let mut bodies: Vec<Body> = vec![];
        let part_a = Body::new(BodyHandle(0));
        let part_b = Body::new(BodyHandle(1));
        let parent_handle = BodyHandle(2);
        bodies.push(part_a);
        bodies.push(part_b);

        // Create parent compound body
        let mut parent = Body::new(parent_handle);
        parent.parts = vec![parent_handle, BodyHandle(0), BodyHandle(1)];
        bodies[0].parent = parent_handle;
        bodies[1].parent = parent_handle;
        bodies.push(parent);

        // Verify structure matches Matter.js
        let parent_body = &bodies[2];
        assert_eq!(parent_body.parts.len(), expected["parts"].as_array().unwrap().len());
        assert_eq!(parent_body.parts[0], parent_body.handle);

        // Each part points to parent
        assert_eq!(bodies[0].parent, parent_handle);
        assert_eq!(bodies[1].parent, parent_handle);
    }

    #[test]
    fn body_defaults_match_matterjs() {
        let data = load_testdata();
        let expected = &data["body_defaults"];
        let body = Body::new(BodyHandle(0));

        assert_vec2_eq(&body.position, &expected["position"]);
        assert_vec2_eq(&body.velocity, &expected["velocity"]);
        assert_vec2_eq(&body.force, &expected["force"]);
        assert_f64_eq(body.torque, expected["torque"].as_f64().unwrap());
        assert_f64_eq(body.angle, expected["angle"].as_f64().unwrap());
        assert_f64_eq(body.angular_velocity, expected["angularVelocity"].as_f64().unwrap());
        assert_f64_eq(body.mass, expected["mass"].as_f64().unwrap());
        assert_f64_eq(body.inverse_mass, expected["inverseMass"].as_f64().unwrap());
        assert_f64_eq(body.inertia, expected["inertia"].as_f64().unwrap());
        assert_f64_eq(body.inverse_inertia, expected["inverseInertia"].as_f64().unwrap());
        assert_f64_eq(body.density, expected["density"].as_f64().unwrap());
        assert_f64_eq(body.area, expected["area"].as_f64().unwrap());
        assert_eq!(body.is_static, expected["isStatic"].as_bool().unwrap());
        assert_eq!(body.is_sensor, expected["isSensor"].as_bool().unwrap());
        assert_eq!(body.is_sleeping, expected["isSleeping"].as_bool().unwrap());
        assert_f64_eq(body.friction, expected["friction"].as_f64().unwrap());
        assert_f64_eq(body.friction_static, expected["frictionStatic"].as_f64().unwrap());
        assert_f64_eq(body.friction_air, expected["frictionAir"].as_f64().unwrap());
        assert_f64_eq(body.restitution, expected["restitution"].as_f64().unwrap());
        assert_f64_eq(body.slop, expected["slop"].as_f64().unwrap());
        assert_f64_eq(body.time_scale, expected["timeScale"].as_f64().unwrap());
        assert_f64_eq(body.delta_time, expected["deltaTime"].as_f64().unwrap());
        assert_eq!(body.collision_filter.category, expected["collisionFilter"]["category"].as_u64().unwrap() as u32);
        assert_eq!(body.collision_filter.mask, expected["collisionFilter"]["mask"].as_u64().unwrap() as u32);
        assert_eq!(body.collision_filter.group, expected["collisionFilter"]["group"].as_i64().unwrap() as i32);

        // Vertices
        let exp_verts = expected["vertices"].as_array().unwrap();
        assert_eq!(body.vertices.len(), exp_verts.len());
        for (v, ev) in body.vertices.iter().zip(exp_verts.iter()) {
            assert_vec2_eq(v, ev);
        }
    }

    #[test]
    fn body_parts_contains_self() {
        let body = Body::new(BodyHandle(0));
        assert_eq!(body.parts[0], body.handle);
    }

    #[test]
    fn body_parent_is_self() {
        let body = Body::new(BodyHandle(0));
        assert_eq!(body.parent, body.handle);
    }

    #[test]
    fn body_update_single_tick_gravity() {
        let data = load_testdata();
        let expected = &data["body_update_gravity"]["after"];

        let mut body = Body::new(BodyHandle(0));
        // Apply gravity: force = mass * gravity
        body.force.y += body.mass * 1.0;

        body.update(1000.0 / 60.0);
        body.update_velocities();

        assert_vec2_eq(&body.position, &expected["position"]);
        assert_vec2_eq(&body.position_prev, &expected["positionPrev"]);
        assert_vec2_eq(&body.velocity, &expected["velocity"]);
        assert_f64_eq(body.speed, expected["speed"].as_f64().unwrap());
        assert_f64_eq(body.angle, expected["angle"].as_f64().unwrap());
        assert_f64_eq(body.angular_velocity, expected["angularVelocity"].as_f64().unwrap());
        assert_f64_eq(body.delta_time, expected["deltaTime"].as_f64().unwrap());
    }

    #[test]
    fn body_update_with_initial_velocity() {
        let data = load_testdata();
        let expected = &data["body_update_moving"]["after"];

        let mut body = Body::new(BodyHandle(0));
        // Set velocity by offsetting positionPrev (like Body.setVelocity)
        body.set_velocity(Vec2 { x: 2.0, y: -3.0 });
        // Apply gravity
        body.force.y += body.mass * 1.0;

        body.update(1000.0 / 60.0);
        body.update_velocities();

        assert_vec2_eq(&body.position, &expected["position"]);
        assert_vec2_eq(&body.velocity, &expected["velocity"]);
        assert_f64_eq(body.speed, expected["speed"].as_f64().unwrap());
    }

    #[test]
    fn body_update_3_ticks_gravity() {
        let data = load_testdata();
        let ticks = data["body_update_3ticks"]["ticks"].as_array().unwrap();

        let mut body = Body::new(BodyHandle(0));

        for expected in ticks.iter() {
            // Reset force each tick (like Engine does)
            body.force.x = body.mass * 0.0;
            body.force.y = body.mass * 1.0;

            body.update(1000.0 / 60.0);
            body.update_velocities();

            assert_vec2_eq(&body.position, &expected["position"]);
            assert_vec2_eq(&body.velocity, &expected["velocity"]);
            assert_f64_eq(body.speed, expected["speed"].as_f64().unwrap());
        }
    }

    fn assert_verts_eq(actual: &[Vec2], expected: &Value) {
        let exp = expected.as_array().unwrap();
        assert_eq!(actual.len(), exp.len());
        for (v, ev) in actual.iter().zip(exp.iter()) {
            assert_vec2_eq(v, ev);
        }
    }

    #[test]
    fn body_apply_force() {
        let data = load_testdata();
        let expected = &data["body_apply_force"];

        let mut body = Body::new(BodyHandle(0));
        body.apply_force(&Vec2 { x: 20.0, y: 20.0 }, &Vec2 { x: 0.05, y: -0.1 });

        assert_f64_eq(body.force.x, expected["force"]["x"].as_f64().unwrap());
        assert_f64_eq(body.force.y, expected["force"]["y"].as_f64().unwrap());
        assert_f64_eq(body.torque, expected["torque"].as_f64().unwrap());
    }

    #[test]
    fn body_set_static() {
        let data = load_testdata();
        let expected = &data["body_set_static"]["after"];

        let mut body = Body::new(BodyHandle(0));
        body.set_static(true);

        assert!(body.is_static);
        assert_eq!(body.mass, f64::INFINITY);
        assert_eq!(body.inertia, f64::INFINITY);
        assert_eq!(body.density, f64::INFINITY);
        assert_f64_eq(body.inverse_mass, 0.0);
        assert_f64_eq(body.inverse_inertia, 0.0);
        assert_f64_eq(body.friction, expected["friction"].as_f64().unwrap());
        assert_f64_eq(body.restitution, expected["restitution"].as_f64().unwrap());
    }

    #[test]
    fn body_set_position() {
        let data = load_testdata();
        let expected = &data["body_set_position"];

        let mut body = Body::new(BodyHandle(0));
        body.set_position(Vec2 { x: 100.0, y: 200.0 }, false);

        assert_vec2_eq(&body.position, &expected["position"]);
        assert_vec2_eq(&body.position_prev, &expected["positionPrev"]);
        assert_verts_eq(&body.vertices, &expected["vertices"]);
    }

    #[test]
    fn body_set_position_update_velocity() {
        let data = load_testdata();
        let expected = &data["body_set_position_update_velocity"];

        let mut body = Body::new(BodyHandle(0));
        body.set_position(Vec2 { x: 100.0, y: 200.0 }, true);

        assert_vec2_eq(&body.position, &expected["position"]);
        assert_vec2_eq(&body.position_prev, &expected["positionPrev"]);
        assert_vec2_eq(&body.velocity, &expected["velocity"]);
        assert_f64_eq(body.speed, expected["speed"].as_f64().unwrap());
    }

    #[test]
    fn body_set_angle() {
        let data = load_testdata();
        let expected = &data["body_set_angle"];

        let mut body = Body::new(BodyHandle(0));
        body.set_angle(std::f64::consts::FRAC_PI_4, false);

        assert_f64_eq(body.angle, expected["angle"].as_f64().unwrap());
        assert_f64_eq(body.angle_prev, expected["anglePrev"].as_f64().unwrap());
        assert_verts_eq(&body.vertices, &expected["vertices"]);
    }

    #[test]
    fn body_set_angle_update_velocity() {
        let data = load_testdata();
        let expected = &data["body_set_angle_update_velocity"];

        let mut body = Body::new(BodyHandle(0));
        body.set_angle(std::f64::consts::FRAC_PI_4, true);

        assert_f64_eq(body.angle, expected["angle"].as_f64().unwrap());
        assert_f64_eq(body.angle_prev, expected["anglePrev"].as_f64().unwrap());
        assert_f64_eq(body.angular_velocity, expected["angularVelocity"].as_f64().unwrap());
        assert_f64_eq(body.angular_speed, expected["angularSpeed"].as_f64().unwrap());
    }

    #[test]
    fn body_set_velocity_test() {
        let data = load_testdata();
        let expected = &data["body_set_velocity"];

        let mut body = Body::new(BodyHandle(0));
        body.set_velocity(Vec2 { x: 5.0, y: -3.0 });

        assert_vec2_eq(&body.velocity, &expected["velocity"]);
        assert_vec2_eq(&body.position_prev, &expected["positionPrev"]);
        assert_f64_eq(body.speed, expected["speed"].as_f64().unwrap());
    }

    #[test]
    fn body_translate() {
        let data = load_testdata();
        let expected = &data["body_translate"];

        let mut body = Body::new(BodyHandle(0));
        body.translate(Vec2 { x: 30.0, y: -15.0 }, false);

        assert_vec2_eq(&body.position, &expected["position"]);
        assert_verts_eq(&body.vertices, &expected["vertices"]);
    }

    #[test]
    fn body_rotate() {
        let data = load_testdata();
        let expected = &data["body_rotate"];

        let mut body = Body::new(BodyHandle(0));
        body.rotate(std::f64::consts::FRAC_PI_6, None, false);

        assert_f64_eq(body.angle, expected["angle"].as_f64().unwrap());
        assert_verts_eq(&body.vertices, &expected["vertices"]);
    }

    #[test]
    fn body_rotate_around_point() {
        let data = load_testdata();
        let expected = &data["body_rotate_point"];

        let mut body = Body::new(BodyHandle(0));
        body.rotate(std::f64::consts::FRAC_PI_3, Some(Vec2 { x: 50.0, y: 50.0 }), false);

        assert_vec2_eq(&body.position, &expected["position"]);
        assert_f64_eq(body.angle, expected["angle"].as_f64().unwrap());
        assert_verts_eq(&body.vertices, &expected["vertices"]);
    }

    #[test]
    fn body_scale() {
        let data = load_testdata();
        let expected = &data["body_scale"];

        let mut body = Body::new(BodyHandle(0));
        body.scale(2.0, 1.5, None);

        assert_f64_eq(body.area, expected["area"].as_f64().unwrap());
        assert_f64_eq(body.mass, expected["mass"].as_f64().unwrap());
        assert_f64_eq(body.inertia, expected["inertia"].as_f64().unwrap());
        assert_verts_eq(&body.vertices, &expected["vertices"]);
    }
}
