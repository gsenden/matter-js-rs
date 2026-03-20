use crate::body::Body;
use crate::collision::{CollisionResult, Detector, Pairs};
use crate::constraint::Constraint;
use crate::geometry::{Vertices, Bounds};

const BASE_DELTA: f64 = 1000.0 / 60.0;

// --- Gravity ---

#[derive(Debug, Clone)]
pub struct Gravity {
    pub x: f64,
    pub y: f64,
    pub scale: f64,
}

impl Default for Gravity {
    fn default() -> Gravity {
        Gravity {
            x: 0.0,
            y: 1.0,
            scale: 0.001,
        }
    }
}

// --- Timing ---

#[derive(Debug, Clone)]
pub struct Timing {
    pub timestamp: f64,
    pub time_scale: f64,
    pub last_delta: f64,
}

impl Default for Timing {
    fn default() -> Timing {
        Timing {
            timestamp: 0.0,
            time_scale: 1.0,
            last_delta: 0.0,
        }
    }
}

// --- Events ---

#[derive(Debug, Clone)]
pub enum PhysicsEvent {
    CollisionStart { pairs: Vec<(usize, usize)> },
    CollisionActive { pairs: Vec<(usize, usize)> },
    CollisionEnd { pairs: Vec<(usize, usize)> },
}

// --- Resolver ---

pub struct Resolver;

const RESTING_THRESH: f64 = 2.0;
const RESTING_THRESH_TANGENT: f64 = 2.449489742783178; // sqrt(6)
const POSITION_DAMPEN: f64 = 0.9;
const POSITION_WARMING: f64 = 0.8;
const FRICTION_NORMAL_MULTIPLIER: f64 = 5.0;
const FRICTION_MAX_STATIC: f64 = f64::MAX;

impl Resolver {
    pub fn pre_solve_position(pairs: &Pairs, bodies: &mut [Body]) {
        for pair in pairs.table.values() {
            if !pair.is_active {
                continue;
            }
            let count = pair.contact_count;
            bodies[pair.collision.parent_a.0].total_contacts += count;
            bodies[pair.collision.parent_b.0].total_contacts += count;
        }
    }

    pub fn solve_position(pairs: &mut Pairs, bodies: &mut [Body], delta: f64, damping: f64) {
        let position_dampen = POSITION_DAMPEN * damping;
        let slop_dampen = (delta / BASE_DELTA).clamp(0.0, 1.0);

        // Update separation
        for pair in pairs.table.values_mut() {
            if !pair.is_active || pair.is_sensor {
                continue;
            }
            let collision = &pair.collision;
            let normal = collision.normal;
            let body_a_impulse = bodies[collision.parent_a.0].position_impulse;
            let body_b_impulse = bodies[collision.parent_b.0].position_impulse;
            pair.separation = collision.depth
                + normal.x * (body_b_impulse.x - body_a_impulse.x)
                + normal.y * (body_b_impulse.y - body_a_impulse.y);
        }

        // Apply position impulses
        for pair in pairs.table.values() {
            if !pair.is_active || pair.is_sensor {
                continue;
            }
            let collision = &pair.collision;
            let normal = collision.normal;
            let parent_a = collision.parent_a;
            let parent_b = collision.parent_b;
            let mut position_impulse = pair.separation - pair.slop * slop_dampen;

            if bodies[parent_a.0].is_static || bodies[parent_b.0].is_static {
                position_impulse *= 2.0;
            }

            if !(bodies[parent_a.0].is_static || bodies[parent_a.0].is_sleeping) {
                let contact_share = position_dampen / bodies[parent_a.0].total_contacts as f64;
                bodies[parent_a.0].position_impulse.x += normal.x * position_impulse * contact_share;
                bodies[parent_a.0].position_impulse.y += normal.y * position_impulse * contact_share;
            }

            if !(bodies[parent_b.0].is_static || bodies[parent_b.0].is_sleeping) {
                let contact_share = position_dampen / bodies[parent_b.0].total_contacts as f64;
                bodies[parent_b.0].position_impulse.x -= normal.x * position_impulse * contact_share;
                bodies[parent_b.0].position_impulse.y -= normal.y * position_impulse * contact_share;
            }
        }
    }

    pub fn post_solve_position(bodies: &mut [Body]) {
        for body in bodies.iter_mut() {
            body.total_contacts = 0;

            let impulse_x = body.position_impulse.x;
            let impulse_y = body.position_impulse.y;

            if impulse_x == 0.0 && impulse_y == 0.0 {
                continue;
            }

            // Update body geometry
            Vertices::translate(&mut body.vertices, &body.position_impulse, 1.0);
            Bounds::update(&mut body.bounds, &body.vertices, &body.velocity);
            body.position.x += impulse_x;
            body.position.y += impulse_y;

            // Move body without changing velocity
            body.position_prev.x += impulse_x;
            body.position_prev.y += impulse_y;

            if impulse_x * body.velocity.x + impulse_y * body.velocity.y < 0.0 {
                // Reset cached impulse if body has velocity along it
                body.position_impulse.x = 0.0;
                body.position_impulse.y = 0.0;
            } else {
                // Warm the next iteration
                body.position_impulse.x *= POSITION_WARMING;
                body.position_impulse.y *= POSITION_WARMING;
            }
        }
    }

    pub fn pre_solve_velocity(pairs: &Pairs, bodies: &mut [Body]) {
        for pair in pairs.table.values() {
            if !pair.is_active || pair.is_sensor {
                continue;
            }
            let collision = &pair.collision;
            let normal = collision.normal;
            let tangent = collision.tangent;
            let parent_a = collision.parent_a;
            let parent_b = collision.parent_b;

            for i in 0..pair.contact_count {
                let contact = &pair.contacts[i];
                let normal_impulse = contact.normal_impulse;
                let tangent_impulse = contact.tangent_impulse;

                if normal_impulse == 0.0 && tangent_impulse == 0.0 {
                    continue;
                }

                let impulse_x = normal.x * normal_impulse + tangent.x * tangent_impulse;
                let impulse_y = normal.y * normal_impulse + tangent.y * tangent_impulse;
                let contact_vertex = contact.vertex;

                if !(bodies[parent_a.0].is_static || bodies[parent_a.0].is_sleeping) {
                    let ba = &mut bodies[parent_a.0];
                    ba.position_prev.x += impulse_x * ba.inverse_mass;
                    ba.position_prev.y += impulse_y * ba.inverse_mass;
                    ba.angle_prev += ba.inverse_inertia
                        * ((contact_vertex.x - ba.position.x) * impulse_y
                            - (contact_vertex.y - ba.position.y) * impulse_x);
                }

                if !(bodies[parent_b.0].is_static || bodies[parent_b.0].is_sleeping) {
                    let bb = &mut bodies[parent_b.0];
                    bb.position_prev.x -= impulse_x * bb.inverse_mass;
                    bb.position_prev.y -= impulse_y * bb.inverse_mass;
                    bb.angle_prev -= bb.inverse_inertia
                        * ((contact_vertex.x - bb.position.x) * impulse_y
                            - (contact_vertex.y - bb.position.y) * impulse_x);
                }
            }
        }
    }

    pub fn solve_velocity(pairs: &mut Pairs, bodies: &mut [Body], delta: f64) {
        let time_scale = delta / BASE_DELTA;
        let time_scale_squared = time_scale * time_scale;
        let time_scale_cubed = time_scale_squared * time_scale;
        let resting_thresh = -RESTING_THRESH * time_scale;
        let resting_thresh_tangent = RESTING_THRESH_TANGENT;
        let friction_normal_multiplier = FRICTION_NORMAL_MULTIPLIER * time_scale;

        for pair in pairs.table.values_mut() {
            if !pair.is_active || pair.is_sensor {
                continue;
            }
            let normal_x = pair.collision.normal.x;
            let normal_y = pair.collision.normal.y;
            let tangent_x = pair.collision.tangent.x;
            let tangent_y = pair.collision.tangent.y;
            let inverse_mass_total = pair.inverse_mass;
            let friction = pair.friction * pair.friction_static * friction_normal_multiplier;
            let contact_count = pair.contact_count;
            let contact_share = 1.0 / contact_count as f64;

            let parent_a = pair.collision.parent_a;
            let parent_b = pair.collision.parent_b;

            // Get body velocities (Verlet: velocity = position - positionPrev)
            let body_a_vel_x = bodies[parent_a.0].position.x - bodies[parent_a.0].position_prev.x;
            let body_a_vel_y = bodies[parent_a.0].position.y - bodies[parent_a.0].position_prev.y;
            let body_a_ang_vel = bodies[parent_a.0].angle - bodies[parent_a.0].angle_prev;
            let body_b_vel_x = bodies[parent_b.0].position.x - bodies[parent_b.0].position_prev.x;
            let body_b_vel_y = bodies[parent_b.0].position.y - bodies[parent_b.0].position_prev.y;
            let body_b_ang_vel = bodies[parent_b.0].angle - bodies[parent_b.0].angle_prev;

            for j in 0..contact_count {
                let contact_vertex = pair.contacts[j].vertex;

                let offset_a_x = contact_vertex.x - bodies[parent_a.0].position.x;
                let offset_a_y = contact_vertex.y - bodies[parent_a.0].position.y;
                let offset_b_x = contact_vertex.x - bodies[parent_b.0].position.x;
                let offset_b_y = contact_vertex.y - bodies[parent_b.0].position.y;

                let vel_point_a_x = body_a_vel_x - offset_a_y * body_a_ang_vel;
                let vel_point_a_y = body_a_vel_y + offset_a_x * body_a_ang_vel;
                let vel_point_b_x = body_b_vel_x - offset_b_y * body_b_ang_vel;
                let vel_point_b_y = body_b_vel_y + offset_b_x * body_b_ang_vel;

                let relative_vel_x = vel_point_a_x - vel_point_b_x;
                let relative_vel_y = vel_point_a_y - vel_point_b_y;

                let normal_velocity =
                    normal_x * relative_vel_x + normal_y * relative_vel_y;
                let tangent_velocity =
                    tangent_x * relative_vel_x + tangent_y * relative_vel_y;

                // Coulomb friction
                let normal_overlap = pair.separation + normal_velocity;
                let normal_force = if normal_overlap < 0.0 {
                    0.0
                } else {
                    normal_overlap.min(1.0)
                };

                let friction_limit = normal_force * friction;

                let (mut tangent_impulse, max_friction) =
                    if tangent_velocity < -friction_limit || tangent_velocity > friction_limit {
                        let max_f = tangent_velocity.abs();
                        let mut ti = pair.friction
                            * if tangent_velocity > 0.0 { 1.0 } else { -1.0 }
                            * time_scale_cubed;
                        ti = ti.clamp(-max_f, max_f);
                        (ti, max_f)
                    } else {
                        (tangent_velocity, FRICTION_MAX_STATIC)
                    };

                // Account for mass, inertia and contact offset
                let oa_cn = offset_a_x * normal_y - offset_a_y * normal_x;
                let ob_cn = offset_b_x * normal_y - offset_b_y * normal_x;
                let share = contact_share
                    / (inverse_mass_total
                        + bodies[parent_a.0].inverse_inertia * oa_cn * oa_cn
                        + bodies[parent_b.0].inverse_inertia * ob_cn * ob_cn);

                // Raw impulses
                let mut normal_impulse =
                    (1.0 + pair.restitution) * normal_velocity * share;
                tangent_impulse *= share;

                // Handle high velocity and resting collisions
                if normal_velocity < resting_thresh {
                    pair.contacts[j].normal_impulse = 0.0;
                } else {
                    let contact_normal_impulse = pair.contacts[j].normal_impulse;
                    pair.contacts[j].normal_impulse += normal_impulse;
                    if pair.contacts[j].normal_impulse > 0.0 {
                        pair.contacts[j].normal_impulse = 0.0;
                    }
                    normal_impulse =
                        pair.contacts[j].normal_impulse - contact_normal_impulse;
                }

                // Tangent resting
                if tangent_velocity < -resting_thresh_tangent
                    || tangent_velocity > resting_thresh_tangent
                {
                    pair.contacts[j].tangent_impulse = 0.0;
                } else {
                    let contact_tangent_impulse = pair.contacts[j].tangent_impulse;
                    pair.contacts[j].tangent_impulse += tangent_impulse;
                    pair.contacts[j].tangent_impulse =
                        pair.contacts[j].tangent_impulse.clamp(-max_friction, max_friction);
                    tangent_impulse =
                        pair.contacts[j].tangent_impulse - contact_tangent_impulse;
                }

                // Total impulse from contact
                let impulse_x =
                    normal_x * normal_impulse + tangent_x * tangent_impulse;
                let impulse_y =
                    normal_y * normal_impulse + tangent_y * tangent_impulse;

                // Apply impulse
                if !(bodies[parent_a.0].is_static || bodies[parent_a.0].is_sleeping) {
                    let ba = &mut bodies[parent_a.0];
                    ba.position_prev.x += impulse_x * ba.inverse_mass;
                    ba.position_prev.y += impulse_y * ba.inverse_mass;
                    ba.angle_prev +=
                        (offset_a_x * impulse_y - offset_a_y * impulse_x) * ba.inverse_inertia;
                }

                if !(bodies[parent_b.0].is_static || bodies[parent_b.0].is_sleeping) {
                    let bb = &mut bodies[parent_b.0];
                    bb.position_prev.x -= impulse_x * bb.inverse_mass;
                    bb.position_prev.y -= impulse_y * bb.inverse_mass;
                    bb.angle_prev -=
                        (offset_b_x * impulse_y - offset_b_y * impulse_x) * bb.inverse_inertia;
                }
            }
        }
    }
}

// --- Engine ---

#[derive(Debug)]
pub struct Engine {
    pub bodies: Vec<Body>,
    pub constraints: Vec<Constraint>,
    pub pairs: Pairs,
    pub gravity: Gravity,
    pub timing: Timing,
    pub position_iterations: usize,
    pub velocity_iterations: usize,
    pub constraint_iterations: usize,
}

impl Default for Engine {
    fn default() -> Engine {
        Engine::new()
    }
}

impl Engine {
    pub fn new() -> Engine {
        Engine {
            bodies: Vec::new(),
            constraints: Vec::new(),
            pairs: Pairs::new(),
            gravity: Gravity::default(),
            timing: Timing::default(),
            position_iterations: 6,
            velocity_iterations: 4,
            constraint_iterations: 2,
        }
    }

    pub fn add_body(&mut self, body: Body) {
        self.bodies.push(body);
    }

    pub fn add_constraint(&mut self, constraint: Constraint) {
        self.constraints.push(constraint);
    }

    pub fn update(&mut self, delta: f64) -> Vec<PhysicsEvent> {
        let mut events = Vec::new();

        let delta = delta * self.timing.time_scale;
        self.timing.timestamp += delta;
        self.timing.last_delta = delta;

        // Apply gravity
        Self::apply_gravity(&mut self.bodies, &self.gravity);

        // Update bodies (Verlet integration)
        if delta > 0.0 {
            Self::update_bodies(&mut self.bodies, delta);
        }

        // Pre-solve constraints
        Constraint::pre_solve_all(&mut self.bodies);
        for _ in 0..self.constraint_iterations {
            Constraint::solve_all(&mut self.constraints, &mut self.bodies, delta);
        }
        Constraint::post_solve_all(&mut self.bodies);

        // Collision detection
        let collisions = self.detect_collisions();

        // Update pairs
        let timestamp = self.timing.timestamp;
        self.pairs.update(collisions, &self.bodies, timestamp);

        // Collision events (start)
        if !self.pairs.collision_start.is_empty() {
            events.push(PhysicsEvent::CollisionStart {
                pairs: self.pairs.collision_start.clone(),
            });
        }

        // Position solving
        let position_damping = (20.0 / self.position_iterations as f64).clamp(0.0, 1.0);

        Resolver::pre_solve_position(&self.pairs, &mut self.bodies);
        for _ in 0..self.position_iterations {
            Resolver::solve_position(&mut self.pairs, &mut self.bodies, delta, position_damping);
        }
        Resolver::post_solve_position(&mut self.bodies);

        // Constraints again after position solving
        Constraint::pre_solve_all(&mut self.bodies);
        for _ in 0..self.constraint_iterations {
            Constraint::solve_all(&mut self.constraints, &mut self.bodies, delta);
        }
        Constraint::post_solve_all(&mut self.bodies);

        // Velocity solving
        Resolver::pre_solve_velocity(&self.pairs, &mut self.bodies);
        for _ in 0..self.velocity_iterations {
            Resolver::solve_velocity(&mut self.pairs, &mut self.bodies, delta);
        }

        // Update velocities
        Self::update_body_velocities(&mut self.bodies);

        // Collision events (active + end)
        if !self.pairs.collision_active.is_empty() {
            events.push(PhysicsEvent::CollisionActive {
                pairs: self.pairs.collision_active.clone(),
            });
        }
        if !self.pairs.collision_end.is_empty() {
            events.push(PhysicsEvent::CollisionEnd {
                pairs: self.pairs.collision_end.clone(),
            });
        }

        // Clear forces
        Self::clear_forces(&mut self.bodies);

        events
    }

    fn detect_collisions(&self) -> Vec<CollisionResult> {
        let mut refs: Vec<&Body> = self.bodies.iter().collect();
        Detector::collisions(&mut refs, &self.pairs)
    }

    fn apply_gravity(bodies: &mut [Body], gravity: &Gravity) {
        if (gravity.x == 0.0 && gravity.y == 0.0) || gravity.scale == 0.0 {
            return;
        }
        for body in bodies.iter_mut() {
            if body.is_static || body.is_sleeping {
                continue;
            }
            body.force.x += body.mass * gravity.x * gravity.scale;
            body.force.y += body.mass * gravity.y * gravity.scale;
        }
    }

    fn update_bodies(bodies: &mut [Body], delta: f64) {
        for body in bodies.iter_mut() {
            if body.is_static || body.is_sleeping {
                continue;
            }
            body.update(delta);
        }
    }

    fn update_body_velocities(bodies: &mut [Body]) {
        for body in bodies.iter_mut() {
            body.update_velocities();
        }
    }

    fn clear_forces(bodies: &mut [Body]) {
        for body in bodies.iter_mut() {
            body.force.x = 0.0;
            body.force.y = 0.0;
            body.torque = 0.0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::BodyHandle;
    use crate::constraint::{Constraint, ConstraintOptions};
    use crate::geometry::Vec2;
    use serde_json::Value;

    const EPSILON: f64 = 1e-10;

    fn load_testdata() -> Value {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/testdata/engine.json");
        let content = std::fs::read_to_string(path).expect("Failed to read engine.json");
        serde_json::from_str(&content).expect("Failed to parse engine.json")
    }

    fn assert_f64_eq(actual: f64, expected: f64, label: &str) {
        let diff = (actual - expected).abs();
        let magnitude = actual.abs().max(expected.abs());
        let tolerance = if magnitude > 1.0 {
            EPSILON * magnitude
        } else {
            EPSILON
        };
        assert!(
            diff < tolerance,
            "{label}: actual {actual} expected {expected} (diff {diff})"
        );
    }

    fn assert_body_matches(body: &Body, expected: &Value, label: &str) {
        assert_f64_eq(body.position.x, expected["position"]["x"].as_f64().unwrap(), &format!("{label}.position.x"));
        assert_f64_eq(body.position.y, expected["position"]["y"].as_f64().unwrap(), &format!("{label}.position.y"));
        assert_f64_eq(body.velocity.x, expected["velocity"]["x"].as_f64().unwrap(), &format!("{label}.velocity.x"));
        assert_f64_eq(body.velocity.y, expected["velocity"]["y"].as_f64().unwrap(), &format!("{label}.velocity.y"));
        assert_f64_eq(body.speed, expected["speed"].as_f64().unwrap(), &format!("{label}.speed"));
        assert_f64_eq(body.angle, expected["angle"].as_f64().unwrap(), &format!("{label}.angle"));
        assert_f64_eq(body.angular_velocity, expected["angularVelocity"].as_f64().unwrap(), &format!("{label}.angularVelocity"));
    }

    #[test]
    fn engine_defaults() {
        let engine = Engine::new();
        assert_eq!(engine.position_iterations, 6);
        assert_eq!(engine.velocity_iterations, 4);
        assert_eq!(engine.constraint_iterations, 2);
        assert_f64_eq(engine.gravity.x, 0.0, "gravity.x");
        assert_f64_eq(engine.gravity.y, 1.0, "gravity.y");
        assert_f64_eq(engine.gravity.scale, 0.001, "gravity.scale");
        assert_f64_eq(engine.timing.timestamp, 0.0, "timestamp");
        assert_f64_eq(engine.timing.time_scale, 1.0, "timeScale");
    }

    #[test]
    fn engine_freefall_1tick() {
        let data = load_testdata();
        let expected = &data["engine_freefall_1tick"]["bodies"][0];

        let mut engine = Engine::new();
        engine.add_body(Body::new(BodyHandle(0)));
        engine.update(BASE_DELTA);

        assert_body_matches(&engine.bodies[0], expected, "body");
    }

    #[test]
    fn engine_freefall_10ticks() {
        let data = load_testdata();
        let ticks = data["engine_freefall_10ticks"]["ticks"].as_array().unwrap();

        let mut engine = Engine::new();
        engine.add_body(Body::new(BodyHandle(0)));

        for (i, expected) in ticks.iter().enumerate() {
            engine.update(BASE_DELTA);
            assert_body_matches(&engine.bodies[0], expected, &format!("tick{i}"));
        }
    }

    #[test]
    fn engine_floor_collision() {
        let data = load_testdata();
        let ticks = data["engine_floor_collision"]["ticks"].as_array().unwrap();
        let expected_collision_tick = data["engine_floor_collision"]["collisionStartTick"]
            .as_u64().unwrap() as usize;

        let mut engine = Engine::new();
        // body at (0,0)
        engine.add_body(Body::new(BodyHandle(0)));
        // static floor at (0,100)
        let mut floor = Body::new(BodyHandle(1));
        floor.set_position(Vec2 { x: 0.0, y: 100.0 }, false);
        floor.set_static(true);
        engine.add_body(floor);

        let mut collision_start_tick = None;
        for i in 0..60 {
            let events = engine.update(BASE_DELTA);
            if collision_start_tick.is_none()
                && events.iter().any(|e| matches!(e, PhysicsEvent::CollisionStart { .. }))
            {
                collision_start_tick = Some(i);
            }
            assert_body_matches(&engine.bodies[0], &ticks[i], &format!("tick{i}"));
        }

        assert_eq!(
            collision_start_tick,
            Some(expected_collision_tick),
            "collision should start at tick {expected_collision_tick}"
        );
    }

    #[test]
    fn engine_head_on_collision() {
        let data = load_testdata();
        let ticks = data["engine_head_on"]["ticks"].as_array().unwrap();

        let mut engine = Engine::new();
        engine.gravity = Gravity { x: 0.0, y: 0.0, scale: 0.0 };

        let mut body_a = Body::new(BodyHandle(0));
        body_a.set_position(Vec2 { x: -50.0, y: 0.0 }, false);
        body_a.set_velocity(Vec2 { x: 5.0, y: 0.0 });

        let mut body_b = Body::new(BodyHandle(1));
        body_b.set_position(Vec2 { x: 50.0, y: 0.0 }, false);
        body_b.set_velocity(Vec2 { x: -5.0, y: 0.0 });

        engine.add_body(body_a);
        engine.add_body(body_b);

        for (i, expected) in ticks.iter().enumerate() {
            engine.update(BASE_DELTA);
            assert_body_matches(&engine.bodies[0], &expected["bodyA"], &format!("tick{i}.bodyA"));
            assert_body_matches(&engine.bodies[1], &expected["bodyB"], &format!("tick{i}.bodyB"));
        }
    }

    #[test]
    fn engine_constraint() {
        let data = load_testdata();
        let ticks = data["engine_constraint"]["ticks"].as_array().unwrap();

        let mut engine = Engine::new();
        engine.add_body(Body::new(BodyHandle(0)));

        let constraint = Constraint::new(
            crate::constraint::ConstraintHandle(0),
            ConstraintOptions {
                body_a: Some(BodyHandle(0)),
                body_b: None,
                point_a: None,
                point_b: Some(Vec2 { x: 0.0, y: 0.0 }),
                length: None,
                stiffness: Some(0.5),
            },
            &engine.bodies,
        );
        engine.add_constraint(constraint);

        for (i, expected) in ticks.iter().enumerate() {
            engine.update(BASE_DELTA);
            assert_body_matches(&engine.bodies[0], expected, &format!("tick{i}"));
        }
    }
}
