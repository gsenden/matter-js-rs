use crate::body::{Body, BodyHandle};
use crate::geometry::{Vec2, Vertices, Bounds, Axes};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ConstraintHandle(pub usize);

const WARMING: f64 = 0.4;
const TORQUE_DAMPEN: f64 = 1.0;
const MIN_LENGTH: f64 = 0.000001;
const BASE_DELTA: f64 = 1000.0 / 60.0;

#[derive(Debug)]
pub struct Constraint {
    pub handle: ConstraintHandle,
    pub body_a: Option<BodyHandle>,
    pub body_b: Option<BodyHandle>,
    pub point_a: Vec2,
    pub point_b: Vec2,
    pub length: f64,
    pub stiffness: f64,
    pub damping: f64,
    pub angular_stiffness: f64,
    pub angle_a: f64,
    pub angle_b: f64,
}

pub struct ConstraintOptions {
    pub body_a: Option<BodyHandle>,
    pub body_b: Option<BodyHandle>,
    pub point_a: Option<Vec2>,
    pub point_b: Option<Vec2>,
    pub length: Option<f64>,
    pub stiffness: Option<f64>,
}

impl Constraint {
    pub fn new(handle: ConstraintHandle, options: ConstraintOptions, bodies: &[Body]) -> Constraint {
        let point_a = options.point_a.unwrap_or(Vec2 { x: 0.0, y: 0.0 });
        let point_b = options.point_b.unwrap_or(Vec2 { x: 0.0, y: 0.0 });
        let body_a = options.body_a;
        let body_b = options.body_b;

        let initial_point_a = match body_a {
            Some(h) => Vec2 { x: bodies[h.0].position.x + point_a.x, y: bodies[h.0].position.y + point_a.y },
            None => point_a,
        };
        let initial_point_b = match body_b {
            Some(h) => Vec2 { x: bodies[h.0].position.x + point_b.x, y: bodies[h.0].position.y + point_b.y },
            None => point_b,
        };

        let calculated_length = Vec2::magnitude(&Vec2 {
            x: initial_point_a.x - initial_point_b.x,
            y: initial_point_a.y - initial_point_b.y,
        });
        let length = options.length.unwrap_or(calculated_length);

        let stiffness = options.stiffness.unwrap_or(if length > 0.0 { 1.0 } else { 0.7 });

        let angle_a = body_a.map_or(0.0, |h| bodies[h.0].angle);
        let angle_b = body_b.map_or(0.0, |h| bodies[h.0].angle);

        Constraint {
            handle,
            body_a,
            body_b,
            point_a,
            point_b,
            length,
            stiffness,
            damping: 0.0,
            angular_stiffness: 0.0,
            angle_a,
            angle_b,
        }
    }

    pub fn solve(&mut self, bodies: &mut [Body], time_scale: f64) {
        let body_a = self.body_a;
        let body_b = self.body_b;

        if body_a.is_none() && body_b.is_none() {
            return;
        }

        // Update reference angles and rotate points
        if let Some(ha) = body_a.filter(|h| !bodies[h.0].is_static) {
            let delta_angle = bodies[ha.0].angle - self.angle_a;
            if delta_angle != 0.0 {
                let cos = delta_angle.cos();
                let sin = delta_angle.sin();
                let px = self.point_a.x;
                let py = self.point_a.y;
                self.point_a.x = px * cos - py * sin;
                self.point_a.y = px * sin + py * cos;
            }
            self.angle_a = bodies[ha.0].angle;
        }

        if let Some(hb) = body_b.filter(|h| !bodies[h.0].is_static) {
            let delta_angle = bodies[hb.0].angle - self.angle_b;
            if delta_angle != 0.0 {
                let cos = delta_angle.cos();
                let sin = delta_angle.sin();
                let px = self.point_b.x;
                let py = self.point_b.y;
                self.point_b.x = px * cos - py * sin;
                self.point_b.y = px * sin + py * cos;
            }
            self.angle_b = bodies[hb.0].angle;
        }

        let point_a_world = match body_a {
            Some(h) => Vec2 {
                x: bodies[h.0].position.x + self.point_a.x,
                y: bodies[h.0].position.y + self.point_a.y,
            },
            None => self.point_a,
        };
        let point_b_world = match body_b {
            Some(h) => Vec2 {
                x: bodies[h.0].position.x + self.point_b.x,
                y: bodies[h.0].position.y + self.point_b.y,
            },
            None => self.point_b,
        };

        let delta = Vec2 {
            x: point_a_world.x - point_b_world.x,
            y: point_a_world.y - point_b_world.y,
        };

        let mut current_length = Vec2::magnitude(&delta);
        if current_length < MIN_LENGTH {
            current_length = MIN_LENGTH;
        }

        let difference = (current_length - self.length) / current_length;
        let is_rigid = self.stiffness >= 1.0 || self.length == 0.0;
        let stiffness = if is_rigid {
            self.stiffness * time_scale
        } else {
            self.stiffness * time_scale * time_scale
        };
        let damping = self.damping * time_scale;

        let force = Vec2 {
            x: delta.x * difference * stiffness,
            y: delta.y * difference * stiffness,
        };

        let mass_total = body_a.map_or(0.0, |h| bodies[h.0].inverse_mass)
            + body_b.map_or(0.0, |h| bodies[h.0].inverse_mass);
        let inertia_total = body_a.map_or(0.0, |h| bodies[h.0].inverse_inertia)
            + body_b.map_or(0.0, |h| bodies[h.0].inverse_inertia);
        let resistance_total = mass_total + inertia_total;

        let mut normal_velocity = 0.0;
        let mut normal = Vec2 { x: 0.0, y: 0.0 };

        if damping > 0.0 {
            normal = Vec2 {
                x: delta.x / current_length,
                y: delta.y / current_length,
            };

            let rel_vel_x = body_b.map_or(0.0, |h| bodies[h.0].position.x - bodies[h.0].position_prev.x)
                - body_a.map_or(0.0, |h| bodies[h.0].position.x - bodies[h.0].position_prev.x);
            let rel_vel_y = body_b.map_or(0.0, |h| bodies[h.0].position.y - bodies[h.0].position_prev.y)
                - body_a.map_or(0.0, |h| bodies[h.0].position.y - bodies[h.0].position_prev.y);

            normal_velocity = normal.x * rel_vel_x + normal.y * rel_vel_y;
        }

        // Apply to body A
        if let Some(ha) = body_a.filter(|h| !bodies[h.0].is_static) {
            let share = bodies[ha.0].inverse_mass / mass_total;

            bodies[ha.0].constraint_impulse.x -= force.x * share;
            bodies[ha.0].constraint_impulse.y -= force.y * share;
            bodies[ha.0].position.x -= force.x * share;
            bodies[ha.0].position.y -= force.y * share;

            if damping > 0.0 {
                bodies[ha.0].position_prev.x -= damping * normal.x * normal_velocity * share;
                bodies[ha.0].position_prev.y -= damping * normal.y * normal_velocity * share;
            }

            let torque = (self.point_a.x * force.y - self.point_a.y * force.x)
                / resistance_total * TORQUE_DAMPEN * bodies[ha.0].inverse_inertia
                * (1.0 - self.angular_stiffness);
            bodies[ha.0].constraint_impulse.angle -= torque;
            bodies[ha.0].angle -= torque;
        }

        // Apply to body B
        if let Some(hb) = body_b.filter(|h| !bodies[h.0].is_static) {
            let share = bodies[hb.0].inverse_mass / mass_total;

            bodies[hb.0].constraint_impulse.x += force.x * share;
            bodies[hb.0].constraint_impulse.y += force.y * share;
            bodies[hb.0].position.x += force.x * share;
            bodies[hb.0].position.y += force.y * share;

            if damping > 0.0 {
                bodies[hb.0].position_prev.x += damping * normal.x * normal_velocity * share;
                bodies[hb.0].position_prev.y += damping * normal.y * normal_velocity * share;
            }

            let torque = (self.point_b.x * force.y - self.point_b.y * force.x)
                / resistance_total * TORQUE_DAMPEN * bodies[hb.0].inverse_inertia
                * (1.0 - self.angular_stiffness);
            bodies[hb.0].constraint_impulse.angle += torque;
            bodies[hb.0].angle += torque;
        }
    }

    pub fn solve_all(constraints: &mut [Constraint], bodies: &mut [Body], delta: f64) {
        let time_scale = (delta / BASE_DELTA).clamp(0.0, 1.0);

        // Solve fixed constraints first
        for constraint in constraints.iter_mut() {
            let fixed_a = constraint.body_a.is_none()
                || constraint.body_a.is_some_and(|h| bodies[h.0].is_static);
            let fixed_b = constraint.body_b.is_none()
                || constraint.body_b.is_some_and(|h| bodies[h.0].is_static);
            if fixed_a || fixed_b {
                constraint.solve(bodies, time_scale);
            }
        }

        // Solve free constraints last
        for constraint in constraints.iter_mut() {
            let fixed_a = constraint.body_a.is_none()
                || constraint.body_a.is_some_and(|h| bodies[h.0].is_static);
            let fixed_b = constraint.body_b.is_none()
                || constraint.body_b.is_some_and(|h| bodies[h.0].is_static);
            if !fixed_a && !fixed_b {
                constraint.solve(bodies, time_scale);
            }
        }
    }

    pub fn pre_solve_all(bodies: &mut [Body]) {
        for body in bodies.iter_mut() {
            let impulse = &body.constraint_impulse;
            if body.is_static || (impulse.x == 0.0 && impulse.y == 0.0 && impulse.angle == 0.0) {
                continue;
            }
            body.position.x += body.constraint_impulse.x;
            body.position.y += body.constraint_impulse.y;
            body.angle += body.constraint_impulse.angle;
        }
    }

    pub fn post_solve_all(bodies: &mut [Body]) {
        for body in bodies.iter_mut() {
            let ix = body.constraint_impulse.x;
            let iy = body.constraint_impulse.y;
            let ia = body.constraint_impulse.angle;

            if body.is_static || (ix == 0.0 && iy == 0.0 && ia == 0.0) {
                continue;
            }

            // Update geometry
            let impulse_vec = Vec2 { x: ix, y: iy };
            Vertices::translate(&mut body.vertices, &impulse_vec, 1.0);

            if ia != 0.0 {
                Vertices::rotate(&mut body.vertices, ia, &body.position);
                Axes::rotate(&mut body.axes, ia);
            }

            let velocity = body.velocity;
            Bounds::update(&mut body.bounds, &body.vertices, &velocity);

            // Dampen for warming
            body.constraint_impulse.x *= WARMING;
            body.constraint_impulse.y *= WARMING;
            body.constraint_impulse.angle *= WARMING;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    const EPSILON: f64 = 1e-14;

    fn load_testdata() -> Value {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../testdata/constraint.json");
        let content = std::fs::read_to_string(path).expect("Failed to read constraint.json");
        serde_json::from_str(&content).expect("Failed to parse constraint.json")
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

    fn assert_vec2_eq(actual: &Vec2, expected: &Value) {
        assert_f64_eq(actual.x, expected["x"].as_f64().unwrap());
        assert_f64_eq(actual.y, expected["y"].as_f64().unwrap());
    }

    fn make_body_at(handle: BodyHandle, x: f64, y: f64) -> Body {
        let mut body = Body::new(handle);
        body.set_position(Vec2 { x, y }, false);
        body
    }

    #[test]
    fn constraint_defaults() {
        let data = load_testdata();
        let expected = &data["constraint_defaults"];

        let bodies = vec![
            make_body_at(BodyHandle(0), 0.0, 0.0),
            make_body_at(BodyHandle(1), 100.0, 0.0),
        ];

        let constraint = Constraint::new(
            ConstraintHandle(0),
            ConstraintOptions {
                body_a: Some(BodyHandle(0)),
                body_b: Some(BodyHandle(1)),
                point_a: None, point_b: None, length: None, stiffness: None,
            },
            &bodies,
        );

        assert_f64_eq(constraint.length, expected["length"].as_f64().unwrap());
        assert_f64_eq(constraint.stiffness, expected["stiffness"].as_f64().unwrap());
        assert_f64_eq(constraint.damping, expected["damping"].as_f64().unwrap());
        assert_f64_eq(constraint.angular_stiffness, expected["angularStiffness"].as_f64().unwrap());
        assert_vec2_eq(&constraint.point_a, &expected["pointA"]);
        assert_vec2_eq(&constraint.point_b, &expected["pointB"]);
    }

    #[test]
    fn constraint_solve_pulls_bodies() {
        let data = load_testdata();
        let expected_a = &data["constraint_solve"]["afterA"];
        let expected_b = &data["constraint_solve"]["afterB"];

        let mut bodies = vec![
            make_body_at(BodyHandle(0), 0.0, 0.0),
            make_body_at(BodyHandle(1), 60.0, 0.0),
        ];

        let mut constraint = Constraint::new(
            ConstraintHandle(0),
            ConstraintOptions {
                body_a: Some(BodyHandle(0)),
                body_b: Some(BodyHandle(1)),
                point_a: None, point_b: None,
                length: Some(40.0),
                stiffness: Some(1.0),
            },
            &bodies,
        );

        constraint.solve(&mut bodies, 1.0);

        assert_vec2_eq(&bodies[0].position, &expected_a["position"]);
        assert_vec2_eq(&bodies[1].position, &expected_b["position"]);
        assert_f64_eq(bodies[0].constraint_impulse.x, expected_a["constraintImpulse"]["x"].as_f64().unwrap());
        assert_f64_eq(bodies[0].constraint_impulse.y, expected_a["constraintImpulse"]["y"].as_f64().unwrap());
    }

    #[test]
    fn constraint_pin_to_world() {
        let data = load_testdata();
        let expected = &data["constraint_pin"];

        let bodies = vec![make_body_at(BodyHandle(0), 50.0, 50.0)];

        let constraint = Constraint::new(
            ConstraintHandle(0),
            ConstraintOptions {
                body_a: Some(BodyHandle(0)),
                body_b: None,
                point_a: None,
                point_b: Some(Vec2 { x: 0.0, y: 0.0 }),
                length: None, stiffness: None,
            },
            &bodies,
        );

        assert_f64_eq(constraint.length, expected["length"].as_f64().unwrap());
        assert_f64_eq(constraint.stiffness, expected["stiffness"].as_f64().unwrap());
    }

    #[test]
    fn constraint_post_solve_dampens_impulse() {
        let mut bodies = vec![make_body_at(BodyHandle(0), 0.0, 0.0)];
        bodies[0].constraint_impulse.x = 10.0;
        bodies[0].constraint_impulse.y = 5.0;

        Constraint::post_solve_all(&mut bodies);

        assert_f64_eq(bodies[0].constraint_impulse.x, 10.0 * WARMING);
        assert_f64_eq(bodies[0].constraint_impulse.y, 5.0 * WARMING);
    }
}
