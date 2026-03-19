use crate::body::{Body, BodyHandle};
use crate::geometry::{Vec2, Vertices, Bounds, Axes};

const INERTIA_SCALE: f64 = 4.0;

pub struct Bodies;

impl Bodies {
    pub fn from_vertices(handle: BodyHandle, position: Vec2, raw_vertices: Vec<Vec2>) -> Body {
        let mut vertices = raw_vertices;

        let area = Vertices::area(&vertices, false);
        let density = 0.001;
        let mass = density * area;

        // Centre vertices around origin
        let centre = Vertices::centre(&vertices);
        Vertices::translate(&mut vertices, &centre, -1.0);

        // Calculate inertia with vertices at origin
        let inertia = INERTIA_SCALE * Vertices::inertia(&vertices, mass);

        // Translate vertices to body position
        Vertices::translate(&mut vertices, &position, 1.0);

        let axes = Axes::from_vertices(&vertices);
        let bounds = Bounds::create(&vertices);

        Body {
            id: handle.0,
            handle,
            position,
            position_prev: position,
            velocity: Vec2 { x: 0.0, y: 0.0 },
            force: Vec2 { x: 0.0, y: 0.0 },
            torque: 0.0,
            position_impulse: Vec2 { x: 0.0, y: 0.0 },
            constraint_impulse: crate::body::ConstraintImpulse { x: 0.0, y: 0.0, angle: 0.0 },
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
            collision_filter: crate::body::CollisionFilter {
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

    pub fn rectangle(handle: BodyHandle, x: f64, y: f64, width: f64, height: f64) -> Body {
        let vertices = vec![
            Vec2 { x: 0.0, y: 0.0 },
            Vec2 { x: width, y: 0.0 },
            Vec2 { x: width, y: height },
            Vec2 { x: 0.0, y: height },
        ];
        Self::from_vertices(handle, Vec2 { x, y }, vertices)
    }

    pub fn circle(handle: BodyHandle, x: f64, y: f64, radius: f64) -> Body {
        let max_sides = 25;
        let mut sides = (radius.ceil() as usize).max(10).min(max_sides);
        if sides % 2 == 1 {
            sides += 1;
        }
        let mut body = Self::polygon(handle, x, y, sides, radius);
        body.circle_radius = radius;
        body
    }

    pub fn polygon(handle: BodyHandle, x: f64, y: f64, sides: usize, radius: f64) -> Body {
        let theta = 2.0 * std::f64::consts::PI / sides as f64;
        let offset = theta * 0.5;

        let vertices: Vec<Vec2> = (0..sides)
            .map(|i| {
                let angle = offset + (i as f64) * theta;
                // Match Matter.js toFixed(3) precision
                let vx = (angle.cos() * radius * 1000.0).round() / 1000.0;
                let vy = (angle.sin() * radius * 1000.0).round() / 1000.0;
                Vec2 { x: vx, y: vy }
            })
            .collect();

        Self::from_vertices(handle, Vec2 { x, y }, vertices)
    }

    pub fn trapezoid(handle: BodyHandle, x: f64, y: f64, width: f64, height: f64, slope: f64) -> Body {
        let slope = slope * 0.5;
        let roof = (1.0 - (slope * 2.0)) * width;
        let x1 = width * slope;
        let x2 = x1 + roof;
        let x3 = x2 + x1;

        let vertices = if slope < 0.5 {
            vec![
                Vec2 { x: 0.0, y: 0.0 },
                Vec2 { x: x1, y: -height },
                Vec2 { x: x2, y: -height },
                Vec2 { x: x3, y: 0.0 },
            ]
        } else {
            vec![
                Vec2 { x: 0.0, y: 0.0 },
                Vec2 { x: x2, y: -height },
                Vec2 { x: x3, y: 0.0 },
            ]
        };

        Self::from_vertices(handle, Vec2 { x, y }, vertices)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    const EPSILON: f64 = 1e-10;

    fn load_testdata() -> Value {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../testdata/factory.json");
        let content = std::fs::read_to_string(path).expect("Failed to read factory.json");
        serde_json::from_str(&content).expect("Failed to parse factory.json")
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
        assert_f64_eq(body.area, expected["area"].as_f64().unwrap(), &format!("{label}.area"));
        assert_f64_eq(body.mass, expected["mass"].as_f64().unwrap(), &format!("{label}.mass"));
        assert_f64_eq(body.inertia, expected["inertia"].as_f64().unwrap(), &format!("{label}.inertia"));
        assert_f64_eq(body.inverse_mass, expected["inverseMass"].as_f64().unwrap(), &format!("{label}.inverseMass"));
        assert_f64_eq(body.inverse_inertia, expected["inverseInertia"].as_f64().unwrap(), &format!("{label}.inverseInertia"));

        // Vertices
        let exp_verts = expected["vertices"].as_array().unwrap();
        assert_eq!(body.vertices.len(), exp_verts.len(), "{label}.vertices.len");
        for (i, (v, ev)) in body.vertices.iter().zip(exp_verts.iter()).enumerate() {
            assert_f64_eq(v.x, ev["x"].as_f64().unwrap(), &format!("{label}.vertices[{i}].x"));
            assert_f64_eq(v.y, ev["y"].as_f64().unwrap(), &format!("{label}.vertices[{i}].y"));
        }
    }

    #[test]
    fn factory_rectangle() {
        let data = load_testdata();
        let body = Bodies::rectangle(BodyHandle(0), 50.0, 80.0, 100.0, 60.0);
        assert_body_matches(&body, &data["factory_rectangle"], "rect");
    }

    #[test]
    fn factory_rectangle_origin() {
        let data = load_testdata();
        let body = Bodies::rectangle(BodyHandle(0), 0.0, 0.0, 40.0, 40.0);
        assert_body_matches(&body, &data["factory_rectangle_origin"], "rect_origin");
    }

    #[test]
    fn factory_circle_r10() {
        let data = load_testdata();
        let body = Bodies::circle(BodyHandle(0), 100.0, 200.0, 10.0);
        assert_body_matches(&body, &data["factory_circle_r10"], "circle_r10");
    }

    #[test]
    fn factory_circle_r25() {
        let data = load_testdata();
        let body = Bodies::circle(BodyHandle(0), 0.0, 0.0, 25.0);
        assert_body_matches(&body, &data["factory_circle_r25"], "circle_r25");
    }

    #[test]
    fn factory_circle_r50() {
        let data = load_testdata();
        let body = Bodies::circle(BodyHandle(0), 50.0, 50.0, 50.0);
        assert_body_matches(&body, &data["factory_circle_r50"], "circle_r50");
    }

    #[test]
    fn factory_triangle() {
        let data = load_testdata();
        let body = Bodies::polygon(BodyHandle(0), 0.0, 0.0, 3, 30.0);
        assert_body_matches(&body, &data["factory_triangle"], "triangle");
    }

    #[test]
    fn factory_pentagon() {
        let data = load_testdata();
        let body = Bodies::polygon(BodyHandle(0), 100.0, 100.0, 5, 40.0);
        assert_body_matches(&body, &data["factory_pentagon"], "pentagon");
    }

    #[test]
    fn factory_hexagon() {
        let data = load_testdata();
        let body = Bodies::polygon(BodyHandle(0), 0.0, 0.0, 6, 50.0);
        assert_body_matches(&body, &data["factory_hexagon"], "hexagon");
    }

    #[test]
    fn factory_trapezoid() {
        let data = load_testdata();
        let body = Bodies::trapezoid(BodyHandle(0), 0.0, 0.0, 100.0, 50.0, 0.3);
        assert_body_matches(&body, &data["factory_trapezoid"], "trapezoid");
    }

    #[test]
    fn factory_trapezoid_steep() {
        let data = load_testdata();
        let body = Bodies::trapezoid(BodyHandle(0), 50.0, 50.0, 80.0, 40.0, 0.5);
        assert_body_matches(&body, &data["factory_trapezoid_steep"], "trapezoid_steep");
    }
}
