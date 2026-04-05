use crate::body::{Body, BodyHandle, CollisionFilter};
use crate::geometry::{Vec2, Vertices};
use crate::engine::CollisionResponsePolicy;

#[derive(Debug, Clone)]
pub struct Contact {
    pub vertex: Vec2,
    pub normal_impulse: f64,
    pub tangent_impulse: f64,
}

impl Default for Contact {
    fn default() -> Contact {
        Contact {
            vertex: Vec2 { x: 0.0, y: 0.0 },
            normal_impulse: 0.0,
            tangent_impulse: 0.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct CollisionResult {
    pub collided: bool,
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub parent_a: BodyHandle,
    pub parent_b: BodyHandle,
    pub depth: f64,
    pub normal: Vec2,
    pub tangent: Vec2,
    pub penetration: Vec2,
    pub supports: Vec<Vec2>,
}

struct OverlapResult {
    overlap: f64,
    axis: Vec2,
}

pub struct Collision;

impl Collision {
    pub fn collides(body_a: &Body, body_b: &Body) -> Option<CollisionResult> {
        let overlap_ab = Self::overlap_axes(&body_a.vertices, &body_b.vertices, &body_a.axes);
        if overlap_ab.overlap <= 0.0 {
            return None;
        }

        let overlap_ba = Self::overlap_axes(&body_b.vertices, &body_a.vertices, &body_b.axes);
        if overlap_ba.overlap <= 0.0 {
            return None;
        }

        // Order bodies by id (like Matter.js)
        let (body_a, body_b) = if body_a.id < body_b.id {
            (body_a, body_b)
        } else {
            (body_b, body_a)
        };

        let min_overlap = if overlap_ab.overlap < overlap_ba.overlap {
            &overlap_ab
        } else {
            &overlap_ba
        };

        let depth = min_overlap.overlap;
        let mut normal_x = min_overlap.axis.x;
        let mut normal_y = min_overlap.axis.y;

        // Ensure normal faces away from bodyA
        let delta_x = body_b.position.x - body_a.position.x;
        let delta_y = body_b.position.y - body_a.position.y;
        if normal_x * delta_x + normal_y * delta_y >= 0.0 {
            normal_x = -normal_x;
            normal_y = -normal_y;
        }

        let normal = Vec2 { x: normal_x, y: normal_y };
        let tangent = Vec2 { x: -normal_y, y: normal_x };
        let penetration = Vec2 { x: normal_x * depth, y: normal_y * depth };

        // Find support points
        let mut supports = Vec::new();

        let supports_b = Self::find_supports(body_a, body_b, &normal, 1.0);
        if Vertices::contains(&body_a.vertices, &supports_b[0]) {
            supports.push(supports_b[0]);
        }
        if Vertices::contains(&body_a.vertices, &supports_b[1]) {
            supports.push(supports_b[1]);
        }

        if supports.len() < 2 {
            let supports_a = Self::find_supports(body_b, body_a, &normal, -1.0);
            if Vertices::contains(&body_b.vertices, &supports_a[0]) {
                supports.push(supports_a[0]);
            }
            if supports.len() < 2 && Vertices::contains(&body_b.vertices, &supports_a[1]) {
                supports.push(supports_a[1]);
            }
        }

        // Edge case: overlapping but no vertex containment
        if supports.is_empty() {
            supports.push(supports_b[0]);
        }

        Some(CollisionResult {
            collided: true,
            body_a: body_a.handle,
            body_b: body_b.handle,
            parent_a: body_a.parent,
            parent_b: body_b.parent,
            depth,
            normal,
            tangent,
            penetration,
            supports,
        })
    }

    fn overlap_axes(vertices_a: &[Vec2], vertices_b: &[Vec2], axes: &[Vec2]) -> OverlapResult {
        let mut overlap_min = f64::MAX;
        let mut overlap_axis_number = 0;

        for (i, axis) in axes.iter().enumerate() {
            let axis_x = axis.x;
            let axis_y = axis.y;

            let mut min_a = vertices_a[0].x * axis_x + vertices_a[0].y * axis_y;
            let mut max_a = min_a;
            for v in &vertices_a[1..] {
                let dot = v.x * axis_x + v.y * axis_y;
                if dot > max_a {
                    max_a = dot;
                } else if dot < min_a {
                    min_a = dot;
                }
            }

            let mut min_b = vertices_b[0].x * axis_x + vertices_b[0].y * axis_y;
            let mut max_b = min_b;
            for v in &vertices_b[1..] {
                let dot = v.x * axis_x + v.y * axis_y;
                if dot > max_b {
                    max_b = dot;
                } else if dot < min_b {
                    min_b = dot;
                }
            }

            let overlap_ab = max_a - min_b;
            let overlap_ba = max_b - min_a;
            let overlap = if overlap_ab < overlap_ba { overlap_ab } else { overlap_ba };

            if overlap < overlap_min {
                overlap_min = overlap;
                overlap_axis_number = i;

                if overlap <= 0.0 {
                    break;
                }
            }
        }

        OverlapResult {
            overlap: overlap_min,
            axis: axes[overlap_axis_number],
        }
    }

    fn find_supports(body_a: &Body, body_b: &Body, normal: &Vec2, direction: f64) -> [Vec2; 2] {
        let vertices = &body_b.vertices;
        let vertices_length = vertices.len();
        let normal_x = normal.x * direction;
        let normal_y = normal.y * direction;

        let mut nearest_distance = normal_x * (body_a.position.x - vertices[0].x)
            + normal_y * (body_a.position.y - vertices[0].y);
        let mut vertex_a_index = 0;

        // Find deepest vertex relative to the axis (hill-climbing)
        for (j, v) in vertices.iter().enumerate().skip(1) {
            let distance = normal_x * (body_a.position.x - v.x)
                + normal_y * (body_a.position.y - v.y);
            if distance < nearest_distance {
                nearest_distance = distance;
                vertex_a_index = j;
            }
        }

        // Measure previous vertex
        let prev_index = (vertices_length + vertex_a_index - 1) % vertices_length;
        let vertex_c = vertices[prev_index];
        let nearest_distance_c = normal_x * (body_a.position.x - vertex_c.x)
            + normal_y * (body_a.position.y - vertex_c.y);

        // Compare with next vertex
        let next_index = (vertex_a_index + 1) % vertices_length;
        let vertex_b = vertices[next_index];
        if normal_x * (body_a.position.x - vertex_b.x) + normal_y * (body_a.position.y - vertex_b.y)
            < nearest_distance_c
        {
            [vertices[vertex_a_index], vertex_b]
        } else {
            [vertices[vertex_a_index], vertex_c]
        }
    }
}

// --- Pair ---

pub fn pair_id(body_a: BodyHandle, body_b: BodyHandle) -> (usize, usize) {
    if body_a.0 < body_b.0 {
        (body_a.0, body_b.0)
    } else {
        (body_b.0, body_a.0)
    }
}

#[derive(Debug, Clone)]
pub struct Pair {
    pub id: (usize, usize),
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub collision: CollisionResult,
    pub contacts: [Contact; 2],
    pub contact_count: usize,
    pub separation: f64,
    pub is_active: bool,
    pub is_sensor: bool,
    pub time_created: f64,
    pub time_updated: f64,
    pub inverse_mass: f64,
    pub friction: f64,
    pub friction_static: f64,
    pub restitution: f64,
    pub slop: f64,
    pub collision_response_policy: CollisionResponsePolicy,
}

impl Pair {
    pub fn create(collision: CollisionResult, bodies: &[Body], timestamp: f64) -> Pair {
        let id = pair_id(collision.body_a, collision.body_b);
        let body_a_ref = &bodies[collision.body_a.0];
        let body_b_ref = &bodies[collision.body_b.0];

        let mut pair = Pair {
            id,
            body_a: collision.body_a,
            body_b: collision.body_b,
            contacts: [Contact::default(), Contact::default()],
            contact_count: 0,
            separation: 0.0,
            is_active: true,
            is_sensor: body_a_ref.is_sensor || body_b_ref.is_sensor,
            time_created: timestamp,
            time_updated: timestamp,
            inverse_mass: 0.0,
            friction: 0.0,
            friction_static: 0.0,
            restitution: 0.0,
            slop: 0.0,
            collision_response_policy: CollisionResponsePolicy::Default,
            collision,
        };
        pair.update_from_bodies(bodies);
        pair
    }

    pub fn update(&mut self, collision: CollisionResult, bodies: &[Body], timestamp: f64) {
        self.is_active = true;
        self.time_updated = timestamp;
        self.collision = collision;
        self.update_from_bodies(bodies);
    }

    fn update_from_bodies(&mut self, bodies: &[Body]) {
        let parent_a = &bodies[self.collision.parent_a.0];
        let parent_b = &bodies[self.collision.parent_b.0];

        self.separation = self.collision.depth;
        self.inverse_mass = parent_a.inverse_mass + parent_b.inverse_mass;
        self.friction = parent_a.friction.min(parent_b.friction);
        self.friction_static = parent_a.friction_static.max(parent_b.friction_static);
        self.restitution = parent_a.restitution.max(parent_b.restitution);
        self.slop = parent_a.slop.max(parent_b.slop);
        self.contact_count = self.collision.supports.len();

        // Update contacts from supports
        for (i, support) in self.collision.supports.iter().enumerate() {
            if i < 2 {
                self.contacts[i].vertex = *support;
            }
        }
    }

    pub fn set_active(&mut self, is_active: bool, _timestamp: f64) {
        if is_active {
            self.is_active = true;
        } else {
            self.is_active = false;
            self.contact_count = 0;
        }
    }
}

// --- Pairs ---

use std::collections::HashMap;

#[derive(Debug, Default)]
pub struct Pairs {
    pub table: HashMap<(usize, usize), Pair>,
    pub collision_start: Vec<(usize, usize)>,
    pub collision_active: Vec<(usize, usize)>,
    pub collision_end: Vec<(usize, usize)>,
    pub collision_end_pairs: Vec<Pair>,
}

impl Pairs {
    pub fn new() -> Pairs {
        Pairs::default()
    }

    pub fn update(&mut self, collisions: Vec<CollisionResult>, bodies: &[Body], timestamp: f64) {
        self.collision_start.clear();
        self.collision_active.clear();
        self.collision_end.clear();
        self.collision_end_pairs.clear();

        for collision in collisions {
            let id = pair_id(collision.body_a, collision.body_b);

            if let Some(pair) = self.table.get_mut(&id) {
                if pair.is_active {
                    self.collision_active.push(id);
                }
                pair.update(collision, bodies, timestamp);
            } else {
                let pair = Pair::create(collision, bodies, timestamp);
                self.table.insert(id, pair);
                self.collision_start.push(id);
            }
        }

        // Find pairs that are no longer active
        let mut ended = Vec::new();
        for (id, pair) in &mut self.table {
            if pair.time_updated < timestamp {
                pair.set_active(false, timestamp);
                ended.push(*id);
            }
        }

        for id in &ended {
            self.collision_end.push(*id);
            if let Some(pair) = self.table.remove(id) {
                self.collision_end_pairs.push(pair);
            }
        }
    }

    pub fn clear(&mut self) {
        self.table.clear();
        self.collision_start.clear();
        self.collision_active.clear();
        self.collision_end.clear();
        self.collision_end_pairs.clear();
    }
}

// --- Detector ---

pub struct Detector;

impl Detector {
    pub fn collisions(bodies: &mut [&Body], _pairs: &Pairs) -> Vec<CollisionResult> {
        let mut results = Vec::new();

        // Broadphase: sort by bounds min x
        bodies.sort_by(|a, b| a.bounds.min.x.partial_cmp(&b.bounds.min.x).unwrap());

        let bodies_length = bodies.len();

        for i in 0..bodies_length {
            let body_a = bodies[i];
            let bound_x_max = body_a.bounds.max.x;
            let bound_y_max = body_a.bounds.max.y;
            let bound_y_min = body_a.bounds.min.y;
            let body_a_static = body_a.is_static || body_a.is_sleeping;

            for body_b in &bodies[(i + 1)..] {

                // Broadphase: x-axis sweep
                if body_b.bounds.min.x > bound_x_max {
                    break;
                }

                // Broadphase: y-axis overlap
                if bound_y_max < body_b.bounds.min.y || bound_y_min > body_b.bounds.max.y {
                    continue;
                }

                // Skip static-static / sleeping-sleeping
                if body_a_static && (body_b.is_static || body_b.is_sleeping) {
                    continue;
                }

                // Collision filter
                if !can_collide(&body_a.collision_filter, &body_b.collision_filter) {
                    continue;
                }

                // Narrowphase: SAT
                if let Some(collision) = Collision::collides(body_a, body_b) {
                    results.push(collision);
                }
            }
        }

        results
    }
}

pub fn can_collide(filter_a: &CollisionFilter, filter_b: &CollisionFilter) -> bool {
    if filter_a.group == filter_b.group && filter_a.group != 0 {
        return filter_a.group > 0;
    }
    (filter_a.mask & filter_b.category) != 0 && (filter_b.mask & filter_a.category) != 0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::Body;
    use serde_json::Value;

    const EPSILON: f64 = 1e-14;

    fn load_testdata() -> Value {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/testdata/collision.json");
        let content = std::fs::read_to_string(path).expect("Failed to read collision.json");
        serde_json::from_str(&content).expect("Failed to parse collision.json")
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
    fn collision_two_overlapping_rects() {
        let data = load_testdata();
        let expected = &data["collision_overlap"]["result"];

        let body_a = make_body_at(BodyHandle(0), 0.0, 0.0);
        let body_b = make_body_at(BodyHandle(1), 30.0, 0.0);

        let result = Collision::collides(&body_a, &body_b).expect("should collide");

        assert!(result.collided);
        assert_f64_eq(result.depth, expected["depth"].as_f64().unwrap());
        assert_vec2_eq(&result.normal, &expected["normal"]);
        assert_vec2_eq(&result.tangent, &expected["tangent"]);
        assert_vec2_eq(&result.penetration, &expected["penetration"]);

        let exp_supports = expected["supports"].as_array().unwrap();
        assert_eq!(result.supports.len(), exp_supports.len());
        for (s, es) in result.supports.iter().zip(exp_supports.iter()) {
            assert_vec2_eq(s, es);
        }
    }

    #[test]
    fn collision_no_overlap() {
        let body_a = make_body_at(BodyHandle(0), 0.0, 0.0);
        let body_b = make_body_at(BodyHandle(1), 100.0, 0.0);

        let result = Collision::collides(&body_a, &body_b);
        assert!(result.is_none());
    }

    #[test]
    fn collision_diagonal_overlap() {
        let data = load_testdata();
        let expected = &data["collision_diagonal"]["result"];

        let body_a = make_body_at(BodyHandle(0), 0.0, 0.0);
        let body_b = make_body_at(BodyHandle(1), 15.0, 25.0);

        let result = Collision::collides(&body_a, &body_b).expect("should collide");

        assert_f64_eq(result.depth, expected["depth"].as_f64().unwrap());
        assert_vec2_eq(&result.normal, &expected["normal"]);
        assert_vec2_eq(&result.penetration, &expected["penetration"]);
        assert_eq!(result.supports.len(), expected["supportCount"].as_u64().unwrap() as usize);
    }

    #[test]
    fn can_collide_default_filters() {
        let data = load_testdata();
        let tests = data["can_collide"].as_array().unwrap();

        for (i, test) in tests.iter().enumerate() {
            let fa = CollisionFilter {
                category: test["filterA"]["category"].as_u64().unwrap() as u32,
                mask: test["filterA"]["mask"].as_u64().unwrap() as u32,
                group: test["filterA"]["group"].as_i64().unwrap() as i32,
            };
            let fb = CollisionFilter {
                category: test["filterB"]["category"].as_u64().unwrap() as u32,
                mask: test["filterB"]["mask"].as_u64().unwrap() as u32,
                group: test["filterB"]["group"].as_i64().unwrap() as i32,
            };
            let expected = test["output"].as_bool().unwrap();
            assert_eq!(can_collide(&fa, &fb), expected, "can_collide test {i} failed");
        }
    }

    #[test]
    fn pair_create_from_collision() {
        let bodies = vec![
            make_body_at(BodyHandle(0), 0.0, 0.0),
            make_body_at(BodyHandle(1), 30.0, 0.0),
        ];
        let collision = Collision::collides(&bodies[0], &bodies[1]).unwrap();
        let pair = Pair::create(collision, &bodies, 1.0);

        assert!(pair.is_active);
        assert_eq!(pair.id, (0, 1));
        assert_eq!(pair.contact_count, 2);
        assert_f64_eq(pair.separation, 10.0);
        assert_f64_eq(pair.time_created, 1.0);
        assert_f64_eq(pair.friction, bodies[0].friction.min(bodies[1].friction));
    }

    #[test]
    fn pairs_update_tracks_start_active_end() {
        let bodies = vec![
            make_body_at(BodyHandle(0), 0.0, 0.0),
            make_body_at(BodyHandle(1), 30.0, 0.0),
        ];
        let mut pairs = Pairs::new();

        // Tick 1: collision starts
        let collision = Collision::collides(&bodies[0], &bodies[1]).unwrap();
        pairs.update(vec![collision], &bodies, 1.0);
        assert_eq!(pairs.collision_start.len(), 1);
        assert!(pairs.collision_active.is_empty());
        assert!(pairs.collision_end.is_empty());

        // Tick 2: collision continues (active)
        let collision = Collision::collides(&bodies[0], &bodies[1]).unwrap();
        pairs.update(vec![collision], &bodies, 2.0);
        assert!(pairs.collision_start.is_empty());
        assert_eq!(pairs.collision_active.len(), 1);
        assert!(pairs.collision_end.is_empty());

        // Tick 3: no collision (end)
        pairs.update(vec![], &bodies, 3.0);
        assert!(pairs.collision_start.is_empty());
        assert!(pairs.collision_active.is_empty());
        assert_eq!(pairs.collision_end.len(), 1);

        // Pair removed from table
        assert!(pairs.table.is_empty());
    }

    #[test]
    fn pair_id_is_ordered() {
        assert_eq!(pair_id(BodyHandle(3), BodyHandle(1)), (1, 3));
        assert_eq!(pair_id(BodyHandle(1), BodyHandle(3)), (1, 3));
    }

    #[test]
    fn detector_finds_overlapping_pairs() {
        let bodies = vec![
            make_body_at(BodyHandle(0), 0.0, 0.0),
            make_body_at(BodyHandle(1), 30.0, 0.0),   // overlaps with 0
            make_body_at(BodyHandle(2), 100.0, 0.0),   // no overlap
        ];
        let pairs = Pairs::new();
        let mut refs: Vec<&Body> = bodies.iter().collect();
        let collisions = Detector::collisions(&mut refs, &pairs);

        assert_eq!(collisions.len(), 1);
        assert_eq!(collisions[0].body_a.0.min(collisions[0].body_b.0), 0);
        assert_eq!(collisions[0].body_a.0.max(collisions[0].body_b.0), 1);
    }

    #[test]
    fn detector_skips_static_static() {
        let mut body_a = make_body_at(BodyHandle(0), 0.0, 0.0);
        body_a.set_static(true);
        let mut body_b = make_body_at(BodyHandle(1), 30.0, 0.0);
        body_b.set_static(true);

        let bodies = vec![body_a, body_b];
        let pairs = Pairs::new();
        let mut refs: Vec<&Body> = bodies.iter().collect();
        let collisions = Detector::collisions(&mut refs, &pairs);

        assert!(collisions.is_empty());
    }

    #[test]
    fn detector_respects_collision_filter() {
        let mut body_a = make_body_at(BodyHandle(0), 0.0, 0.0);
        body_a.collision_filter.mask = 0; // can't collide with anything
        let body_b = make_body_at(BodyHandle(1), 30.0, 0.0);

        let bodies = vec![body_a, body_b];
        let pairs = Pairs::new();
        let mut refs: Vec<&Body> = bodies.iter().collect();
        let collisions = Detector::collisions(&mut refs, &pairs);

        assert!(collisions.is_empty());
    }
}
