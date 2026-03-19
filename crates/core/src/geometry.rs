use serde::Deserialize;

#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub fn add(a: &Vec2, b: &Vec2) -> Vec2 {
        Vec2 { x: a.x + b.x, y: a.y + b.y }
    }

    pub fn sub(a: &Vec2, b: &Vec2) -> Vec2 {
        Vec2 { x: a.x - b.x, y: a.y - b.y }
    }

    pub fn mult(v: &Vec2, scalar: f64) -> Vec2 {
        Vec2 { x: v.x * scalar, y: v.y * scalar }
    }

    pub fn div(v: &Vec2, scalar: f64) -> Vec2 {
        Vec2 { x: v.x / scalar, y: v.y / scalar }
    }

    pub fn dot(a: &Vec2, b: &Vec2) -> f64 {
        a.x * b.x + a.y * b.y
    }

    pub fn cross(a: &Vec2, b: &Vec2) -> f64 {
        a.x * b.y - a.y * b.x
    }

    pub fn magnitude(v: &Vec2) -> f64 {
        (v.x * v.x + v.y * v.y).sqrt()
    }

    pub fn normalise(v: &Vec2) -> Vec2 {
        let mag = Self::magnitude(v);
        Vec2 { x: v.x / mag, y: v.y / mag }
    }

    pub fn neg(v: &Vec2) -> Vec2 {
        Vec2 { x: -v.x, y: -v.y }
    }

    pub fn angle(a: &Vec2, b: &Vec2) -> f64 {
        (b.y - a.y).atan2(b.x - a.x)
    }

    pub fn rotate(v: &Vec2, angle: f64) -> Vec2 {
        let cos = angle.cos();
        let sin = angle.sin();
        Vec2 {
            x: v.x * cos - v.y * sin,
            y: v.x * sin + v.y * cos,
        }
    }
}

use std::ops;

impl ops::Add for Vec2 {
    type Output = Vec2;
    fn add(self, rhs: Vec2) -> Vec2 {
        Vec2 { x: self.x + rhs.x, y: self.y + rhs.y }
    }
}

impl ops::Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, rhs: Vec2) -> Vec2 {
        Vec2 { x: self.x - rhs.x, y: self.y - rhs.y }
    }
}

impl ops::Mul<f64> for Vec2 {
    type Output = Vec2;
    fn mul(self, rhs: f64) -> Vec2 {
        Vec2 { x: self.x * rhs, y: self.y * rhs }
    }
}

impl ops::Neg for Vec2 {
    type Output = Vec2;
    fn neg(self) -> Vec2 {
        Vec2 { x: -self.x, y: -self.y }
    }
}

pub struct Vertices;

impl Vertices {
    pub fn area(vertices: &[Vec2], signed: bool) -> f64 {
        let mut area = 0.0;
        let len = vertices.len();
        for i in 0..len {
            let j = (i + 1) % len;
            area += vertices[i].x * vertices[j].y;
            area -= vertices[j].x * vertices[i].y;
        }
        area /= 2.0;
        if signed { area } else { area.abs() }
    }

    pub fn centre(vertices: &[Vec2]) -> Vec2 {
        let area = Self::area(vertices, true);
        let mut centre = Vec2 { x: 0.0, y: 0.0 };
        for i in 0..vertices.len() {
            let j = (i + 1) % vertices.len();
            let cross = Vec2::cross(&vertices[i], &vertices[j]);
            let temp = Vec2::mult(&Vec2::add(&vertices[i], &vertices[j]), cross);
            centre = Vec2::add(&centre, &temp);
        }
        Vec2::div(&centre, 6.0 * area)
    }

    pub fn inertia(vertices: &[Vec2], mass: f64) -> f64 {
        let mut numerator = 0.0;
        let mut denominator = 0.0;
        for n in 0..vertices.len() {
            let j = (n + 1) % vertices.len();
            let cross = Vec2::cross(&vertices[j], &vertices[n]).abs();
            numerator += cross
                * (Vec2::dot(&vertices[j], &vertices[j])
                    + Vec2::dot(&vertices[j], &vertices[n])
                    + Vec2::dot(&vertices[n], &vertices[n]));
            denominator += cross;
        }
        (mass / 6.0) * (numerator / denominator)
    }

    pub fn contains(vertices: &[Vec2], point: &Vec2) -> bool {
        let mut vertex = &vertices[vertices.len() - 1];
        for next_vertex in vertices {
            if (point.x - vertex.x) * (next_vertex.y - vertex.y)
                + (point.y - vertex.y) * (vertex.x - next_vertex.x)
                > 0.0
            {
                return false;
            }
            vertex = next_vertex;
        }
        true
    }

    pub fn translate(vertices: &mut [Vec2], vector: &Vec2, scalar: f64) {
        for v in vertices.iter_mut() {
            v.x += vector.x * scalar;
            v.y += vector.y * scalar;
        }
    }

    pub fn rotate(vertices: &mut [Vec2], angle: f64, point: &Vec2) {
        let cos = angle.cos();
        let sin = angle.sin();
        for v in vertices.iter_mut() {
            let dx = v.x - point.x;
            let dy = v.y - point.y;
            v.x = point.x + (dx * cos - dy * sin);
            v.y = point.y + (dx * sin + dy * cos);
        }
    }

    pub fn scale(vertices: &mut [Vec2], scale_x: f64, scale_y: f64, point: &Vec2) {
        for v in vertices.iter_mut() {
            v.x = point.x + (v.x - point.x) * scale_x;
            v.y = point.y + (v.y - point.y) * scale_y;
        }
    }
}

#[derive(Debug, PartialEq, Deserialize)]
pub struct Bounds {
    pub min: Vec2,
    pub max: Vec2,
}

impl Bounds {
    pub fn create(vertices: &[Vec2]) -> Bounds {
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        for v in vertices {
            if v.x < min_x { min_x = v.x; }
            if v.y < min_y { min_y = v.y; }
            if v.x > max_x { max_x = v.x; }
            if v.y > max_y { max_y = v.y; }
        }
        Bounds {
            min: Vec2 { x: min_x, y: min_y },
            max: Vec2 { x: max_x, y: max_y },
        }
    }

    pub fn update(bounds: &mut Bounds, vertices: &[Vec2], velocity: &Vec2) {
        bounds.min.x = f64::INFINITY;
        bounds.min.y = f64::INFINITY;
        bounds.max.x = f64::NEG_INFINITY;
        bounds.max.y = f64::NEG_INFINITY;
        for v in vertices {
            if v.x < bounds.min.x { bounds.min.x = v.x; }
            if v.y < bounds.min.y { bounds.min.y = v.y; }
            if v.x > bounds.max.x { bounds.max.x = v.x; }
            if v.y > bounds.max.y { bounds.max.y = v.y; }
        }
        if velocity.x > 0.0 {
            bounds.max.x += velocity.x;
        } else {
            bounds.min.x += velocity.x;
        }
        if velocity.y > 0.0 {
            bounds.max.y += velocity.y;
        } else {
            bounds.min.y += velocity.y;
        }
    }

    pub fn contains(bounds: &Bounds, point: &Vec2) -> bool {
        point.x >= bounds.min.x
            && point.x <= bounds.max.x
            && point.y >= bounds.min.y
            && point.y <= bounds.max.y
    }

    pub fn overlaps(bounds_a: &Bounds, bounds_b: &Bounds) -> bool {
        bounds_a.min.x <= bounds_b.max.x
            && bounds_a.max.x >= bounds_b.min.x
            && bounds_a.min.y <= bounds_b.max.y
            && bounds_a.max.y >= bounds_b.min.y
    }
}

pub struct Axes;

impl Axes {
    pub fn from_vertices(vertices: &[Vec2]) -> Vec<Vec2> {
        let mut axes = indexmap::IndexMap::new();
        let len = vertices.len();
        for i in 0..len {
            let j = (i + 1) % len;
            let normal = Vec2::normalise(&Vec2 {
                x: vertices[j].y - vertices[i].y,
                y: vertices[i].x - vertices[j].x,
            });
            let gradient = if normal.y == 0.0 {
                f64::INFINITY
            } else {
                normal.x / normal.y
            };
            // Match Matter.js: -0.000 and 0.000 must be the same key
            let gradient = if gradient == 0.0 { 0.0 } else { gradient };
            let key = format!("{:.3}", gradient);
            axes.insert(key, normal);
        }
        axes.into_values().collect()
    }

    pub fn rotate(axes: &mut [Vec2], angle: f64) {
        if angle == 0.0 {
            return;
        }
        let cos = angle.cos();
        let sin = angle.sin();
        for axis in axes.iter_mut() {
            let xx = axis.x * cos - axis.y * sin;
            axis.y = axis.x * sin + axis.y * cos;
            axis.x = xx;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    fn load_testdata() -> Value {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../testdata/geometry.json");
        let content = std::fs::read_to_string(path).expect("Failed to read geometry.json");
        serde_json::from_str(&content).expect("Failed to parse geometry.json")
    }

    fn vec2(v: &Value) -> Vec2 {
        serde_json::from_value(v.clone()).expect("Failed to parse Vec2")
    }

    fn verts(v: &Value) -> Vec<Vec2> {
        v.as_array().unwrap().iter().map(|v| vec2(v)).collect()
    }

    const EPSILON: f64 = 1e-15;

    fn assert_f64_eq(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < EPSILON,
            "f64 differs: actual {actual} expected {expected}"
        );
    }

    fn assert_vec2_eq(actual: &Vec2, expected: &Vec2) {
        assert!(
            (actual.x - expected.x).abs() < EPSILON && (actual.y - expected.y).abs() < EPSILON,
            "Vec2 differs: actual {:?} expected {:?}", actual, expected
        );
    }

    fn assert_verts_eq(actual: &[Vec2], expected: &[Vec2]) {
        assert_eq!(actual.len(), expected.len(), "different number of vertices");
        for (i, (a, e)) in actual.iter().zip(expected.iter()).enumerate() {
            assert!(
                (a.x - e.x).abs() < EPSILON && (a.y - e.y).abs() < EPSILON,
                "vertex {i} differs: actual {:?} expected {:?}", a, e
            );
        }
    }

    // --- Axes ---

    #[test]
    fn axes_from_vertices() {
        let data = load_testdata();
        for case in data["axes_from_vertices"].as_array().unwrap() {
            let input = verts(&case["vertices"]);
            let expected = verts(&case["output"]);
            assert_verts_eq(&Axes::from_vertices(&input), &expected);
        }
    }

    #[test]
    fn axes_rotate() {
        let data = load_testdata();
        for case in data["axes_rotate"].as_array().unwrap() {
            let mut axes = verts(&case["input_axes"]);
            let angle = case["angle"].as_f64().unwrap();
            Axes::rotate(&mut axes, angle);
            let expected = verts(&case["output"]);
            assert_verts_eq(&axes, &expected);
        }
    }

    // --- Bounds ---

    #[test]
    fn bounds_create() {
        let data = load_testdata();
        for case in data["bounds_create"].as_array().unwrap() {
            let input = verts(&case["vertices"]);
            let expected: Bounds = serde_json::from_value(case["output"].clone()).unwrap();
            assert_eq!(Bounds::create(&input), expected);
        }
    }

    #[test]
    fn bounds_contains() {
        let data = load_testdata();
        for case in data["bounds_contains"].as_array().unwrap() {
            let bounds: Bounds = serde_json::from_value(case["bounds"].clone()).unwrap();
            let point = vec2(&case["point"]);
            let expected = case["output"].as_bool().unwrap();
            assert_eq!(Bounds::contains(&bounds, &point), expected);
        }
    }

    #[test]
    fn bounds_overlaps() {
        let data = load_testdata();
        for case in data["bounds_overlaps"].as_array().unwrap() {
            let a: Bounds = serde_json::from_value(case["boundsA"].clone()).unwrap();
            let b: Bounds = serde_json::from_value(case["boundsB"].clone()).unwrap();
            let expected = case["output"].as_bool().unwrap();
            assert_eq!(Bounds::overlaps(&a, &b), expected);
        }
    }

    // --- Vertices: scale ---

    #[test]
    fn vertices_scale() {
        let data = load_testdata();
        for case in data["vertices_scale"].as_array().unwrap() {
            let mut input = verts(&case["vertices"]);
            let point = vec2(&case["point"]);
            let scale_x = case["scaleX"].as_f64().unwrap();
            let scale_y = case["scaleY"].as_f64().unwrap();
            Vertices::scale(&mut input, scale_x, scale_y, &point);
            assert_verts_eq(&input, &verts(&case["output"]));
        }
    }

    // --- Vertices: rotate ---

    #[test]
    fn vertices_rotate() {
        let data = load_testdata();
        for case in data["vertices_rotate"].as_array().unwrap() {
            let mut input = verts(&case["vertices"]);
            let angle = case["angle"].as_f64().unwrap();
            let point = vec2(&case["point"]);
            Vertices::rotate(&mut input, angle, &point);
            assert_verts_eq(&input, &verts(&case["output"]));
        }
    }

    // --- Vertices: translate ---

    #[test]
    fn vertices_translate() {
        let data = load_testdata();
        for case in data["vertices_translate"].as_array().unwrap() {
            let mut input = verts(&case["vertices"]);
            let vector = vec2(&case["vector"]);
            let scalar = case["scalar"].as_f64().unwrap();
            Vertices::translate(&mut input, &vector, scalar);
            assert_verts_eq(&input, &verts(&case["output"]));
        }
    }

    // --- Vertices: contains ---

    #[test]
    fn vertices_contains() {
        let data = load_testdata();
        for case in data["vertices_contains"].as_array().unwrap() {
            let input = verts(&case["vertices"]);
            let point = vec2(&case["point"]);
            let expected = case["output"].as_bool().unwrap();
            assert_eq!(Vertices::contains(&input, &point), expected);
        }
    }

    // --- Vertices: inertia ---

    #[test]
    fn vertices_inertia() {
        let data = load_testdata();
        for case in data["vertices_inertia"].as_array().unwrap() {
            let input = verts(&case["vertices"]);
            let mass = case["mass"].as_f64().unwrap();
            let expected = case["output"].as_f64().unwrap();
            assert_f64_eq(Vertices::inertia(&input, mass), expected);
        }
    }

    // --- Vertices: area ---

    #[test]
    fn vertices_area() {
        let data = load_testdata();
        for case in data["vertices_area"].as_array().unwrap() {
            let input = verts(&case["vertices"]);
            let signed = case["signed"].as_bool().unwrap();
            let expected = case["output"].as_f64().unwrap();
            assert_f64_eq(Vertices::area(&input, signed), expected);
        }
    }

    // --- Vertices: centre ---

    #[test]
    fn vertices_centre() {
        let data = load_testdata();
        for case in data["vertices_centre"].as_array().unwrap() {
            let input = verts(&case["vertices"]);
            let expected = vec2(&case["output"]);
            assert_vec2_eq(&Vertices::centre(&input), &expected);
        }
    }

    // --- Vector ---

    #[test]
    fn vector_add() {
        let data = load_testdata();
        for case in data["vector_add"].as_array().unwrap() {
            assert_vec2_eq(&Vec2::add(&vec2(&case["a"]), &vec2(&case["b"])), &vec2(&case["output"]));
        }
    }

    #[test]
    fn vector_sub() {
        let data = load_testdata();
        for case in data["vector_sub"].as_array().unwrap() {
            assert_vec2_eq(&Vec2::sub(&vec2(&case["a"]), &vec2(&case["b"])), &vec2(&case["output"]));
        }
    }

    #[test]
    fn vector_mult() {
        let data = load_testdata();
        for case in data["vector_mult"].as_array().unwrap() {
            let scalar = case["scalar"].as_f64().unwrap();
            assert_vec2_eq(&Vec2::mult(&vec2(&case["v"]), scalar), &vec2(&case["output"]));
        }
    }

    #[test]
    fn vector_div() {
        let data = load_testdata();
        for case in data["vector_div"].as_array().unwrap() {
            let scalar = case["scalar"].as_f64().unwrap();
            assert_vec2_eq(&Vec2::div(&vec2(&case["v"]), scalar), &vec2(&case["output"]));
        }
    }

    #[test]
    fn vector_dot() {
        let data = load_testdata();
        for case in data["vector_dot"].as_array().unwrap() {
            let expected = case["output"].as_f64().unwrap();
            assert_f64_eq(Vec2::dot(&vec2(&case["a"]), &vec2(&case["b"])), expected);
        }
    }

    #[test]
    fn vector_cross() {
        let data = load_testdata();
        for case in data["vector_cross"].as_array().unwrap() {
            let expected = case["output"].as_f64().unwrap();
            assert_f64_eq(Vec2::cross(&vec2(&case["a"]), &vec2(&case["b"])), expected);
        }
    }

    #[test]
    fn vector_magnitude() {
        let data = load_testdata();
        for case in data["vector_magnitude"].as_array().unwrap() {
            let expected = case["output"].as_f64().unwrap();
            assert_f64_eq(Vec2::magnitude(&vec2(&case["v"])), expected);
        }
    }

    #[test]
    fn vector_normalise() {
        let data = load_testdata();
        for case in data["vector_normalise"].as_array().unwrap() {
            assert_vec2_eq(&Vec2::normalise(&vec2(&case["v"])), &vec2(&case["output"]));
        }
    }

    #[test]
    fn vector_rotate() {
        let data = load_testdata();
        for case in data["vector_rotate"].as_array().unwrap() {
            let angle = case["angle"].as_f64().unwrap();
            assert_vec2_eq(&Vec2::rotate(&vec2(&case["v"]), angle), &vec2(&case["output"]));
        }
    }

    #[test]
    fn vector_angle() {
        let data = load_testdata();
        for case in data["vector_angle"].as_array().unwrap() {
            let expected = case["output"].as_f64().unwrap();
            assert_f64_eq(Vec2::angle(&vec2(&case["a"]), &vec2(&case["b"])), expected);
        }
    }

    #[test]
    fn vector_neg() {
        let data = load_testdata();
        for case in data["vector_neg"].as_array().unwrap() {
            assert_vec2_eq(&Vec2::neg(&vec2(&case["v"])), &vec2(&case["output"]));
        }
    }

    // --- Operator traits ---

    #[test]
    fn operator_add() {
        let result = Vec2 { x: 1.0, y: 2.0 } + Vec2 { x: 3.0, y: 4.0 };
        assert_eq!(result, Vec2 { x: 4.0, y: 6.0 });
    }

    #[test]
    fn operator_sub() {
        let result = Vec2 { x: 3.0, y: 4.0 } - Vec2 { x: 1.0, y: 2.0 };
        assert_eq!(result, Vec2 { x: 2.0, y: 2.0 });
    }

    #[test]
    fn operator_mul() {
        let result = Vec2 { x: 3.0, y: 4.0 } * 2.0;
        assert_eq!(result, Vec2 { x: 6.0, y: 8.0 });
    }

    #[test]
    fn operator_neg() {
        let result = -Vec2 { x: 3.0, y: -4.0 };
        assert_eq!(result, Vec2 { x: -3.0, y: 4.0 });
    }

    #[test]
    fn create_vec2() {
        let _v = Vec2 { x: 1.0, y: 2.0 };
    }
}
