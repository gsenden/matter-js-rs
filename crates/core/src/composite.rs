use crate::body::BodyHandle;
use crate::constraint::ConstraintHandle;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CompositeHandle(pub usize);

#[derive(Debug, Clone)]
pub struct Composite {
    pub handle: CompositeHandle,
    pub bodies: Vec<BodyHandle>,
    pub constraints: Vec<ConstraintHandle>,
    pub composites: Vec<CompositeHandle>,
    pub parent: Option<CompositeHandle>,
}

impl Composite {
    pub fn new(handle: CompositeHandle) -> Composite {
        Composite {
            handle,
            bodies: Vec::new(),
            constraints: Vec::new(),
            composites: Vec::new(),
            parent: None,
        }
    }

    pub fn add_body(&mut self, body: BodyHandle) {
        self.bodies.push(body);
    }

    pub fn remove_body(&mut self, body: BodyHandle) {
        self.bodies.retain(|b| *b != body);
    }

    pub fn add_constraint(&mut self, constraint: ConstraintHandle) {
        self.constraints.push(constraint);
    }

    pub fn remove_constraint(&mut self, constraint: ConstraintHandle) {
        self.constraints.retain(|c| *c != constraint);
    }

    pub fn add_composite(&mut self, composite: CompositeHandle) {
        self.composites.push(composite);
    }

    pub fn remove_composite(&mut self, composite: CompositeHandle) {
        self.composites.retain(|c| *c != composite);
    }

    pub fn all_bodies(&self, composites: &[Composite]) -> Vec<BodyHandle> {
        let mut bodies = self.bodies.clone();
        for &child_handle in &self.composites {
            if let Some(child) = composites.iter().find(|c| c.handle == child_handle) {
                bodies.extend(child.all_bodies(composites));
            }
        }
        bodies
    }

    pub fn all_constraints(&self, composites: &[Composite]) -> Vec<ConstraintHandle> {
        let mut constraints = self.constraints.clone();
        for &child_handle in &self.composites {
            if let Some(child) = composites.iter().find(|c| c.handle == child_handle) {
                constraints.extend(child.all_constraints(composites));
            }
        }
        constraints
    }

    pub fn clear(&mut self) {
        self.bodies.clear();
        self.constraints.clear();
        self.composites.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn composite_new_is_empty() {
        let composite = Composite::new(CompositeHandle(0));
        assert!(composite.bodies.is_empty());
        assert!(composite.constraints.is_empty());
        assert!(composite.composites.is_empty());
        assert_eq!(composite.parent, None);
    }

    #[test]
    fn composite_add_remove_body() {
        let mut composite = Composite::new(CompositeHandle(0));
        composite.add_body(BodyHandle(1));
        composite.add_body(BodyHandle(2));

        assert_eq!(composite.bodies.len(), 2);
        assert_eq!(composite.bodies[0], BodyHandle(1));

        composite.remove_body(BodyHandle(1));
        assert_eq!(composite.bodies.len(), 1);
        assert_eq!(composite.bodies[0], BodyHandle(2));
    }

    #[test]
    fn composite_add_remove_constraint() {
        let mut composite = Composite::new(CompositeHandle(0));
        composite.add_constraint(ConstraintHandle(1));
        composite.add_constraint(ConstraintHandle(2));

        assert_eq!(composite.constraints.len(), 2);

        composite.remove_constraint(ConstraintHandle(1));
        assert_eq!(composite.constraints.len(), 1);
        assert_eq!(composite.constraints[0], ConstraintHandle(2));
    }

    #[test]
    fn composite_add_remove_sub_composite() {
        let mut composite = Composite::new(CompositeHandle(0));
        composite.add_composite(CompositeHandle(1));

        assert_eq!(composite.composites.len(), 1);

        composite.remove_composite(CompositeHandle(1));
        assert!(composite.composites.is_empty());
    }

    #[test]
    fn composite_all_bodies_flat() {
        let mut composite = Composite::new(CompositeHandle(0));
        composite.add_body(BodyHandle(1));
        composite.add_body(BodyHandle(2));

        let all = composite.all_bodies(&[]);
        assert_eq!(all, vec![BodyHandle(1), BodyHandle(2)]);
    }

    #[test]
    fn composite_all_bodies_recursive() {
        let mut parent = Composite::new(CompositeHandle(0));
        parent.add_body(BodyHandle(1));
        parent.add_composite(CompositeHandle(1));

        let mut child = Composite::new(CompositeHandle(1));
        child.add_body(BodyHandle(2));
        child.add_body(BodyHandle(3));

        let composites = vec![parent.clone(), child];
        let all = composites[0].all_bodies(&composites);
        assert_eq!(all, vec![BodyHandle(1), BodyHandle(2), BodyHandle(3)]);
    }

    #[test]
    fn composite_all_constraints_recursive() {
        let mut parent = Composite::new(CompositeHandle(0));
        parent.add_constraint(ConstraintHandle(1));
        parent.add_composite(CompositeHandle(1));

        let mut child = Composite::new(CompositeHandle(1));
        child.add_constraint(ConstraintHandle(2));

        let composites = vec![parent.clone(), child];
        let all = composites[0].all_constraints(&composites);
        assert_eq!(all, vec![ConstraintHandle(1), ConstraintHandle(2)]);
    }

    #[test]
    fn composite_clear() {
        let mut composite = Composite::new(CompositeHandle(0));
        composite.add_body(BodyHandle(1));
        composite.add_constraint(ConstraintHandle(1));
        composite.add_composite(CompositeHandle(1));

        composite.clear();
        assert!(composite.bodies.is_empty());
        assert!(composite.constraints.is_empty());
        assert!(composite.composites.is_empty());
    }
}
