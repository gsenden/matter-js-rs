const Matter = require('matter-js');
const fs = require('fs');
const path = require('path');

const { Vector, Vertices, Bounds, Axes, Body, Bodies, Composite, Engine, Collision, Detector, Constraint } = Matter;

function generateGeometry() {
    const data = {};

    // --- Vector ---

    data.vector_add = [
        { a: { x: 1, y: 2 }, b: { x: 3, y: 4 }, output: Vector.add({ x: 1, y: 2 }, { x: 3, y: 4 }) },
        { a: { x: -1, y: 0 }, b: { x: 0, y: -1 }, output: Vector.add({ x: -1, y: 0 }, { x: 0, y: -1 }) },
        { a: { x: 0, y: 0 }, b: { x: 0, y: 0 }, output: Vector.add({ x: 0, y: 0 }, { x: 0, y: 0 }) },
    ];

    data.vector_sub = [
        { a: { x: 3, y: 4 }, b: { x: 1, y: 2 }, output: Vector.sub({ x: 3, y: 4 }, { x: 1, y: 2 }) },
        { a: { x: 0, y: 0 }, b: { x: 5, y: 5 }, output: Vector.sub({ x: 0, y: 0 }, { x: 5, y: 5 }) },
    ];

    data.vector_mult = [
        { v: { x: 3, y: 4 }, scalar: 2, output: Vector.mult({ x: 3, y: 4 }, 2) },
        { v: { x: 1, y: -1 }, scalar: 0, output: Vector.mult({ x: 1, y: -1 }, 0) },
        { v: { x: 2, y: 3 }, scalar: -1.5, output: Vector.mult({ x: 2, y: 3 }, -1.5) },
    ];

    data.vector_div = [
        { v: { x: 6, y: 4 }, scalar: 2, output: Vector.div({ x: 6, y: 4 }, 2) },
        { v: { x: 3, y: 9 }, scalar: 3, output: Vector.div({ x: 3, y: 9 }, 3) },
    ];

    data.vector_dot = [
        { a: { x: 1, y: 0 }, b: { x: 0, y: 1 }, output: Vector.dot({ x: 1, y: 0 }, { x: 0, y: 1 }) },
        { a: { x: 3, y: 4 }, b: { x: 4, y: 3 }, output: Vector.dot({ x: 3, y: 4 }, { x: 4, y: 3 }) },
    ];

    data.vector_cross = [
        { a: { x: 1, y: 0 }, b: { x: 0, y: 1 }, output: Vector.cross({ x: 1, y: 0 }, { x: 0, y: 1 }) },
        { a: { x: 3, y: 4 }, b: { x: 4, y: 3 }, output: Vector.cross({ x: 3, y: 4 }, { x: 4, y: 3 }) },
    ];

    data.vector_magnitude = [
        { v: { x: 3, y: 4 }, output: Vector.magnitude({ x: 3, y: 4 }) },
        { v: { x: 0, y: 0 }, output: Vector.magnitude({ x: 0, y: 0 }) },
        { v: { x: 1, y: 1 }, output: Vector.magnitude({ x: 1, y: 1 }) },
    ];

    data.vector_normalise = [
        { v: { x: 3, y: 4 }, output: Vector.normalise({ x: 3, y: 4 }) },
        { v: { x: 0, y: 5 }, output: Vector.normalise({ x: 0, y: 5 }) },
    ];

    data.vector_rotate = [
        { v: { x: 1, y: 0 }, angle: Math.PI / 2, output: Vector.rotate({ x: 1, y: 0 }, Math.PI / 2) },
        { v: { x: 1, y: 0 }, angle: Math.PI, output: Vector.rotate({ x: 1, y: 0 }, Math.PI) },
        { v: { x: 3, y: 4 }, angle: 0, output: Vector.rotate({ x: 3, y: 4 }, 0) },
    ];

    data.vector_angle = [
        { a: { x: 0, y: 0 }, b: { x: 1, y: 0 }, output: Vector.angle({ x: 0, y: 0 }, { x: 1, y: 0 }) },
        { a: { x: 0, y: 0 }, b: { x: 0, y: 1 }, output: Vector.angle({ x: 0, y: 0 }, { x: 0, y: 1 }) },
        { a: { x: 0, y: 0 }, b: { x: -1, y: 0 }, output: Vector.angle({ x: 0, y: 0 }, { x: -1, y: 0 }) },
    ];

    data.vector_neg = [
        { v: { x: 3, y: -4 }, output: Vector.neg({ x: 3, y: -4 }) },
        { v: { x: 0, y: 0 }, output: Vector.neg({ x: 0, y: 0 }) },
    ];

    // --- Vertices ---

    const square = [{ x: 0, y: 0 }, { x: 10, y: 0 }, { x: 10, y: 10 }, { x: 0, y: 10 }];
    const triangle = [{ x: 0, y: 0 }, { x: 10, y: 0 }, { x: 5, y: 10 }];

    data.vertices_area = [
        { vertices: square, signed: false, output: Vertices.area(square, false) },
        { vertices: triangle, signed: false, output: Vertices.area(triangle, false) },
        { vertices: square, signed: true, output: Vertices.area(square, true) },
    ];

    data.vertices_centre = [
        { vertices: square, output: Vertices.centre(square) },
        { vertices: triangle, output: Vertices.centre(triangle) },
    ];

    data.vertices_inertia = [
        { vertices: square, mass: 1, output: Vertices.inertia(square, 1) },
        { vertices: square, mass: 10, output: Vertices.inertia(square, 10) },
        { vertices: triangle, mass: 1, output: Vertices.inertia(triangle, 1) },
    ];

    data.vertices_contains = [
        { vertices: square, point: { x: 5, y: 5 }, output: Vertices.contains(square, { x: 5, y: 5 }) },
        { vertices: square, point: { x: 15, y: 5 }, output: Vertices.contains(square, { x: 15, y: 5 }) },
        { vertices: triangle, point: { x: 5, y: 5 }, output: Vertices.contains(triangle, { x: 5, y: 5 }) },
    ];

    // Vertices.rotate mutates in place, so we clone
    function rotateTest(verts, angle, point) {
        const clone = verts.map(v => ({ x: v.x, y: v.y }));
        Vertices.rotate(clone, angle, point);
        return clone;
    }

    data.vertices_rotate = [
        { vertices: square, angle: Math.PI / 4, point: { x: 5, y: 5 },
          output: rotateTest(square, Math.PI / 4, { x: 5, y: 5 }) },
        { vertices: triangle, angle: Math.PI / 2, point: { x: 0, y: 0 },
          output: rotateTest(triangle, Math.PI / 2, { x: 0, y: 0 }) },
    ];

    function translateTest(verts, vector, scalar) {
        const clone = verts.map(v => ({ x: v.x, y: v.y }));
        Vertices.translate(clone, vector, scalar);
        return clone;
    }

    data.vertices_translate = [
        { vertices: square, vector: { x: 5, y: 3 }, scalar: 1,
          output: translateTest(square, { x: 5, y: 3 }, 1) },
        { vertices: triangle, vector: { x: 1, y: 1 }, scalar: 2,
          output: translateTest(triangle, { x: 1, y: 1 }, 2) },
    ];

    function scaleTest(verts, scaleX, scaleY, point) {
        const clone = verts.map(v => ({ x: v.x, y: v.y }));
        Vertices.scale(clone, scaleX, scaleY, point);
        return clone;
    }

    data.vertices_scale = [
        { vertices: square, scaleX: 2, scaleY: 2, point: { x: 5, y: 5 },
          output: scaleTest(square, 2, 2, { x: 5, y: 5 }) },
        { vertices: triangle, scaleX: 0.5, scaleY: 1.5, point: { x: 0, y: 0 },
          output: scaleTest(triangle, 0.5, 1.5, { x: 0, y: 0 }) },
    ];

    // --- Bounds ---

    const squareBounds = Bounds.create(square);

    data.bounds_create = [
        { vertices: square, output: Bounds.create(square) },
        { vertices: triangle, output: Bounds.create(triangle) },
    ];

    data.bounds_contains = [
        { bounds: squareBounds, point: { x: 5, y: 5 }, output: Bounds.contains(squareBounds, { x: 5, y: 5 }) },
        { bounds: squareBounds, point: { x: 15, y: 5 }, output: Bounds.contains(squareBounds, { x: 15, y: 5 }) },
    ];

    const otherBounds = Bounds.create([{ x: 5, y: 5 }, { x: 15, y: 15 }]);
    const farBounds = Bounds.create([{ x: 20, y: 20 }, { x: 30, y: 30 }]);

    data.bounds_overlaps = [
        { boundsA: squareBounds, boundsB: otherBounds, output: Bounds.overlaps(squareBounds, otherBounds) },
        { boundsA: squareBounds, boundsB: farBounds, output: Bounds.overlaps(squareBounds, farBounds) },
    ];

    // --- Axes ---

    data.axes_from_vertices = [
        { vertices: square, output: Axes.fromVertices(square) },
        { vertices: triangle, output: Axes.fromVertices(triangle) },
    ];

    function rotateAxesTest(verts, angle) {
        // Deep clone to avoid mutating shared state
        const axes = JSON.parse(JSON.stringify(Axes.fromVertices(verts)));
        Axes.rotate(axes, angle);
        return axes;
    }

    data.axes_rotate = [
        { vertices: square, angle: Math.PI / 4,
          input_axes: Axes.fromVertices(square),
          output: rotateAxesTest(square, Math.PI / 4) },
        { vertices: triangle, angle: Math.PI / 2,
          input_axes: Axes.fromVertices(triangle),
          output: rotateAxesTest(triangle, Math.PI / 2) },
    ];

    return data;
}

function serializeBody(b) {
    return {
        id: b.id,
        position: { x: b.position.x, y: b.position.y },
        positionPrev: b.positionPrev ? { x: b.positionPrev.x, y: b.positionPrev.y } : null,
        velocity: { x: b.velocity.x, y: b.velocity.y },
        force: { x: b.force.x, y: b.force.y },
        torque: b.torque,
        positionImpulse: { x: b.positionImpulse.x, y: b.positionImpulse.y },
        constraintImpulse: { x: b.constraintImpulse.x, y: b.constraintImpulse.y, angle: b.constraintImpulse.angle },
        angle: b.angle,
        anglePrev: b.anglePrev,
        angularVelocity: b.angularVelocity,
        speed: b.speed,
        angularSpeed: b.angularSpeed,
        mass: b.mass,
        inverseMass: b.inverseMass,
        inertia: b.inertia,
        inverseInertia: b.inverseInertia,
        density: b.density,
        area: b.area,
        isStatic: b.isStatic,
        isSensor: b.isSensor,
        isSleeping: b.isSleeping,
        motion: b.motion,
        sleepThreshold: b.sleepThreshold,
        timeScale: b.timeScale,
        friction: b.friction,
        frictionStatic: b.frictionStatic,
        frictionAir: b.frictionAir,
        restitution: b.restitution,
        slop: b.slop,
        collisionFilter: {
            category: b.collisionFilter.category,
            mask: b.collisionFilter.mask,
            group: b.collisionFilter.group,
        },
        parts: b.parts.map(p => p.id),
        parent: b.parent ? b.parent.id : b.id,
        vertices: b.vertices.map(v => ({ x: v.x, y: v.y })),
        axes: b.axes ? b.axes.map(a => ({ x: a.x, y: a.y })) : null,
        bounds: b.bounds ? { min: { x: b.bounds.min.x, y: b.bounds.min.y }, max: { x: b.bounds.max.x, y: b.bounds.max.y } } : null,
        circleRadius: b.circleRadius,
        deltaTime: b.deltaTime,
    };
}

function generateBody() {
    const data = {};

    // Default body (40x40 rectangle at origin)
    const defaultBody = Body.create({});
    data.body_defaults = serializeBody(defaultBody);

    // Circle
    const circle = Bodies.circle(100, 200, 25);
    data.body_circle = serializeBody(circle);

    // Rectangle
    const rect = Bodies.rectangle(50, 50, 80, 40);
    data.body_rectangle = serializeBody(rect);

    // Static body
    const staticBody = Bodies.rectangle(400, 600, 800, 50, { isStatic: true });
    data.body_static = serializeBody(staticBody);

    // Compound body (two parts)
    const partA = Bodies.rectangle(0, 0, 40, 40);
    const partB = Bodies.rectangle(40, 0, 40, 40);
    const compound = Body.create({ parts: [partA, partB] });
    data.body_compound = {
        parent: serializeBody(compound),
        parts: compound.parts.map(p => serializeBody(p)),
    };

    // Body.update() — single tick with gravity
    const updateBody = Body.create({});
    // Apply gravity as force (like Engine does: body.force.y += body.mass * gravity.y)
    const gravity = { x: 0, y: 1 };
    updateBody.force.x += updateBody.mass * gravity.x;
    updateBody.force.y += updateBody.mass * gravity.y;
    const beforeUpdate = serializeBody(updateBody);
    Body.update(updateBody, 1000 / 60);
    Body.updateVelocities(updateBody);
    const afterUpdate = serializeBody(updateBody);
    data.body_update_gravity = {
        before: beforeUpdate,
        after: afterUpdate,
        gravity: gravity,
        deltaTime: 1000 / 60,
    };

    // Body.update() — body with initial velocity (positionPrev offset)
    const movingBody = Body.create({});
    Body.setVelocity(movingBody, { x: 2, y: -3 });
    movingBody.force.x += movingBody.mass * gravity.x;
    movingBody.force.y += movingBody.mass * gravity.y;
    const beforeMoving = serializeBody(movingBody);
    Body.update(movingBody, 1000 / 60);
    Body.updateVelocities(movingBody);
    const afterMoving = serializeBody(movingBody);
    data.body_update_moving = {
        before: beforeMoving,
        after: afterMoving,
        gravity: gravity,
        deltaTime: 1000 / 60,
    };

    // Body.update() — multiple ticks (3 ticks of gravity)
    const fallingBody = Body.create({});
    const ticks = [];
    for (let i = 0; i < 3; i++) {
        fallingBody.force.x = fallingBody.mass * gravity.x;
        fallingBody.force.y = fallingBody.mass * gravity.y;
        Body.update(fallingBody, 1000 / 60);
        Body.updateVelocities(fallingBody);
        ticks.push(serializeBody(fallingBody));
    }
    data.body_update_3ticks = {
        gravity: gravity,
        deltaTime: 1000 / 60,
        ticks: ticks,
    };

    // --- apply_force ---
    const forceBody = Body.create({});
    Body.applyForce(forceBody, { x: 20, y: 20 }, { x: 0.05, y: -0.1 });
    data.body_apply_force = {
        force: { x: forceBody.force.x, y: forceBody.force.y },
        torque: forceBody.torque,
    };

    // --- set_static ---
    const staticSetBody = Body.create({});
    const beforeStatic = serializeBody(staticSetBody);
    Body.setStatic(staticSetBody, true);
    data.body_set_static = {
        before: beforeStatic,
        after: serializeBody(staticSetBody),
    };

    // --- set_position ---
    const posBody = Body.create({});
    Body.setPosition(posBody, { x: 100, y: 200 });
    data.body_set_position = serializeBody(posBody);

    const posBodyVel = Body.create({});
    Body.setPosition(posBodyVel, { x: 100, y: 200 }, true);
    data.body_set_position_update_velocity = serializeBody(posBodyVel);

    // --- set_angle ---
    const angleBody = Body.create({});
    Body.setAngle(angleBody, Math.PI / 4);
    data.body_set_angle = serializeBody(angleBody);

    const angleBodyVel = Body.create({});
    Body.setAngle(angleBodyVel, Math.PI / 4, true);
    data.body_set_angle_update_velocity = serializeBody(angleBodyVel);

    // --- set_velocity ---
    const velBody = Body.create({});
    Body.setVelocity(velBody, { x: 5, y: -3 });
    data.body_set_velocity = serializeBody(velBody);

    // --- translate ---
    const translateBody = Body.create({});
    Body.translate(translateBody, { x: 30, y: -15 });
    data.body_translate = serializeBody(translateBody);

    // --- rotate (around own position) ---
    const rotateBody = Body.create({});
    Body.rotate(rotateBody, Math.PI / 6);
    data.body_rotate = serializeBody(rotateBody);

    // --- rotate (around external point) ---
    const rotatePointBody = Body.create({});
    Body.rotate(rotatePointBody, Math.PI / 3, { x: 50, y: 50 });
    data.body_rotate_point = serializeBody(rotatePointBody);

    // --- scale ---
    const scaleBody = Body.create({});
    Body.scale(scaleBody, 2, 1.5);
    data.body_scale = serializeBody(scaleBody);

    return data;
}

function generateCollision() {
    const data = {};

    // Two overlapping rectangles
    const bodyA = Bodies.rectangle(0, 0, 40, 40);
    const bodyB = Bodies.rectangle(30, 0, 40, 40);
    const collision = Collision.collides(bodyA, bodyB);
    data.collision_overlap = {
        bodyA: serializeBody(bodyA),
        bodyB: serializeBody(bodyB),
        result: collision ? {
            collided: collision.collided,
            depth: collision.depth,
            normal: { x: collision.normal.x, y: collision.normal.y },
            tangent: { x: collision.tangent.x, y: collision.tangent.y },
            penetration: { x: collision.penetration.x, y: collision.penetration.y },
            supports: collision.supports.slice(0, collision.supportCount).map(s => ({ x: s.x, y: s.y })),
            supportCount: collision.supportCount,
        } : null,
    };

    // Two non-overlapping rectangles
    const bodyC = Bodies.rectangle(0, 0, 40, 40);
    const bodyD = Bodies.rectangle(100, 0, 40, 40);
    const noCollision = Collision.collides(bodyC, bodyD);
    data.collision_no_overlap = {
        bodyA: serializeBody(bodyC),
        bodyB: serializeBody(bodyD),
        result: noCollision,
    };

    // Partial vertical overlap
    const bodyE = Bodies.rectangle(0, 0, 40, 40);
    const bodyF = Bodies.rectangle(15, 25, 40, 40);
    const diagonalCollision = Collision.collides(bodyE, bodyF);
    data.collision_diagonal = {
        bodyA: serializeBody(bodyE),
        bodyB: serializeBody(bodyF),
        result: diagonalCollision ? {
            collided: diagonalCollision.collided,
            depth: diagonalCollision.depth,
            normal: { x: diagonalCollision.normal.x, y: diagonalCollision.normal.y },
            tangent: { x: diagonalCollision.tangent.x, y: diagonalCollision.tangent.y },
            penetration: { x: diagonalCollision.penetration.x, y: diagonalCollision.penetration.y },
            supports: diagonalCollision.supports.slice(0, diagonalCollision.supportCount).map(s => ({ x: s.x, y: s.y })),
            supportCount: diagonalCollision.supportCount,
        } : null,
    };

    // canCollide filter tests
    data.can_collide = [
        {
            filterA: { category: 1, mask: 0xFFFFFFFF, group: 0 },
            filterB: { category: 1, mask: 0xFFFFFFFF, group: 0 },
            output: Detector.canCollide({ category: 1, mask: 0xFFFFFFFF, group: 0 }, { category: 1, mask: 0xFFFFFFFF, group: 0 }),
        },
        {
            filterA: { category: 1, mask: 2, group: 0 },
            filterB: { category: 2, mask: 1, group: 0 },
            output: Detector.canCollide({ category: 1, mask: 2, group: 0 }, { category: 2, mask: 1, group: 0 }),
        },
        {
            filterA: { category: 1, mask: 0, group: 0 },
            filterB: { category: 1, mask: 0xFFFFFFFF, group: 0 },
            output: Detector.canCollide({ category: 1, mask: 0, group: 0 }, { category: 1, mask: 0xFFFFFFFF, group: 0 }),
        },
        {
            filterA: { category: 1, mask: 0xFFFFFFFF, group: 1 },
            filterB: { category: 1, mask: 0xFFFFFFFF, group: 1 },
            output: Detector.canCollide({ category: 1, mask: 0xFFFFFFFF, group: 1 }, { category: 1, mask: 0xFFFFFFFF, group: 1 }),
        },
        {
            filterA: { category: 1, mask: 0xFFFFFFFF, group: -1 },
            filterB: { category: 1, mask: 0xFFFFFFFF, group: -1 },
            output: Detector.canCollide({ category: 1, mask: 0xFFFFFFFF, group: -1 }, { category: 1, mask: 0xFFFFFFFF, group: -1 }),
        },
    ];

    return data;
}

function generateConstraint() {
    const data = {};

    // Two bodies connected by a rigid constraint
    const bodyA = Bodies.rectangle(0, 0, 40, 40);
    const bodyB = Bodies.rectangle(100, 0, 40, 40);
    const constraint = Constraint.create({
        bodyA: bodyA,
        bodyB: bodyB,
    });
    data.constraint_defaults = {
        length: constraint.length,
        stiffness: constraint.stiffness,
        damping: constraint.damping,
        angularStiffness: constraint.angularStiffness,
        pointA: { x: constraint.pointA.x, y: constraint.pointA.y },
        pointB: { x: constraint.pointB.x, y: constraint.pointB.y },
        angleA: constraint.angleA,
        angleB: constraint.angleB,
    };

    // Solve a constraint: two bodies pulled together
    const solveA = Bodies.rectangle(0, 0, 40, 40);
    const solveB = Bodies.rectangle(60, 0, 40, 40);
    const solveConstraint = Constraint.create({
        bodyA: solveA,
        bodyB: solveB,
        length: 40, // shorter than distance (60), so should pull together
        stiffness: 1,
    });
    const beforeA = serializeBody(solveA);
    const beforeB = serializeBody(solveB);
    Constraint.solve(solveConstraint, 1);
    data.constraint_solve = {
        beforeA: beforeA,
        beforeB: beforeB,
        afterA: serializeBody(solveA),
        afterB: serializeBody(solveB),
        length: solveConstraint.length,
        stiffness: solveConstraint.stiffness,
    };

    // Spring constraint (low stiffness)
    const springA = Bodies.rectangle(0, 0, 40, 40);
    const springB = Bodies.rectangle(100, 0, 40, 40);
    const spring = Constraint.create({
        bodyA: springA,
        bodyB: springB,
        stiffness: 0.1,
        damping: 0.05,
    });
    Constraint.solve(spring, 1);
    data.constraint_spring = {
        afterA: serializeBody(springA),
        afterB: serializeBody(springB),
    };

    // Pin constraint (one body pinned to world point)
    const pinBody = Bodies.rectangle(50, 50, 40, 40);
    const pin = Constraint.create({
        bodyA: pinBody,
        pointB: { x: 0, y: 0 },
    });
    data.constraint_pin = {
        length: pin.length,
        stiffness: pin.stiffness,
        pointA: { x: pin.pointA.x, y: pin.pointA.y },
        pointB: { x: pin.pointB.x, y: pin.pointB.y },
    };

    return data;
}

function serializePair(pair) {
    return {
        id: pair.id,
        bodyA: pair.bodyA.id,
        bodyB: pair.bodyB.id,
        isActive: pair.isActive,
        isSensor: pair.isSensor,
        contactCount: pair.contactCount,
        separation: pair.separation,
        inverseMass: pair.inverseMass,
        friction: pair.friction,
        frictionStatic: pair.frictionStatic,
        restitution: pair.restitution,
        slop: pair.slop,
        collision: {
            depth: pair.collision.depth,
            normal: { x: pair.collision.normal.x, y: pair.collision.normal.y },
            tangent: { x: pair.collision.tangent.x, y: pair.collision.tangent.y },
        },
        contacts: pair.contacts.slice(0, pair.contactCount).map(c => ({
            vertex: { x: c.vertex.x, y: c.vertex.y },
            normalImpulse: c.normalImpulse,
            tangentImpulse: c.tangentImpulse,
        })),
    };
}

function generateEngine() {
    const data = {};

    // --- Scenario 1: single body freefall (1 tick) ---
    {
        const engine = Engine.create();
        const body = Bodies.rectangle(0, 0, 40, 40);
        Composite.add(engine.world, body);
        Engine.update(engine, 1000 / 60);
        data.engine_freefall_1tick = {
            bodies: [serializeBody(body)],
        };
    }

    // --- Scenario 2: freefall 10 ticks ---
    {
        const engine = Engine.create();
        const body = Bodies.rectangle(0, 0, 40, 40);
        Composite.add(engine.world, body);
        const ticks = [];
        for (let i = 0; i < 10; i++) {
            Engine.update(engine, 1000 / 60);
            ticks.push(serializeBody(body));
        }
        data.engine_freefall_10ticks = { ticks };
    }

    // --- Scenario 3: body lands on static floor ---
    {
        const engine = Engine.create();
        const body = Bodies.rectangle(0, 0, 40, 40);
        const floor = Bodies.rectangle(0, 100, 200, 40, { isStatic: true });
        Composite.add(engine.world, [body, floor]);
        const ticks = [];
        let collisionStartTick = null;
        Matter.Events.on(engine, 'collisionStart', (e) => {
            if (collisionStartTick === null) {
                collisionStartTick = ticks.length;
            }
        });
        for (let i = 0; i < 60; i++) {
            Engine.update(engine, 1000 / 60);
            ticks.push(serializeBody(body));
        }
        data.engine_floor_collision = {
            ticks,
            floor: serializeBody(floor),
            collisionStartTick,
        };
    }

    // --- Scenario 4: two dynamic bodies collide head-on ---
    {
        const engine = Engine.create();
        engine.gravity.y = 0; // no gravity
        const bodyA = Bodies.rectangle(-50, 0, 40, 40);
        const bodyB = Bodies.rectangle(50, 0, 40, 40);
        Body.setVelocity(bodyA, { x: 5, y: 0 });
        Body.setVelocity(bodyB, { x: -5, y: 0 });
        Composite.add(engine.world, [bodyA, bodyB]);
        const ticks = [];
        for (let i = 0; i < 20; i++) {
            Engine.update(engine, 1000 / 60);
            ticks.push({
                bodyA: serializeBody(bodyA),
                bodyB: serializeBody(bodyB),
            });
        }
        data.engine_head_on = { ticks };
    }

    // --- Scenario 5: body with constraint ---
    {
        const engine = Engine.create();
        const body = Bodies.rectangle(0, 0, 40, 40);
        const constraint = Constraint.create({
            bodyA: body,
            pointB: { x: 0, y: 0 },
            stiffness: 0.5,
        });
        Composite.add(engine.world, [body, constraint]);
        const ticks = [];
        for (let i = 0; i < 30; i++) {
            Engine.update(engine, 1000 / 60);
            ticks.push(serializeBody(body));
        }
        data.engine_constraint = { ticks };
    }

    return data;
}

// --- Main ---

const geometry = generateGeometry();
const body = generateBody();
const collision = generateCollision();
const constraintData = generateConstraint();
const engineData = generateEngine();
fs.writeFileSync(
    path.join(__dirname, 'geometry.json'),
    JSON.stringify(geometry, null, 2)
);
fs.writeFileSync(
    path.join(__dirname, 'body.json'),
    JSON.stringify(body, null, 2)
);
fs.writeFileSync(
    path.join(__dirname, 'collision.json'),
    JSON.stringify(collision, null, 2)
);

fs.writeFileSync(
    path.join(__dirname, 'constraint.json'),
    JSON.stringify(constraintData, null, 2)
);
fs.writeFileSync(
    path.join(__dirname, 'engine.json'),
    JSON.stringify(engineData, null, 2)
);

console.log('Generated: testdata/geometry.json');
console.log('Generated: testdata/body.json');
console.log('Generated: testdata/collision.json');
console.log('Generated: testdata/constraint.json');
console.log('Generated: testdata/engine.json');
