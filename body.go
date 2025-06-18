package cm

import (
	"fmt"
	"log"
	"math"
	"slices"

	"github.com/setanarut/v"
)

// BodyType for bodies; Dynamic, Kinematic or Static
type BodyType uint8

const (
	Dynamic   BodyType = 0
	Kinematic BodyType = 1
	Static    BodyType = 2
)

var bodyCur int = 0

// BodyVelocityFunc is rigid body velocity update function type.
type BodyVelocityFunc func(body *Body, gravity v.Vec, damping float64, dt float64)

// BodyPositionFunc is rigid body position update function type.
type BodyPositionFunc func(body *Body, dt float64)

type Body struct {
	// UserData is an object that this constraint is associated with.
	//
	// You can use this get a reference to your game object or controller object from within callbacks.
	UserData any
	Space    *Space

	id                     int // Body id
	Shapes                 []*Shape
	velocityFunc           BodyVelocityFunc // Integration function
	positionFunc           BodyPositionFunc // Integration function
	mass                   float64          // Mass
	massInverse            float64          // Mass inverse
	momentOfInertia        float64          // Moment of inertia
	momentOfInertiaInverse float64          // Inverse of moment of inertia i
	angle                  float64          // Angle (radians)
	w                      float64          // Angular velocity,
	torque                 float64          // Torque (radians)
	centerOfGravity        v.Vec            // Center of gravity
	position               v.Vec            // Position
	velocity               v.Vec            // Velocity
	force                  v.Vec            // Force
	transform              Transform
	arbiterList            *Arbiter
	constraintList         *Constraint
	sleepingRoot           *Body
	sleepingNext           *Body
	sleepingIdleTime       float64
	vBias                  v.Vec   // "pseudo-velocities" used for eliminating overlap. (Erin Catto)
	wBias                  float64 // "pseudo-velocities" used for eliminating overlap. (Erin Catto)
}

// String returns body id as string
func (b Body) String() string {
	return fmt.Sprint("Body ", b.id, ", Shapes ", b.Shapes)
}

// ShapeAtIndex returns shape at index attached to this body
func (b *Body) ShapeAtIndex(index int) *Shape {
	return b.Shapes[index]
}

// NewBody Initializes a rigid body with the given mass and moment of inertia.
//
// Guessing the moment of inertia is usually a bad idea. Use the moment estimation functions MomentFor*().
func NewBody(mass, moment float64) *Body {
	body := &Body{
		id:           bodyCur,
		transform:    NewTransformIdentity(),
		velocityFunc: BodyUpdateVelocity,
		positionFunc: BodyUpdatePosition,
	}
	bodyCur++
	body.SetMass(mass)
	body.SetMoment(moment)
	body.SetAngle(0)
	return body
}

// NewStaticBody allocates and initializes a Body, and set it as a static body.
func NewStaticBody() *Body {
	body := NewBody(0, 0)
	body.SetType(Static)
	return body
}

// NewKinematicBody allocates and initializes a Body, and set it as a kinematic body.
func NewKinematicBody() *Body {
	body := NewBody(0, 0)
	body.SetType(Kinematic)
	return body
}

// SetAngle sets the angle of body.
func (body *Body) SetAngle(angle float64) {
	body.Activate()
	body.angle = angle
	body.SetTransform(body.position, angle)
}

// Moment returns moment of inertia of the body.
func (body Body) Moment() float64 {
	return body.momentOfInertia
}

// SetMoment sets moment of inertia of the body.
func (body *Body) SetMoment(moment float64) {
	body.Activate()
	body.momentOfInertia = moment
	body.momentOfInertiaInverse = 1 / moment
}

// Mass returns mass of the body
func (body *Body) Mass() float64 {
	return body.mass
}

// SetMass sets mass of the body
func (body *Body) SetMass(mass float64) {
	body.Activate()
	body.mass = mass
	body.massInverse = 1 / mass
}

// IdleTime returns sleeping idle time of the body
func (body *Body) IdleTime() float64 {
	return body.sleepingIdleTime
}

// SetType sets the type of the body.
func (body *Body) SetType(bt BodyType) {
	oldType := body.Type()
	if oldType == bt {
		return
	}

	if bt == Static {
		body.sleepingIdleTime = infinity
	} else {
		body.sleepingIdleTime = 0
	}

	if bt == Dynamic {
		body.mass = 0
		body.momentOfInertia = 0
		body.massInverse = infinity
		body.momentOfInertiaInverse = infinity

		body.AccumulateMassFromShapes()
	} else {
		body.mass = infinity
		body.momentOfInertia = infinity
		body.massInverse = 0
		body.momentOfInertiaInverse = 0

		body.velocity = v.Vec{}
		body.w = 0
	}

	// If the body is added to a space already, we'll need to update some space data structures.
	if body.Space == nil {
		return
	}
	if body.Space.locked != 0 {
		log.Fatalln("Space is locked")
	}

	if oldType != Static {
		body.Activate()
	}

	if oldType == Static {
		for i, b := range body.Space.StaticBodies {
			if b == body {
				body.Space.StaticBodies = append(body.Space.StaticBodies[:i], body.Space.StaticBodies[i+1:]...)
				break
			}
		}
		body.Space.DynamicBodies = append(body.Space.DynamicBodies, body)
	} else if bt == Static {
		for i, b := range body.Space.DynamicBodies {
			if b == body {
				body.Space.DynamicBodies = append(body.Space.DynamicBodies[:i], body.Space.DynamicBodies[i+1:]...)
				break
			}
		}
		body.Space.StaticBodies = append(body.Space.StaticBodies, body)
	}

	var fromIndex, toIndex *SpatialIndex
	if oldType == Static {
		fromIndex = body.Space.staticShapes
	} else {
		fromIndex = body.Space.dynamicShapes
	}

	if bt == Static {
		toIndex = body.Space.staticShapes
	} else {
		toIndex = body.Space.dynamicShapes
	}

	if oldType != bt {
		for _, shape := range body.Shapes {
			fromIndex.class.Remove(shape, shape.hashid)
			toIndex.class.Insert(shape, shape.hashid)
		}
	}
}

// Type returns the type of the body.
func (body *Body) Type() BodyType {
	if body.sleepingIdleTime == infinity {
		return Static
	}
	if body.mass == infinity {
		return Kinematic
	}
	return Dynamic
}

// AccumulateMassFromShapes should *only* be called when shapes with mass info are modified, added or removed.
func (body *Body) AccumulateMassFromShapes() {
	if body == nil || body.Type() != Dynamic {
		return
	}

	body.mass = 0
	body.momentOfInertia = 0
	body.centerOfGravity = v.Vec{}

	// cache position, realign at the end
	pos := body.Position()

	for _, shape := range body.Shapes {
		info := shape.MassInfo()
		m := info.m

		if info.m > 0 {
			msum := body.mass + m
			body.momentOfInertia += m*info.i + body.centerOfGravity.DistSq(info.cog)*(m*body.mass)/msum
			body.centerOfGravity = body.centerOfGravity.Lerp(info.cog, m/msum)
			body.mass = msum
		}
	}

	body.massInverse = 1.0 / body.mass
	body.momentOfInertiaInverse = 1.0 / body.momentOfInertia

	body.SetPosition(pos)
}

// CenterOfGravity returns the offset of the center of gravity in body local coordinates.
func (body Body) CenterOfGravity() v.Vec {
	return body.centerOfGravity
}

// Angle returns the angle of the body.
func (body *Body) Angle() float64 {
	return body.angle
}

// Rotation returns the rotation vector of the body.
//
// (The x basis vector of it's transform.)
func (body *Body) Rotation() v.Vec {
	return v.Vec{body.transform.a, body.transform.b}
}

// Position returns the position of the body.
func (body *Body) Position() v.Vec {
	return body.transform.Apply(v.Vec{})
}

// SetPosition sets the position of the body.
func (body *Body) SetPosition(position v.Vec) {
	body.Activate()
	body.position = body.transform.ApplyVector(body.centerOfGravity).Add(position)
	body.SetTransform(body.position, body.angle)
}

// Velocity returns the velocity of the body.
func (body *Body) Velocity() v.Vec {
	return body.velocity
}

// SetVelocity sets the velocity of the body.
//
// Shorthand for Body.SetVelocityVector()
func (body *Body) SetVelocity(x, y float64) {
	body.Activate()
	body.velocity = v.Vec{x, y}
}

// SetVelocityVector sets the velocity of the body
func (body *Body) SetVelocityVector(v v.Vec) {
	body.Activate()
	body.velocity = v
}

// UpdateVelocity is the default velocity integration function.
func (body *Body) UpdateVelocity(gravity v.Vec, damping, dt float64) {
	if body.Type() == Kinematic {
		return
	}
	// if body.mass < 0 && body.moi < 0 {
	// 	log.Fatalln("Body's mass and moment must be positive")
	// }

	body.velocity = body.velocity.Scale(damping).Add(gravity.Add(body.force.Scale(body.massInverse)).Scale(dt))
	body.w = body.w*damping + body.torque*body.momentOfInertiaInverse*dt

	body.force = v.Vec{}
	body.torque = 0
}

// Force returns the force applied to the body for the next time step.
func (body *Body) Force() v.Vec {
	return body.force
}

// SetForce sets the force applied to the body for the next time step.
func (body *Body) SetForce(force v.Vec) {
	body.Activate()
	body.force = force
}

// Torque returns the torque applied to the body for the next time step.
func (body *Body) Torque() float64 {
	return body.torque
}

// SetTorque sets the torque applied to the body for the next time step.
func (body *Body) SetTorque(torque float64) {
	body.Activate()
	body.torque = torque
}

// AngularVelocity returns the angular velocity of the body.
func (body *Body) AngularVelocity() float64 {
	return body.w
}

// SetAngularVelocity sets the angular velocity of the body.
func (body *Body) SetAngularVelocity(angularVelocity float64) {
	body.Activate()
	body.w = angularVelocity
}

// SetTransform sets transform
func (body *Body) SetTransform(p v.Vec, a float64) {
	rot := v.Vec{math.Cos(a), math.Sin(a)}
	c := body.centerOfGravity

	body.transform = NewTransformTranspose(
		rot.X, -rot.Y, p.X-(c.X*rot.X-c.Y*rot.Y),
		rot.Y, rot.X, p.Y-(c.X*rot.Y+c.Y*rot.X),
	)
}

// Transform returns body's transform
func (body *Body) Transform() Transform {
	return body.transform
}

// Activate wakes up a sleeping or idle body.
func (body *Body) Activate() {
	if !(body != nil && body.Type() == Dynamic) {
		return
	}
	body.sleepingIdleTime = 0
	root := body.ComponentRoot()
	if root != nil && root.IsSleeping() {
		space := root.Space
		// in the chipmunk code they shadow body, so here I am not
		bodyToo := root
		for bodyToo != nil {
			next := bodyToo.sleepingNext
			bodyToo.sleepingIdleTime = 0
			bodyToo.sleepingRoot = nil
			bodyToo.sleepingNext = nil
			space.Activate(bodyToo)

			bodyToo = next
		}

		for i := range space.sleepingComponents {
			if space.sleepingComponents[i] == root {
				space.sleepingComponents = append(space.sleepingComponents[:i], space.sleepingComponents[i+1:]...)
				break
			}
		}
	}

	for arbiter := body.arbiterList; arbiter != nil; arbiter = arbiter.Next(body) {
		// Reset the idle timer of things the body is touching as well.
		// That way things don't get left hanging in the air.
		var other *Body
		if arbiter.bodyA == body {
			other = arbiter.bodyB
		} else {
			other = arbiter.bodyA
		}
		if other.Type() != Static {
			other.sleepingIdleTime = 0
		}
	}
}

// ActivateStatic wakes up any sleeping or idle bodies touching this static body.
func (body *Body) ActivateStatic(filter *Shape) {
	for arb := body.arbiterList; arb != nil; arb = arb.Next(body) {
		if filter == nil || filter == arb.shapeA || filter == arb.shapeB {
			if arb.bodyA == body {
				arb.bodyB.Activate()
			} else {
				arb.bodyA.Activate()
			}
		}
	}
}

// IsSleeping returns true if the body is sleeping.
func (body *Body) IsSleeping() bool {
	return body.sleepingRoot != nil
}

// AttachShape adds shape to the body
func (body *Body) AttachShape(shape *Shape) {
	body.Shapes = append(body.Shapes, shape)
	if shape.MassInfo().m > 0 {
		body.AccumulateMassFromShapes()
	}
}

// KineticEnergy returns the kinetic energy of this body.
func (body *Body) KineticEnergy() float64 {
	// Need to do some fudging to avoid NaNs
	vsq := body.velocity.Dot(body.velocity)
	wsq := body.w * body.w
	var a, b float64
	if vsq != 0 {
		a = vsq * body.mass
	}
	if wsq != 0 {
		b = wsq * body.momentOfInertia
	}
	return a + b
}

func (body *Body) PushArbiter(arb *Arbiter) {
	next := body.arbiterList
	arb.ThreadForBody(body).next = next
	if next != nil {
		next.ThreadForBody(body).prev = arb
	}
	body.arbiterList = arb
}

func (root *Body) ComponentAdd(body *Body) {
	body.sleepingRoot = root

	if body != root {
		body.sleepingNext = root.sleepingNext
		root.sleepingNext = body
	}
}

func (body *Body) ComponentRoot() *Body {
	if body != nil {
		return body.sleepingRoot
	}
	return nil
}

// WorldToLocal converts from world to body local Coordinates.
//
// Convert a point in body local coordinates to world (absolute) coordinates.
func (body *Body) WorldToLocal(point v.Vec) v.Vec {
	return NewTransformRigidInverse(body.transform).Apply(point)
}

// LocalToWorld converts from body local to world coordinates.
//
// Convert a point in world (absolute) coordinates to body local coordinates affected by the position and rotation of the rigid body.
func (body *Body) LocalToWorld(point v.Vec) v.Vec {
	return body.transform.Apply(point)
}

// ApplyForceAtWorldPoint applies a force at world point.
func (body *Body) ApplyForceAtWorldPoint(force, point v.Vec) {
	body.Activate()
	body.force = body.force.Add(force)

	r := point.Sub(body.transform.Apply(body.centerOfGravity))
	body.torque += r.Cross(force)
}

// ApplyForceAtLocalPoint applies a force at local point.
func (body *Body) ApplyForceAtLocalPoint(force, point v.Vec) {
	body.ApplyForceAtWorldPoint(body.transform.ApplyVector(force), body.transform.Apply(point))
}

// ApplyImpulseAtWorldPoint applies impulse at world point
func (body *Body) ApplyImpulseAtWorldPoint(impulse, point v.Vec) {
	body.Activate()

	r := point.Sub(body.transform.Apply(body.centerOfGravity))
	applyImpulse(body, impulse, r)
}

// ApplyImpulseAtLocalPoint applies impulse at local point
func (body *Body) ApplyImpulseAtLocalPoint(impulse, point v.Vec) {
	body.ApplyImpulseAtWorldPoint(body.transform.ApplyVector(impulse), body.transform.Apply(point))
}

// VelocityAtLocalPoint returns the velocity of a point on a body.
//
// Get the world (absolute) velocity of a point on a rigid body specified in body local coordinates.
func (body *Body) VelocityAtLocalPoint(point v.Vec) v.Vec {
	r := body.transform.ApplyVector(point.Sub(body.centerOfGravity))
	return body.velocity.Add(perp(r).Scale(body.w))
}

// VelocityAtWorldPoint returns the velocity of a point on a body.
//
// Get the world (absolute) velocity of a point on a rigid body specified in world coordinates.
func (body *Body) VelocityAtWorldPoint(point v.Vec) v.Vec {
	r := point.Sub(body.transform.Apply(body.centerOfGravity))
	return body.velocity.Add(perp(r).Scale(body.w))
}

// RemoveConstraint removes constraint from the body.
func (body *Body) RemoveConstraint(constraint *Constraint) {
	body.constraintList = filterConstraints(body.constraintList, body, constraint)
}

// RemoveShape removes collision shape from the body.
func (body *Body) RemoveShape(shape *Shape) {
	body.Shapes = slices.DeleteFunc(body.Shapes, func(s *Shape) bool {
		return s == shape
	})
	if body.Type() == Dynamic && shape.massInfo.m > 0 {
		body.AccumulateMassFromShapes()
	}
}

// RemoveShapeAtIndex removes collision shape from the body.
func (body *Body) RemoveShapeAtIndex(index int) {
	body.RemoveShape(body.Shapes[index])
}

// SetVelocityUpdateFunc sets the callback used to update a body's velocity.
func (body *Body) SetVelocityUpdateFunc(f BodyVelocityFunc) {
	body.velocityFunc = f
}

// SetPositionUpdateFunc sets the callback used to update a body's position.
func (body *Body) SetPositionUpdateFunc(f BodyPositionFunc) {
	body.positionFunc = f
}

// EachArbiter calls f once for each arbiter that is currently active on the body.
func (body *Body) EachArbiter(f func(*Arbiter)) {
	arb := body.arbiterList
	for arb != nil {
		next := arb.Next(body)
		swapped := arb.swapped

		arb.swapped = body == arb.bodyB
		f(arb)

		arb.swapped = swapped
		arb = next
	}
}

// EachShape calls f once for each shape attached to this body
func (body *Body) EachShape(f func(*Shape)) {
	for i := range body.Shapes {
		f(body.Shapes[i])
	}
}

// EachConstraint calls f once for each constraint attached to this body
func (body *Body) EachConstraint(f func(*Constraint)) {
	constraint := body.constraintList
	for constraint != nil {
		next := constraint.Next(body)
		f(constraint)
		constraint = next
	}
}

func filterConstraints(node *Constraint, body *Body, filter *Constraint) *Constraint {
	if node == filter {
		return node.Next(body)
	} else if node.bodyA == body {
		node.nextA = filterConstraints(node.nextA, body, filter)
	} else {
		node.nextB = filterConstraints(node.nextB, body, filter)
	}
	return node
}

// BodyUpdateVelocity is default velocity integration function.
func BodyUpdateVelocity(body *Body, gravity v.Vec, damping, dt float64) {
	if body.Type() == Kinematic {
		return
	}

	body.velocity = body.velocity.Scale(damping).Add(gravity.Add(body.force.Scale(body.massInverse)).Scale(dt))
	body.w = body.w*damping + body.torque*body.momentOfInertiaInverse*dt

	body.force = v.Vec{}
	body.torque = 0
}

// BodyUpdatePosition is default position integration function.
func BodyUpdatePosition(body *Body, dt float64) {
	body.position = body.position.Add(body.velocity.Add(body.vBias).Scale(dt))
	body.angle = body.angle + (body.w+body.wBias)*dt
	body.SetTransform(body.position, body.angle)

	body.vBias = v.Vec{}
	body.wBias = 0
}
