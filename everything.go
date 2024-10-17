package cm

import (
	"fmt"
	"math"

	"github.com/setanarut/vec"
)

const (
	Infinity         = math.MaxFloat64
	MagicEpsilon     = 1e-5
	RadianConst      = math.Pi / 180
	DegreeConst      = 180 / math.Pi
	PooledBufferSize = 1024
)

// Arbiter states
const (
	// Arbiter is active and its the first collision.
	ArbiterStateFirstCollision = iota
	// Arbiter is active and its not the first collision.
	ArbiterStateNormal
	// Collision has been explicitly ignored.
	// Either by returning false from a begin collision handler or calling cmArbiterIgnore().
	ArbiterStateIgnore
	// Collison is no longer active. A space will cache an arbiter for up to cmSpace.collisionPersistence more steps.
	ArbiterStateCached
	// Collison arbiter is invalid because one of the shapes was removed.
	ArbiterStateInvalidated
)

var (
	NoGroup       uint = 0        // Value for group signifying that a shape is in no group.
	AllCategories uint = ^uint(0) // Value for Shape layers signifying that a shape is in every layer.
)

// ShapeFilterAll is s collision filter value for a shape that will collide with anything except ShapeFilterNone.
var ShapeFilterAll = ShapeFilter{NoGroup, AllCategories, AllCategories}

// ShapeFilterNone is a collision filter value for a shape that does not collide with anything.
var ShapeFilterNone = ShapeFilter{NoGroup, ^AllCategories, ^AllCategories}

// CollisionBeginFunc is collision begin event function callback type.
//
// Returning false from a begin callback causes the collision to be ignored until the the separate callback is called when the objects stop colliding.
type CollisionBeginFunc func(arb *Arbiter, space *Space, userData interface{}) bool

// CollisionPreSolveFunc is collision pre-solve event function callback type.
//
// Returning false from a pre-step callback causes the collision to be ignored until the next step.
type CollisionPreSolveFunc func(arb *Arbiter, space *Space, userData interface{}) bool

// CollisionPostSolveFunc is collision post-solve event function callback type.
type CollisionPostSolveFunc func(arb *Arbiter, space *Space, userData interface{})

// CollisionSeparateFunc is collision separate event function callback type.
type CollisionSeparateFunc func(arb *Arbiter, space *Space, userData interface{})

type CollisionType uintptr

// CollisionHandler is struct that holds function callback pointers to configure custom collision handling.
// Collision handlers have a pair of types; when a collision occurs between two shapes that have these types, the collision handler functions are triggered.
type CollisionHandler struct {
	// Collision type identifier of the first shape that this handler recognizes.
	// In the collision handler callback, the shape with this type will be the first argument. Read only.
	TypeA CollisionType
	// Collision type identifier of the second shape that this handler recognizes.
	// In the collision handler callback, the shape with this type will be the second argument. Read only.
	TypeB CollisionType
	// This function is called when two shapes with types that match this collision handler begin colliding.
	BeginFunc CollisionBeginFunc
	// This function is called each step when two shapes with types that match this collision handler are colliding.
	// It's called before the collision solver runs so that you can affect a collision's outcome.
	PreSolveFunc CollisionPreSolveFunc
	// This function is called each step when two shapes with types that match this collision handler are colliding.
	// It's called after the collision solver runs so that you can read back information about the collision to trigger events in your game.
	PostSolveFunc CollisionPostSolveFunc
	// This function is called when two shapes with types that match this collision handler stop colliding.
	SeparateFunc CollisionSeparateFunc
	// This is a user definable context pointer that is passed to all of the collision handler functions.
	UserData interface{}
}

type Contact struct {
	r1, r2 vec.Vec2

	nMass, tMass float64
	bounce       float64 // TODO: look for an alternate bounce solution

	jnAcc, jtAcc, jBias float64
	bias                float64

	hash HashValue
}

func (c *Contact) Clone() Contact {
	return Contact{
		r1:     c.r1,
		r2:     c.r2,
		nMass:  c.nMass,
		tMass:  c.tMass,
		bounce: c.bounce,
		jnAcc:  c.jnAcc,
		jtAcc:  c.jtAcc,
		jBias:  c.jBias,
		bias:   c.bias,
		hash:   c.hash,
	}
}

// CollisionInfo collision info struct
type CollisionInfo struct {
	a, b        *Shape
	collisionId uint32

	n     vec.Vec2
	count int
	arr   []Contact
}

func (info *CollisionInfo) PushContact(p1, p2 vec.Vec2, hash HashValue) {

	con := &info.arr[info.count]
	con.r1 = p1
	con.r2 = p2
	con.hash = hash

	info.count++
}

// ShapeMassInfo is mass info struct
type ShapeMassInfo struct {
	m, i, area float64
	cog        vec.Vec2
}

// PointQueryInfo is point query info struct.
type PointQueryInfo struct {
	// The nearest shape, nil if no shape was within range.
	Shape *Shape
	// The closest point on the shape's surface. (in world space coordinates)
	Point vec.Vec2
	// The distance to the point. The distance is negative if the point is inside the shape.
	Distance float64
	// The gradient of the signed distance function.
	// The value should be similar to info.p/info.d, but accurate even for very small values of info.d.
	Gradient vec.Vec2
}

// SegmentQueryInfo is segment query info struct.
type SegmentQueryInfo struct {
	// The shape that was hit, or nil if no collision occurred.
	Shape *Shape
	// The point of impact.
	Point vec.Vec2
	// The normal of the surface hit.
	Normal vec.Vec2
	// The normalized distance along the query segment in the range [0, 1].
	Alpha float64
}

type SplittingPlane struct {
	v0, n vec.Vec2
}

// ShapeFilter is fast collision filtering type that is used to determine if two objects collide before calling collision or query callbacks.
type ShapeFilter struct {
	// Two objects with the same non-zero group value do not collide.
	// This is generally used to group objects in a composite object together to disable self collisions.
	Group uint
	// A bitmask of user definable categories that this object belongs to.
	// The category/mask combinations of both objects in a collision must agree for a collision to occur.
	Categories uint
	// A bitmask of user definable category types that this object object collides with.
	// The category/mask combinations of both objects in a collision must agree for a collision to occur.
	Mask uint
}

// NewShapeFilter creates a new collision filter.
func NewShapeFilter(group, categories, mask uint) ShapeFilter {
	return ShapeFilter{group, categories, mask}
}

// Reject checks whether two ShapeFilter objects should be considered incompatible.
// It returns true if the filters should be rejected based on the following conditions:
// - If both filters belong to the same group (and the group is not 0).
// - If the category/mask combination of either filter does not match the other.
//   - Specifically, it checks if the categories of the first filter don't match
//     the mask of the second, or vice versa.
//
// Returns true if the filters are considered incompatible, otherwise false.
func (sf ShapeFilter) Reject(other ShapeFilter) bool {
	return (sf.Group != 0 && sf.Group == other.Group) ||
		(sf.Categories&other.Mask) == 0 ||
		(other.Categories&sf.Mask) == 0
}

// Mat2x2 is a 2x2 matrix type used for tensors and such.
type Mat2x2 struct {
	a, b, c, d float64
}

// Transform transforms Vector v
func (m *Mat2x2) Transform(v vec.Vec2) vec.Vec2 {
	return vec.Vec2{v.X*m.a + v.Y*m.b, v.X*m.c + v.Y*m.d}
}

// MomentForBox calculates the moment of inertia for a solid box.
func MomentForBox(mass, width, height float64) float64 {
	return mass * (width*width + height*height) / 12.0
}

// MomentForBox2 calculates the moment of inertia for a solid box.
func MomentForBox2(mass float64, box BB) float64 {
	width := box.R - box.L
	height := box.T - box.B
	offset := vec.Vec2{box.L + box.R, box.B + box.T}.Scale(0.5)

	// TODO: NaN when offset is 0 and m is INFINITY
	return MomentForBox(mass, width, height) + mass*offset.LengthSq()
}

// MomentForCircle calculates the moment of inertia for a circle.
//
// d1 and d2 are the inner and outer diameters. A solid circle has an inner diameter (d1) of 0.
func MomentForCircle(mass, d1, d2 float64, offset vec.Vec2) float64 {
	return mass * (0.5*(d1*d1+d2*d2) + offset.LengthSq())
}

// MomentForSegment calculates the moment of inertia for a line segment.
//
// Beveling radius is not supported.
func MomentForSegment(mass float64, a, b vec.Vec2, r float64) float64 {
	offset := a.Lerp(b, 0.5)
	length := b.Distance(a) + 2.0*r
	return mass * ((length*length+4.0*r*r)/12.0 + offset.LengthSq())
}

// MomentForPoly calculates the moment of inertia for a solid polygon shape assuming it's center of gravity is at it's centroid.
// The offset is added to each vertex.
func MomentForPoly(mass float64, count int, verts []vec.Vec2, offset vec.Vec2, r float64) float64 {
	if count == 2 {
		return MomentForSegment(mass, verts[0], verts[1], 0)
	}

	var sum1 float64
	var sum2 float64
	for i := 0; i < count; i++ {
		v1 := verts[i].Add(offset)
		v2 := verts[(i+1)%count].Add(offset)

		a := v2.Cross(v1)
		b := v1.Dot(v1) + v1.Dot(v2) + v2.Dot(v2)

		sum1 += a * b
		sum2 += a
	}

	return (mass * sum1) / (6.0 * sum2)
}

// AreaForCircle returns area of a hollow circle.
//
// r1 and r2 are the inner and outer diameters. A solid circle has an inner diameter of 0.
func AreaForCircle(r1, r2 float64) float64 {
	return math.Pi * math.Abs(r1*r1-r2*r2)
}

// AreaForSegment calculates the area of a fattened (capsule shaped) line segment.
func AreaForSegment(a, b vec.Vec2, r float64) float64 {
	return r * (math.Pi*r + 2.0*a.Distance(b))
}

// AreaForPoly calculates the signed area of a polygon.
//
// A Clockwise winding gives positive area. This is probably backwards from what you expect, but matches Chipmunk's the winding for poly shapes.
func AreaForPoly(count int, verts []vec.Vec2, r float64) float64 {
	var area float64
	var perimeter float64
	for i := 0; i < count; i++ {
		v1 := verts[i]
		v2 := verts[(i+1)%count]

		area += v1.Cross(v2)
		perimeter += v1.Distance(v2)
	}

	return r*(math.Pi*math.Abs(r)+perimeter) + area/2.0
}

// CentroidForPoly calculates the natural centroid of a polygon.
func CentroidForPoly(count int, verts []vec.Vec2) vec.Vec2 {
	var sum float64
	vsum := vec.Vec2{}

	for i := 0; i < count; i++ {
		v1 := verts[i]
		v2 := verts[(i+1)%count]
		cross := v1.Cross(v2)

		sum += cross
		vsum = vsum.Add(v1.Add(v2).Scale(cross))
	}

	return vsum.Scale(1.0 / (3.0 * sum))
}

// DebugInfo returns info of space
func DebugInfo(space *Space) string {
	arbiters := len(space.Arbiters)
	points := 0

	for i := 0; i < arbiters; i++ {
		points += int(space.Arbiters[i].count)
	}

	constraints := len(space.constraints) + points*int(space.Iterations)
	if arbiters > maxArbiters {
		maxArbiters = arbiters
	}
	if points > maxPoints {
		maxPoints = points
	}
	if constraints > maxConstraints {
		maxConstraints = constraints
	}

	var ke float64
	for _, body := range space.DynamicBodies {
		if body.mass == Infinity || body.momentOfInertia == Infinity {
			continue
		}
		ke += body.mass*body.velocity.Dot(body.velocity) + body.momentOfInertia*body.w*body.w
	}

	return fmt.Sprintf(`Arbiters: %d (%d) - Contact Points: %d (%d)
Other Constraints: %d, Iterations: %d
Constraints x Iterations: %d (%d)
KE: %e`, arbiters, maxArbiters,
		points, maxPoints, len(space.constraints), space.Iterations, constraints, maxConstraints, ke)
}

func kScalarBody(body *Body, r, n vec.Vec2) float64 {
	rcn := r.Cross(n)
	return body.massInverse + body.momentOfInertiaInverse*rcn*rcn
}

func lerp(f1, f2, t float64) float64 {
	return f1*(1.0-t) + f2*t
}

func clamp01(f float64) float64 {
	return math.Max(0, math.Min(f, 1))
}

func kScalar(a, b *Body, r1, r2, n vec.Vec2) float64 {
	return kScalarBody(a, r1, n) + kScalarBody(b, r2, n)
}

func normalRelativeVelocity(a, b *Body, r1, r2, n vec.Vec2) float64 {
	return relativeVelocity(a, b, r1, r2).Dot(n)
}

func kTensor(a, b *Body, r1, r2 vec.Vec2) Mat2x2 {
	mSum := a.massInverse + b.massInverse

	// start with Identity*mSum
	k11 := mSum
	k12 := 0.0
	k21 := 0.0
	k22 := mSum

	// add the influence from r1
	aIInv := a.momentOfInertiaInverse
	r1xsq := r1.X * r1.X * aIInv
	r1ysq := r1.Y * r1.Y * aIInv
	r1nxy := -r1.X * r1.Y * aIInv
	k11 += r1ysq
	k12 += r1nxy
	k21 += r1nxy
	k22 += r1xsq

	// add the influence from r2
	bIInv := b.momentOfInertiaInverse
	r2xsq := r2.X * r2.X * bIInv
	r2ysq := r2.Y * r2.Y * bIInv
	r2nxy := -r2.X * r2.Y * bIInv
	k11 += r2ysq
	k12 += r2nxy
	k21 += r2nxy
	k22 += r2xsq

	// invert
	det := k11*k22 - k12*k21
	// if det == 0.0 {
	// 	log.Fatalln("Unsolvable constraint")
	// }

	detInv := 1.0 / det
	return Mat2x2{
		k22 * detInv, -k12 * detInv,
		-k21 * detInv, k11 * detInv,
	}
}

func biasCoef(errorBias, dt float64) float64 {
	return 1.0 - math.Pow(errorBias, dt)
}

var maxArbiters, maxPoints, maxConstraints int

func clamp(f, min, max float64) float64 {
	if f > min {
		return math.Min(f, max)
	} else {
		return math.Min(min, max)
	}
}

// collision related
func lerpT(a, b vec.Vec2, t float64) vec.Vec2 {
	ht := 0.5 * t
	return a.Scale(0.5 - ht).Add(b.Scale(0.5 + ht))
}

func closestDist(v0, v1 vec.Vec2) float64 {
	return lerpT(v0, v1, closestT(v0, v1)).LengthSq()
}

func closestT(a, b vec.Vec2) float64 {
	delta := b.Sub(a)
	return -clamp(delta.Dot(a.Add(b))/delta.LengthSq(), -1.0, 1.0)
}

func closestPointOnSegment(v, a, b vec.Vec2) vec.Vec2 {
	delta := a.Sub(b)
	t := clamp01(delta.Dot(v.Sub(b)) / delta.LengthSq())
	return b.Add(delta.Scale(t))
}

func checkAxis(v, v1, p, n vec.Vec2) bool {
	return p.Dot(n) <= math.Max(v.Dot(n), v1.Dot(n))
}

func pointGreater(v, b, c vec.Vec2) bool {
	return (b.Y-v.Y)*(v.X+b.X-2*c.X) > (b.X-v.X)*(v.Y+b.Y-2*c.Y)
}
