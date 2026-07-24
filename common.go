package cm

import (
	"fmt"
	"math"

	"github.com/setanarut/v"
)

const (
	pooledBufferSize int     = 1024
	magicEpsilon     float64 = 1e-8
)

var infinity = math.Inf(1)

type CollisionType uintptr

type Contact struct {
	R1, R2 v.Vec

	nMass, tMass float64
	bounce       float64 // TODO: look for an alternate bounce solution

	jnAcc, jtAcc, jBias float64
	bias                float64

	hash HashValue
}

// CollisionInfo collision info struct
type CollisionInfo struct {
	a, b        *Shape
	collisionId uint32

	n     v.Vec
	count int
	arr   []Contact
}

func (info *CollisionInfo) PushContact(p1, p2 v.Vec, hash HashValue) {

	con := &info.arr[info.count]
	con.R1 = p1
	con.R2 = p2
	con.hash = hash

	info.count++
}

// ShapeMassInfo is mass info struct
type ShapeMassInfo struct {
	m, i, area float64
	// Center of gravity
	cog v.Vec
}

// PointQueryInfo is point query info struct.
type PointQueryInfo struct {
	// The nearest shape, nil if no shape was within range.
	Shape *Shape
	// The closest point on the shape's surface. (in world space coordinates)
	Point v.Vec
	// The distance to the point. The distance is negative if the point is inside the shape.
	Distance float64
	// The gradient of the signed distance function.
	// The value should be similar to info.p/info.d, but accurate even for very small values of info.d.
	Gradient v.Vec
}

// SegmentQueryInfo is segment query info struct.
type SegmentQueryInfo struct {
	// The shape that was hit, or nil if no collision occurred.
	Shape *Shape
	// The point of impact.
	Point v.Vec
	// The normal of the surface hit.
	Normal v.Vec
	// The normalized distance along the query segment in the range [0, 1].
	Alpha float64
}

type SplittingPlane struct {
	V0, N v.Vec
}

// mat2x2 is a 2x2 matrix type used for tensors and such.
type mat2x2 struct {
	a, b, c, d float64
}

// transform transforms Vector a
func (m *mat2x2) transform(a v.Vec) v.Vec {
	return v.Vec{a.X*m.a + a.Y*m.b, a.X*m.c + a.Y*m.d}
}

// DebugInfo returns info of space
func DebugInfo(space *Space) string {
	var maxArbiters, maxPoints, maxConstraints int
	arbiters := len(space.Arbiters)
	points := 0

	for i := range arbiters {
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
		if body.mass == infinity || body.momentOfInertia == infinity {
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

func kScalarBody(body *Body, r, n v.Vec) float64 {
	rcn := r.Cross(n)
	return body.massInverse + body.momentOfInertiaInverse*rcn*rcn
}

func clamp01(f float64) float64 {
	return max(0, min(f, 1))
}

func kScalar(a, b *Body, r1, r2, n v.Vec) float64 {
	return kScalarBody(a, r1, n) + kScalarBody(b, r2, n)
}

func normalRelativeVelocity(a, b *Body, r1, r2, n v.Vec) float64 {
	return relativeVelocity(a, b, r1, r2).Dot(n)
}

func kTensor(a, b *Body, r1, r2 v.Vec) mat2x2 {
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
	return mat2x2{
		k22 * detInv, -k12 * detInv,
		-k21 * detInv, k11 * detInv,
	}
}

func biasCoef(errorBias, dt float64) float64 {
	return 1.0 - math.Pow(errorBias, dt)
}

func clamp(f, low, high float64) float64 {
	if f > low {
		return min(f, high)
	} else {
		return min(low, high)
	}
}

// collision related
func lerpT(a, b v.Vec, t float64) v.Vec {
	ht := 0.5 * t
	return a.Scale(0.5 - ht).Add(b.Scale(0.5 + ht))
}

func closestDist(v0, v1 v.Vec) float64 {
	return lerpT(v0, v1, closestT(v0, v1)).MagSq()
}

func closestT(a, b v.Vec) float64 {
	delta := b.Sub(a)
	return -clamp(delta.Dot(a.Add(b))/delta.MagSq(), -1.0, 1.0)
}

func closestPointOnSegment(v, a, b v.Vec) v.Vec {
	delta := a.Sub(b)
	t := clamp01(delta.Dot(v.Sub(b)) / delta.MagSq())
	return b.Add(delta.Scale(t))
}

func checkAxis(v, v1, p, n v.Vec) bool {
	return p.Dot(n) <= max(v.Dot(n), v1.Dot(n))
}

func pointGreater(v, b, c v.Vec) bool {
	return (b.Y-v.Y)*(v.X+b.X-2*c.X) > (b.X-v.X)*(v.Y+b.Y-2*c.Y)
}

// RotateComplex uses complex number multiplication to rotate this by other.
//
// Scaling will occur if this is not a unit vector.
func rotateComplex(this, other v.Vec) v.Vec {
	return v.Vec{this.X*other.X - this.Y*other.Y, this.X*other.Y + this.Y*other.X}
}

// Perp returns a perpendicular vector. (90 degree rotation)
func perp(a v.Vec) v.Vec {
	return v.Vec{-a.Y, a.X}
}

// ReversePerp returns a perpendicular vector. (-90 degree rotation)
func reversePerp(a v.Vec) v.Vec {
	return v.Vec{a.Y, -a.X}
}

// IsNear returns true if the distance between this and other is less than dist.
func isNear(this, other v.Vec, dist float64) bool {
	return this.DistSq(other) < dist*dist
}
