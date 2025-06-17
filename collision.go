package cm

import (
	"log"
	"math"

	"github.com/setanarut/v"
)

const (
	maxGjkIterations  = 30
	maxEpaIterations  = 30
	warnEpaIterations = 20
)

type SupportPoint struct {
	p v.Vec
	// Save an index of the point so it can be cheaply looked up as a starting point for the next frame.
	index uint32
}

func NewSupportPoint(p v.Vec, index uint32) SupportPoint {
	return SupportPoint{p, index}
}

type SupportPointFunc func(shape *Shape, n v.Vec) SupportPoint

func PolySupportPoint(shape *Shape, n v.Vec) SupportPoint {
	poly := shape.Class.(*PolyShape)
	planes := poly.Planes
	i := PolySupportPointIndex(poly.count, planes, n)
	return NewSupportPoint(planes[i].V0, uint32(i))
}

func SegmentSupportPoint(shape *Shape, n v.Vec) SupportPoint {
	seg := shape.Class.(*Segment)
	if seg.transformA.Dot(n) > seg.transformB.Dot(n) {
		return NewSupportPoint(seg.transformA, 0)
	} else {
		return NewSupportPoint(seg.transformB, 1)
	}
}

func CircleSupportPoint(shape *Shape, _ v.Vec) SupportPoint {
	return NewSupportPoint(shape.Class.(*Circle).transformC, 0)
}

func PolySupportPointIndex(count int, planes []SplittingPlane, n v.Vec) int {
	max := -infinity
	var index int
	for i := range count {
		v := planes[i].V0
		d := v.Dot(n)
		if d > max {
			max = d
			index = i
		}
	}

	return index
}

type SupportContext struct {
	shape1, shape2 *Shape
	func1, func2   SupportPointFunc
}

// Support calculates the maximal point on the minkowski difference of two shapes along a particular axis.
func (ctx *SupportContext) Support(n v.Vec) MinkowskiPoint {
	a := ctx.func1(ctx.shape1, n.Neg())
	b := ctx.func2(ctx.shape2, n)
	return NewMinkowskiPoint(a, b)
}

type ClosestPoints struct {
	// Surface points in absolute coordinates.
	a, b v.Vec
	// Minimum separating axis of the two shapes.
	n v.Vec
	// Signed distance between the points.
	d float64
	// Concatenation of the id's of the minkoski points.
	collisionId uint32
}

type CollisionFunc func(info *CollisionInfo)

func CircleToCircle(info *CollisionInfo) {
	c1 := info.a.Class.(*Circle)
	c2 := info.b.Class.(*Circle)

	mindist := c1.radius + c2.radius
	delta := c2.transformC.Sub(c1.transformC)
	distsq := delta.MagSq()

	if distsq < mindist*mindist {
		dist := math.Sqrt(distsq)
		if dist != 0 {
			info.n = delta.Scale(1.0 / dist)
		} else {
			info.n = v.Vec{1, 0}
		}
		info.PushContact(c1.transformC.Add(info.n.Scale(c1.radius)), c2.transformC.Add(info.n.Scale(-c2.radius)), 0)
	}
}

func CollisionError(_ *CollisionInfo) {
	panic("Shape types are not sorted")
}

func CircleToSegment(info *CollisionInfo) {
	circle := info.a.Class.(*Circle)
	segment := info.b.Class.(*Segment)

	segA := segment.transformA
	segB := segment.transformB
	center := circle.transformC

	segDelta := segB.Sub(segA)
	closestT := clamp01(segDelta.Dot(center.Sub(segA)) / segDelta.MagSq())
	closest := segA.Add(segDelta.Scale(closestT))

	mindist := circle.radius + segment.radius
	delta := closest.Sub(center)
	distsq := delta.MagSq()
	if distsq < mindist*mindist {
		dist := math.Sqrt(distsq)
		if dist != 0 {
			info.n = delta.Scale(1 / dist)
		} else {
			info.n = segment.transformN
		}
		n := info.n

		rot := segment.Shape.Body.Rotation()
		if (closestT != 0.0 || n.Dot(rotateComplex(segment.aTangent, rot)) >= 0.0) &&
			(closestT != 1.0 || n.Dot(rotateComplex(segment.bTangent, rot)) >= 0.0) {
			info.PushContact(center.Add(n.Scale(circle.radius)), closest.Add(n.Scale(-segment.radius)), 0)
		}
	}
}

func SegmentToSegment(info *CollisionInfo) {
	seg1 := info.a.Class.(*Segment)
	seg2 := info.b.Class.(*Segment)

	context := SupportContext{info.a, info.b, SegmentSupportPoint, SegmentSupportPoint}
	points := GJK(context, &info.collisionId)

	n := points.n
	rot1 := seg1.Body.Rotation()
	rot2 := seg2.Body.Rotation()

	if points.d > (seg1.radius + seg2.radius) {
		return
	}

	if (!points.a.Equals(seg1.transformA) || n.Dot(rotateComplex(seg1.aTangent, rot1)) <= 0) &&
		(!points.a.Equals(seg1.transformB) || n.Dot(rotateComplex(seg1.bTangent, rot1)) <= 0) &&
		(!points.b.Equals(seg2.transformA) || n.Dot(rotateComplex(seg2.aTangent, rot2)) >= 0) &&
		(!points.b.Equals(seg2.transformB) || n.Dot(rotateComplex(seg2.bTangent, rot2)) >= 0) {
		ContactPoints(SupportEdgeForSegment(seg1, n), SupportEdgeForSegment(seg2, n.Neg()), points, info)
	}
}

func CircleToPoly(info *CollisionInfo) {
	context := SupportContext{info.a, info.b, CircleSupportPoint, PolySupportPoint}
	points := GJK(context, &info.collisionId)

	circle := info.a.Class.(*Circle)
	poly := info.b.Class.(*PolyShape)

	if points.d <= circle.radius+poly.Radius {
		info.n = points.n
		info.PushContact(points.a.Add(info.n.Scale(circle.radius)), points.b.Add(info.n.Scale(poly.Radius)), 0)
	}
}

func SegmentToPoly(info *CollisionInfo) {
	context := SupportContext{info.a, info.b, SegmentSupportPoint, PolySupportPoint}
	points := GJK(context, &info.collisionId)

	n := points.n
	rot := info.a.Body.Rotation()

	segment := info.a.Class.(*Segment)
	polyshape := info.b.Class.(*PolyShape)

	// If the closest points are nearer than the sum of the radii...
	if points.d-segment.radius-polyshape.Radius <= 0 && (
	// Reject endcap collisions if tangents are provided.
	(!points.a.Equals(segment.transformA) || n.Dot(rotateComplex(segment.aTangent, rot)) <= 0) &&
		(!points.a.Equals(segment.transformB) || n.Dot(rotateComplex(segment.bTangent, rot)) <= 0)) {
		ContactPoints(SupportEdgeForSegment(segment, n), SupportEdgeForPoly(polyshape, n.Neg()), points, info)
	}
}

func PolyToPoly(info *CollisionInfo) {
	context := SupportContext{info.a, info.b, PolySupportPoint, PolySupportPoint}
	points := GJK(context, &info.collisionId)

	// TODO: add debug drawing logic like chipmunk does

	poly1 := info.a.Class.(*PolyShape)
	poly2 := info.b.Class.(*PolyShape)
	if points.d-poly1.Radius-poly2.Radius <= 0 {
		ContactPoints(SupportEdgeForPoly(poly1, points.n), SupportEdgeForPoly(poly2, points.n.Neg()), points, info)
	}
}

// MinkowskiPoint is a point on the surface of two shapes' minkowski difference.
type MinkowskiPoint struct {
	// Cache the two original support points.
	a, b v.Vec
	// b - a
	ab v.Vec
	// Concatenate the two support point indexes.
	collisionId uint32
}

func NewMinkowskiPoint(a, b SupportPoint) MinkowskiPoint {
	return MinkowskiPoint{a.p, b.p, b.p.Sub(a.p), (a.index&0xFF)<<8 | (b.index & 0xFF)}
}

// ClosestPoints calculates the closest points on two shapes given the closest edge on their minkowski difference to (0, 0)
func (v0 MinkowskiPoint) ClosestPoints(v1 MinkowskiPoint) ClosestPoints {
	// Find the closest p(t) on the minkowski difference to (0, 0)
	t := closestT(v0.ab, v1.ab)
	p := lerpT(v0.ab, v1.ab, t)

	// Interpolate the original support points using the same 't' value as above.
	// This gives you the closest surface points in absolute coordinates. NEAT!
	pa := lerpT(v0.a, v1.a, t)
	pb := lerpT(v0.b, v1.b, t)
	id := (v0.collisionId&0xFFFF)<<16 | (v1.collisionId & 0xFFFF)

	// First try calculating the MSA from the minkowski difference edge.
	// This gives us a nice, accurate MSA when the surfaces are close together.
	delta := v1.ab.Sub(v0.ab)
	n := reversePerp(delta).Unit()
	d := n.Dot(p)

	if d <= 0 || (-1 < t && t < 1) {
		// If the shapes are overlapping, or we have a regular vertex/edge collision, we are done.
		return ClosestPoints{pa, pb, n, d, id}
	}

	// Vertex/vertex collisions need special treatment since the MSA won't be shared with an axis of the minkowski difference.
	d2 := p.Mag()
	n2 := p.Scale(1 / (d2 + math.SmallestNonzeroFloat64))

	return ClosestPoints{pa, pb, n2, d2, id}
}

type EdgePoint struct {
	p v.Vec
	// Keep a hash value for Chipmunk's collision hashing mechanism.
	hash HashValue
}

type Edge struct {
	a, b EdgePoint
	r    float64
	n    v.Vec
}

func SupportEdgeForSegment(seg *Segment, n v.Vec) Edge {
	hashid := seg.Shape.hashid
	if seg.transformN.Dot(n) > 0 {
		return Edge{
			a: EdgePoint{seg.transformA, HashPair(hashid, 0)},
			b: EdgePoint{seg.transformB, HashPair(hashid, 1)},
			r: seg.radius,
			n: seg.transformN,
		}
	}

	return Edge{
		a: EdgePoint{seg.transformB, HashPair(hashid, 1)},
		b: EdgePoint{seg.transformA, HashPair(hashid, 0)},
		r: seg.radius,
		n: seg.transformN.Neg(),
	}
}

func SupportEdgeForPoly(poly *PolyShape, n v.Vec) Edge {
	count := poly.count
	i1 := PolySupportPointIndex(poly.count, poly.Planes, n)

	i0 := (i1 - 1 + count) % count
	i2 := (i1 + 1) % count

	planes := poly.Planes
	hashId := poly.hashid

	if n.Dot(planes[i1].N) > n.Dot(planes[i2].N) {
		return Edge{
			EdgePoint{planes[i0].V0, HashPair(hashId, HashValue(i0))},
			EdgePoint{planes[i1].V0, HashPair(hashId, HashValue(i1))},
			poly.Radius,
			planes[i1].N,
		}
	}

	return Edge{
		EdgePoint{planes[i1].V0, HashPair(hashId, HashValue(i1))},
		EdgePoint{planes[i2].V0, HashPair(hashId, HashValue(i2))},
		poly.Radius,
		planes[i2].N,
	}
}

// ContactPoints finds contact point pairs on two support edges' surfaces
func ContactPoints(e1, e2 Edge, points ClosestPoints, info *CollisionInfo) {
	mindist := e1.r + e2.r

	if points.d > mindist {
		return
	}

	n := points.n
	info.n = points.n

	dE1A := e1.a.p.Cross(n)
	dE1B := e1.b.p.Cross(n)
	dE2A := e2.a.p.Cross(n)
	dE2B := e2.b.p.Cross(n)

	// TODO + min isn't a complete fix
	e1Denom := 1 / (dE1B - dE1A + math.SmallestNonzeroFloat64) // try 1e-15
	e2Denom := 1 / (dE2B - dE2A + math.SmallestNonzeroFloat64) // try 1e-15

	// Project the endpoints of the two edges onto the opposing edge, clamping them as necessary.
	// Compare the projected points to the collision normal to see if the shapes overlap there.
	{
		p1 := n.Scale(e1.r).Add(e1.a.p.Lerp(e1.b.p, clamp01((dE2B-dE1A)*e1Denom)))
		p2 := n.Scale(-e2.r).Add(e2.a.p.Lerp(e2.b.p, clamp01((dE1A-dE2A)*e2Denom)))
		dist := p2.Sub(p1).Dot(n)
		if dist <= 0 {
			hash1a2b := HashPair(e1.a.hash, e2.b.hash)
			info.PushContact(p1, p2, hash1a2b)
		}
	}
	{
		p1 := n.Scale(e1.r).Add(e1.a.p.Lerp(e1.b.p, clamp01((dE2A-dE1A)*e1Denom)))
		p2 := n.Scale(-e2.r).Add(e2.a.p.Lerp(e2.b.p, clamp01((dE1B-dE2A)*e2Denom)))
		dist := p2.Sub(p1).Dot(n)
		if dist <= 0 {
			hash1b2a := HashPair(e1.b.hash, e2.a.hash)
			info.PushContact(p1, p2, hash1b2a)
		}
	}
}

const hashCoef = 3344921057

func HashPair(a, b HashValue) HashValue {
	return a*hashCoef ^ b*hashCoef
}

// GJK finds the closest points between two shapes using the GJK algorithm.
func GJK(ctx SupportContext, collisionId *uint32) ClosestPoints {
	var v0, v1 MinkowskiPoint

	if *collisionId != 0 {
		// Use the minkowski points from the last frame as a starting point using the cached indexes.
		v0 = NewMinkowskiPoint(ctx.shape1.Point((*collisionId>>24)&0xFF), ctx.shape2.Point((*collisionId>>16)&0xFF))
		v1 = NewMinkowskiPoint(ctx.shape1.Point((*collisionId>>8)&0xFF), ctx.shape2.Point((*collisionId)&0xFF))
	} else {
		// No cached indexes, use the shapes' bounding box centers as a guess for a starting axis.
		axis := perp(ctx.shape1.BB.Center().Sub(ctx.shape2.BB.Center()))
		v0 = ctx.Support(axis)
		v1 = ctx.Support(axis.Neg())
	}

	points := GJKRecurse(ctx, v0, v1, 1)
	*collisionId = points.collisionId
	return points
}

// GJKRecurse implementation of the GJK loop.
func GJKRecurse(ctx SupportContext, v0, v1 MinkowskiPoint, iteration int) ClosestPoints {
	if iteration > maxGjkIterations {
		return v0.ClosestPoints(v1)
	}

	if pointGreater(v1.ab, v0.ab, v.Vec{}) {
		// Origin is behind axis. Flip and try again.
		return GJKRecurse(ctx, v1, v0, iteration)
	}
	t := closestT(v0.ab, v1.ab)
	var n v.Vec
	if -1.0 < t && t < 1.0 {
		n = perp(v1.ab.Sub(v0.ab))
	} else {
		n = lerpT(v0.ab, v1.ab, t).Neg()
	}
	p := ctx.Support(n)

	// Draw debug

	if pointGreater(p.ab, v0.ab, v.Vec{}) && pointGreater(v1.ab, p.ab, v.Vec{}) {
		return EPA(ctx, v0, p, v1)
	}

	if checkAxis(v0.ab, v1.ab, p.ab, n) {
		return v0.ClosestPoints(v1)
	}

	if closestDist(v0.ab, p.ab) < closestDist(p.ab, v1.ab) {
		return GJKRecurse(ctx, v0, p, iteration+1)
	}

	return GJKRecurse(ctx, p, v1, iteration+1)
}

// EPA is called from GJK when two shapes overlap.
// Finds the closest points on the surface of two overlapping shapes using the EPA algorithm.
// This is a moderately expensive step! Avoid it by adding radii to your shapes so their inner polygons won't overlap.
func EPA(ctx SupportContext, v0, v1, v2 MinkowskiPoint) ClosestPoints {
	// TODO: allocate a NxM array here and do an in place convex hull reduction in EPARecurse?
	hull := []MinkowskiPoint{v0, v1, v2}
	return EPARecurse(ctx, 3, hull, 1)
}

// EPARecurse implementation of the EPA loop.
// Each recursion adds a point to the convex hull until it's known that we have the closest point on the surface.
func EPARecurse(ctx SupportContext, count int, hull []MinkowskiPoint, iteration int) ClosestPoints {
	mini := 0
	minDist := infinity

	// TODO: precalculate this when building the hull and save a step.
	// Find the closest segment hull[i] and hull[i + 1] to (0, 0)
	i := count - 1
	j := 0
	for j < count {
		d := closestDist(hull[i].ab, hull[j].ab)
		if d < minDist {
			minDist = d
			mini = i
		}
		i = j
		j++
	}

	v0 := hull[mini]
	v1 := hull[(mini+1)%count]

	p := ctx.Support(perp(v1.ab.Sub(v0.ab)))

	duplicate := p.collisionId == v0.collisionId || p.collisionId == v1.collisionId

	if !duplicate && pointGreater(v0.ab, v1.ab, p.ab) && iteration < maxEpaIterations {
		// Rebuild the convex hull by inserting p.
		hull2 := make([]MinkowskiPoint, count+1)
		count2 := 1
		hull2[0] = p

		for i := range count {
			index := (mini + 1 + i) % count

			h0 := hull2[count2-1].ab
			h1 := hull[index].ab
			var h2 v.Vec
			if i+1 < count {
				h2 = hull[(index+1)%count].ab
			} else {
				h2 = p.ab
			}

			if pointGreater(h0, h2, h1) {
				hull2[count2] = hull[index]
				count2++
			}
		}

		return EPARecurse(ctx, count2, hull2, iteration+1)
	}

	if iteration > warnEpaIterations {
		log.Println("Warning: High EPA iterations:", iteration)
	}

	// Could not find a new point to insert, so we have found the closest edge of the minkowski difference.
	return v0.ClosestPoints(v1)
}

var BuiltinCollisionFuncs = [9]CollisionFunc{
	CircleToCircle,
	CollisionError,
	CollisionError,
	CircleToSegment,
	SegmentToSegment,
	CollisionError,
	CircleToPoly,
	SegmentToPoly,
	PolyToPoly,
}

// Collide performs a collision between two shapes
func Collide(a, b *Shape, collisionID uint32, contacts []Contact) CollisionInfo {
	info := CollisionInfo{
		a:           a,
		b:           b,
		collisionId: collisionID,
		arr:         contacts,
	}

	// Make sure the shape types are in order.
	if a.Order() > b.Order() {
		info.a = b
		info.b = a
	} else {
		info.a = a
		info.b = b
	}

	BuiltinCollisionFuncs[info.a.Order()+info.b.Order()*ShapeTypeNum](&info)
	return info
}

// Return contact information about two shapes.
func ShapesCollideInfo(a, b *Shape) ContactPointSet {
	contacts := make([]Contact, MaxContactsPerArbiter)
	info := Collide(a, b, 0, contacts)

	var set ContactPointSet
	set.Count = info.count

	// Collide may have swapped the contact order, flip the normal.
	swapped := a != info.a
	if swapped {
		set.Normal = info.n.Neg()
	} else {
		set.Normal = info.n
	}

	for i := 0; i < info.count; i++ {
		p1 := contacts[i].R1
		p2 := contacts[i].R2

		if swapped {
			set.Points[i].PointA = p2
			set.Points[i].PointB = p1
		} else {
			set.Points[i].PointA = p1
			set.Points[i].PointB = p2
		}
		set.Points[i].Distance = p2.Sub(p1).Dot(set.Normal)
	}

	return set
}
