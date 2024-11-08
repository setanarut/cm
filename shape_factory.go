package cm

import (
	"math"

	"github.com/setanarut/vec"
)

// NewSegmentShape returns a new Segment shape using two points 'a' and 'b'
// and rounding radius 'r'. The shape is attached to the given body.
//
// Parameters:
//   - body: The body to which the shape will be attached.
//   - a: The first point of the segment.
//   - b: The second point of the segment.
//   - r: The radius of the segment shape.
func NewSegmentShape(body *Body, a, b vec.Vec2, r float64) *Shape {
	segment := &Segment{
		a: a,
		b: b,
		n: b.Sub(a).Unit().ReversePerp(),

		radius:   r,
		aTangent: vec.Vec2{},
		bTangent: vec.Vec2{},
	}
	segment.Shape = NewShape(segment, body, NewSegmentMassInfo(0, a, b, r))
	body.AttachShape(segment.Shape)
	return segment.Shape
}

// NewBoxShape returns a Box shape with specified width 'w',
// height 'h', and rounding radius 'roundingRadius'. The shape is attached to the body.
//
// Parameters:
//   - body: The body to which the shape will be attached.
//   - w: The width of the box.
//   - h: The height of the box.
//   - r: The rounding radius of the box corners.
func NewBoxShape(body *Body, w, h, r float64) *Shape {
	hw := w / 2.0
	hh := h / 2.0
	bb := &BB{-hw, -hh, hw, hh}
	verts := []vec.Vec2{
		{bb.R, bb.B},
		{bb.R, bb.T},
		{bb.L, bb.T},
		{bb.L, bb.B},
	}
	shape := NewPolyShapeRaw(body, 4, verts, r)
	body.AttachShape(shape)
	return shape
}

// NewBoxShape2 returns a Box shape using a bounding box 'bb'
// and a rounding radius 'roundingRadius'. The shape is attached to the body.
//
// Parameters:
//   - body: The body to which the shape will be attached.
//   - bb: The bounding box defining the shape dimensions.
//   - r: The rounding radius of the box corners.
func NewBoxShape2(body *Body, bb BB, r float64) *Shape {
	verts := []vec.Vec2{
		{bb.R, bb.B},
		{bb.R, bb.T},
		{bb.L, bb.T},
		{bb.L, bb.B},
	}
	shape := NewPolyShapeRaw(body, 4, verts, r)
	body.AttachShape(shape)
	return shape
}

// NewCircleShape returns a Circle shape with a specified radius and offset.
//
// The shape is attached to the given body.
//
// Parameters:
//   - b: The body to which the shape will be attached.
//   - r: The radius of the circle.
//   - offset: The position of the circle's center.
func NewCircleShape(b *Body, r float64, offset vec.Vec2) *Shape {
	circle := &Circle{
		c:      offset,
		radius: r,
	}
	circle.Shape = NewShape(circle, b, CircleShapeMassInfo(0, r, offset))
	b.AttachShape(circle.Shape)
	return circle.Shape
}

// NewPolyShape returns a Polygon shape with a given vertices.
//
// Parameters:
//   - b: The body to which the shape will be attached.
//   - vertices: The list of vertex positions.
//   - t: The transformation applied to the vertex positions.
//   - r: The rounding radius of the polygon corners.
func NewPolyShape(b *Body, vertices []vec.Vec2, t Transform, r float64) *Shape {

	hullVerts := []vec.Vec2{}
	// Transform the verts before building the hull in case of a negative scale.
	for _, v := range vertices {
		hullVerts = append(hullVerts, t.Apply(v))
	}

	hullCount := convexHull(len(vertices), hullVerts, nil, 0)
	plshape := NewPolyShapeRaw(b, hullCount, hullVerts, r)
	b.AttachShape(plshape)
	return plshape
}

// Initialize a polygon shape with rounded corners.
// The vertexes must be convex with a counter-clockwise winding.
func NewPolyShapeRaw(
	body *Body,
	count int,
	verts []vec.Vec2,
	roundingRadius float64,
) *Shape {
	poly := &PolyShape{
		Radius: roundingRadius,
		count:  count,
		Planes: []SplittingPlane{},
	}
	poly.Shape = NewShape(poly, body, PolyShapeMassInfo(0, count, verts, roundingRadius))
	poly.SetVerts(count, verts)
	return poly.Shape
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
// d1 and d2 are the inner and outer diameters. A solid circle has an inner
// diameter (d1) of 0.
// offset is a Vec2 representing the displacement of the circle's center of
// mass from the axis of rotation. If the center is not aligned with the axis,
// the offset increases the moment of inertia.
func MomentForCircle(mass, d1, d2 float64, offset vec.Vec2) float64 {
	return mass * (0.5*(d1*d1+d2*d2) + offset.LengthSq())
}

// MomentForCircle calculates the moment of inertia for a solid circle with
// radius and mass
func MomentForCircle2(mass, radius float64) float64 {
	return 0.5 * mass * math.Pow(radius*2, 2)
}

// MomentForSegment calculates the moment of inertia for a line segment.
//
// Beveling radius is not supported.
func MomentForSegment(mass float64, a, b vec.Vec2, radius float64) float64 {
	offset := a.Lerp(b, 0.5)
	length := b.Distance(a) + 2.0*radius
	return mass * ((length*length+4.0*radius*radius)/12.0 + offset.LengthSq())
}

// MomentForPoly calculates the moment of inertia for a solid polygon shape
// assuming it's center of gravity is at it's centroid.
//
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
