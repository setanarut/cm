package cm

import (
	"fmt"

	"github.com/setanarut/v"
)

// Draw flags
const (
	DrawShapes          = 1 << 0
	DrawConstraints     = 1 << 1
	DrawCollisionPoints = 1 << 2
)

// 16 bytes
type FColor struct {
	R, G, B, A float32
}

type IDrawer interface {
	DrawCircle(pos v.Vec, angle, radius float64, outline, fill FColor, data any)
	DrawSegment(a, b v.Vec, fill FColor, data any)
	DrawFatSegment(a, b v.Vec, radius float64, outline, fill FColor, data any)
	DrawPolygon(count int, verts []v.Vec, radius float64, outline, fill FColor, data any)
	DrawDot(size float64, pos v.Vec, fill FColor, data any)

	Flags() uint
	OutlineColor() FColor
	ShapeColor(shape *Shape, data any) FColor
	ConstraintColor() FColor
	CollisionPointColor() FColor
	Data() any
}

// DrawShape draws shapes with the drawer implementation
func DrawShape(shape *Shape, drawer IDrawer) {
	body := shape.Body
	data := drawer.Data()

	outline := drawer.OutlineColor()
	fill := drawer.ShapeColor(shape, data)

	switch shape.Class.(type) {
	case *Circle:
		circle := shape.Class.(*Circle)
		drawer.DrawCircle(circle.transformC, body.angle, circle.radius, outline, fill, data)
	case *Segment:
		seg := shape.Class.(*Segment)
		drawer.DrawFatSegment(seg.transformA, seg.transformB, seg.radius, outline, fill, data)
	case *PolyShape:
		poly := shape.Class.(*PolyShape)

		count := poly.count
		planes := poly.Planes
		verts := make([]v.Vec, count)

		for i := range count {
			verts[i] = planes[i].V0
		}
		drawer.DrawPolygon(count, verts, poly.Radius, outline, fill, data)
	default:
		panic("Unknown shape type")
	}
}

var springVerts = []v.Vec{
	{0.00, 0.0},
	{0.20, 0.0},
	{0.25, 3.0},
	{0.30, -6.0},
	{0.35, 6.0},
	{0.40, -6.0},
	{0.45, 6.0},
	{0.50, -6.0},
	{0.55, 6.0},
	{0.60, -6.0},
	{0.65, 6.0},
	{0.70, -3.0},
	{0.75, 6.0},
	{0.80, 0.0},
	{1.00, 0.0},
}

// DrawConstraint draws constraints with the drawer implementation
func DrawConstraint(constraint *Constraint, drawer IDrawer) {
	data := drawer.Data()
	color := drawer.ConstraintColor()

	bodyA := constraint.bodyA
	bodyB := constraint.bodyB

	switch constraint.Class.(type) {
	case *PinJoint:
		joint := constraint.Class.(*PinJoint)

		a := bodyA.transform.Apply(joint.AnchorA)
		b := bodyB.transform.Apply(joint.AnchorB)

		drawer.DrawDot(5, a, color, data)
		drawer.DrawDot(5, b, color, data)
		drawer.DrawSegment(a, b, color, data)
	case *SlideJoint:
		joint := constraint.Class.(*SlideJoint)

		a := bodyA.transform.Apply(joint.AnchorA)
		b := bodyB.transform.Apply(joint.AnchorB)

		drawer.DrawDot(5, a, color, data)
		drawer.DrawDot(5, b, color, data)
		drawer.DrawSegment(a, b, color, data)
	case *PivotJoint:
		joint := constraint.Class.(*PivotJoint)

		a := bodyA.transform.Apply(joint.AnchorA)
		b := bodyB.transform.Apply(joint.AnchorB)

		drawer.DrawDot(5, a, color, data)
		drawer.DrawDot(5, b, color, data)
	case *GrooveJoint:
		joint := constraint.Class.(*GrooveJoint)

		a := bodyA.transform.Apply(joint.GrooveA)
		b := bodyA.transform.Apply(joint.GrooveB)
		c := bodyB.transform.Apply(joint.AnchorB)

		drawer.DrawDot(5, c, color, data)
		drawer.DrawSegment(a, b, color, data)
	case *DampedSpring:
		spring := constraint.Class.(*DampedSpring)
		a := bodyA.transform.Apply(spring.AnchorA)
		b := bodyB.transform.Apply(spring.AnchorB)

		drawer.DrawDot(5, a, color, data)
		drawer.DrawDot(5, b, color, data)

		delta := b.Sub(a)
		cos := delta.X
		sin := delta.Y
		s := 1.0 / delta.Mag()

		r1 := v.Vec{cos, -sin * s}
		r2 := v.Vec{sin, cos * s}

		verts := []v.Vec{}
		for i := range springVerts {
			vt := springVerts[i]
			verts = append(verts, v.Vec{vt.Dot(r1) + a.X, vt.Dot(r2) + a.Y})
		}

		for i := 0; i < len(springVerts)-1; i++ {
			drawer.DrawSegment(verts[i], verts[i+1], color, data)
		}
	// these aren't drawn in Chipmunk, so they aren't drawn here
	case *GearJoint:
	case *SimpleMotor:
	case *DampedRotarySpring:
	case *RotaryLimitJoint:
	case *RatchetJoint:
	default:
		panic(fmt.Sprintf("Implement me: %#v", constraint.Class))
	}
}

// DrawSpace draws all shapes in space with the drawer implementation
func DrawSpace(space *Space, drawer IDrawer) {
	space.dynamicShapes.class.Each(func(obj *Shape) {
		DrawShape(obj, drawer)
	})
	space.staticShapes.class.Each(func(obj *Shape) {
		DrawShape(obj, drawer)
	})

	for _, constraint := range space.constraints {
		DrawConstraint(constraint, drawer)
	}

	drawSeg := drawer.DrawSegment
	data := drawer.Data()

	for _, arb := range space.Arbiters {
		n := arb.normal

		for j := 0; j < arb.count; j++ {
			p1 := arb.bodyA.position.Add(arb.Contacts[j].R1)
			p2 := arb.bodyB.position.Add(arb.Contacts[j].R2)

			a := p1.Add(n.Scale(-2))
			b := p2.Add(n.Scale(2))
			drawSeg(a, b, drawer.CollisionPointColor(), data)
		}
	}
}
