package cm

import (
	"fmt"

	"github.com/setanarut/vec"
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
	DrawCircle(pos vec.Vec2, angle, radius float64, outline, fill FColor, data interface{})
	DrawSegment(a, b vec.Vec2, fill FColor, data interface{})
	DrawFatSegment(a, b vec.Vec2, radius float64, outline, fill FColor, data interface{})
	DrawPolygon(count int, verts []vec.Vec2, radius float64, outline, fill FColor, data interface{})
	DrawDot(size float64, pos vec.Vec2, fill FColor, data interface{})

	Flags() uint
	OutlineColor() FColor
	ShapeColor(shape *Shape, data interface{}) FColor
	ConstraintColor() FColor
	CollisionPointColor() FColor
	Data() interface{}
}

func DrawShape(shape *Shape, options IDrawer) {
	body := shape.body
	data := options.Data()

	outline := options.OutlineColor()
	fill := options.ShapeColor(shape, data)

	switch shape.Class.(type) {
	case *Circle:
		circle := shape.Class.(*Circle)
		options.DrawCircle(circle.transformC, body.angle, circle.radius, outline, fill, data)
	case *Segment:
		seg := shape.Class.(*Segment)
		options.DrawFatSegment(seg.transformA, seg.transformB, seg.radius, outline, fill, data)
	case *PolyShape:
		poly := shape.Class.(*PolyShape)

		count := poly.count
		planes := poly.planes
		verts := make([]vec.Vec2, count)

		for i := 0; i < count; i++ {
			verts[i] = planes[i].v0
		}
		options.DrawPolygon(count, verts, poly.radius, outline, fill, data)
	default:
		panic("Unknown shape type")
	}
}

var springVerts = []vec.Vec2{
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

func DrawConstraint(constraint *Constraint, options IDrawer) {
	data := options.Data()
	color := options.ConstraintColor()

	bodyA := constraint.bodyA
	bodyB := constraint.bodyB

	switch constraint.Class.(type) {
	case *PinJoint:
		joint := constraint.Class.(*PinJoint)

		a := bodyA.transform.Point(joint.AnchorA)
		b := bodyB.transform.Point(joint.AnchorB)

		options.DrawDot(5, a, color, data)
		options.DrawDot(5, b, color, data)
		options.DrawSegment(a, b, color, data)
	case *SlideJoint:
		joint := constraint.Class.(*SlideJoint)

		a := bodyA.transform.Point(joint.AnchorA)
		b := bodyB.transform.Point(joint.AnchorB)

		options.DrawDot(5, a, color, data)
		options.DrawDot(5, b, color, data)
		options.DrawSegment(a, b, color, data)
	case *PivotJoint:
		joint := constraint.Class.(*PivotJoint)

		a := bodyA.transform.Point(joint.AnchorA)
		b := bodyB.transform.Point(joint.AnchorB)

		options.DrawDot(5, a, color, data)
		options.DrawDot(5, b, color, data)
	case *GrooveJoint:
		joint := constraint.Class.(*GrooveJoint)

		a := bodyA.transform.Point(joint.GrooveA)
		b := bodyA.transform.Point(joint.GrooveB)
		c := bodyB.transform.Point(joint.AnchorB)

		options.DrawDot(5, c, color, data)
		options.DrawSegment(a, b, color, data)
	case *DampedSpring:
		spring := constraint.Class.(*DampedSpring)
		a := bodyA.transform.Point(spring.AnchorA)
		b := bodyB.transform.Point(spring.AnchorB)

		options.DrawDot(5, a, color, data)
		options.DrawDot(5, b, color, data)

		delta := b.Sub(a)
		cos := delta.X
		sin := delta.Y
		s := 1.0 / delta.Mag()

		r1 := vec.Vec2{cos, -sin * s}
		r2 := vec.Vec2{sin, cos * s}

		verts := []vec.Vec2{}
		for i := 0; i < len(springVerts); i++ {
			v := springVerts[i]
			verts = append(verts, vec.Vec2{v.Dot(r1) + a.X, v.Dot(r2) + a.Y})
		}

		for i := 0; i < len(springVerts)-1; i++ {
			options.DrawSegment(verts[i], verts[i+1], color, data)
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

func DrawSpace(space *Space, options IDrawer) {
	space.dynamicShapes.class.Each(func(obj *Shape) {
		DrawShape(obj, options)
	})
	space.staticShapes.class.Each(func(obj *Shape) {
		DrawShape(obj, options)
	})

	for _, constraint := range space.constraints {
		DrawConstraint(constraint, options)
	}

	drawSeg := options.DrawSegment
	data := options.Data()

	for _, arb := range space.Arbiters {
		n := arb.normal

		for j := 0; j < arb.count; j++ {
			p1 := arb.body_a.position.Add(arb.contacts[j].r1)
			p2 := arb.body_b.position.Add(arb.contacts[j].r2)

			a := p1.Add(n.Scale(-2))
			b := p2.Add(n.Scale(2))
			drawSeg(a, b, options.CollisionPointColor(), data)
		}
	}
}
