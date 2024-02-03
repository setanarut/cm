package drawer

import (
	"fmt"
	"image/color"

	"github.com/setanarut/cm"
)

// Draw flags
const (
	DRAW_SHAPES           = 1 << 0
	DRAW_CONSTRAINTS      = 1 << 1
	DRAW_COLLISION_POINTS = 1 << 2
)

type Drawer interface {
	DrawCircle(pos cm.Vector, angle, radius float64, outline, fill color.RGBA, data interface{})
	DrawSegment(a, b cm.Vector, fill color.RGBA, data interface{})
	DrawFatSegment(a, b cm.Vector, radius float64, outline, fill color.RGBA, data interface{})
	DrawPolygon(count int, verts []cm.Vector, radius float64, outline, fill color.RGBA, data interface{})
	DrawDot(size float64, pos cm.Vector, fill color.RGBA, data interface{})

	Flags() uint
	OutlineColor() color.RGBA
	ShapeColor(shape *cm.Shape, data interface{}) color.RGBA
	ConstraintColor() color.RGBA
	CollisionPointColor() color.RGBA
	Data() interface{}
}

func DrawShape(shape *cm.Shape, options Drawer) {
	body := shape.Body()
	data := options.Data()

	outline := options.OutlineColor()
	fill := options.ShapeColor(shape, data)

	switch shape.Class.(type) {
	case *cm.Circle:
		circle := shape.Class.(*cm.Circle)
		options.DrawCircle(circle.TransformC(), body.Angle(), circle.Radius(), outline, fill, data)
	case *cm.Segment:
		seg := shape.Class.(*cm.Segment)
		options.DrawFatSegment(seg.TransformA(), seg.TransformB(), seg.Radius(), outline, fill, data)
	case *cm.PolyShape:
		poly := shape.Class.(*cm.PolyShape)

		count := poly.Count()
		verts := make([]cm.Vector, count)

		for i := 0; i < count; i++ {
			verts[i] = poly.TransformVert(i)
		}
		options.DrawPolygon(count, verts, poly.Radius(), outline, fill, data)
	default:
		panic("Unknown shape type")
	}
}

var springVerts = []cm.Vector{
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

func DrawConstraint(constraint *cm.Constraint, options Drawer) {
	data := options.Data()
	color := options.ConstraintColor()

	bodyA := constraint.BodyA()
	bodyB := constraint.BodyB()

	bodyATransform := bodyA.Transform()
	bodyBTransform := bodyB.Transform()

	switch constraint.Class.(type) {
	case *cm.PinJoint:
		joint := constraint.Class.(*cm.PinJoint)

		a := bodyATransform.Point(joint.AnchorA)
		b := bodyB.Transform().Point(joint.AnchorB)

		options.DrawDot(5, a, color, data)
		options.DrawDot(5, b, color, data)
		options.DrawSegment(a, b, color, data)
	case *cm.SlideJoint:
		joint := constraint.Class.(*cm.SlideJoint)

		a := bodyATransform.Point(joint.AnchorA)
		b := bodyBTransform.Point(joint.AnchorB)

		options.DrawDot(5, a, color, data)
		options.DrawDot(5, b, color, data)
		options.DrawSegment(a, b, color, data)
	case *cm.PivotJoint:
		joint := constraint.Class.(*cm.PivotJoint)

		a := bodyATransform.Point(joint.AnchorA)
		b := bodyBTransform.Point(joint.AnchorB)

		options.DrawDot(5, a, color, data)
		options.DrawDot(5, b, color, data)
	case *cm.GrooveJoint:
		joint := constraint.Class.(*cm.GrooveJoint)

		a := bodyATransform.Point(joint.GrooveA)
		b := bodyATransform.Point(joint.GrooveB)
		c := bodyBTransform.Point(joint.AnchorB)

		options.DrawDot(5, c, color, data)
		options.DrawSegment(a, b, color, data)
	case *cm.DampedSpring:
		spring := constraint.Class.(*cm.DampedSpring)
		a := bodyATransform.Point(spring.AnchorA)
		b := bodyBTransform.Point(spring.AnchorB)

		options.DrawDot(5, a, color, data)
		options.DrawDot(5, b, color, data)

		delta := b.Sub(a)
		cos := delta.X
		sin := delta.Y
		s := 1.0 / delta.Length()

		r1 := cm.Vector{cos, -sin * s}
		r2 := cm.Vector{sin, cos * s}

		verts := []cm.Vector{}
		for i := 0; i < len(springVerts); i++ {
			v := springVerts[i]
			verts = append(verts, cm.Vector{v.Dot(r1) + a.X, v.Dot(r2) + a.Y})
		}

		for i := 0; i < len(springVerts)-1; i++ {
			options.DrawSegment(verts[i], verts[i+1], color, data)
		}
	// these aren't drawn in Chipmunk, so they aren't drawn here
	case *cm.GearJoint:
	case *cm.SimpleMotor:
	case *cm.DampedRotarySpring:
	case *cm.RotaryLimitJoint:
	case *cm.RatchetJoint:
	default:
		panic(fmt.Sprintf("Implement me: %#v", constraint.Class))
	}
}

func DrawSpace(space *cm.Space, options Drawer) {
	space.EachShape(func(obj *cm.Shape) {
		DrawShape(obj, options)
	})

	space.EachConstraint(func(c *cm.Constraint) {
		DrawConstraint(c, options)
	})

	// drawSeg := options.DrawSegment
	// data := options.Data()

	// for _, arb := range space.Arbiters() {
	// 	n := arb.Normal()

	// 	for j := 0; j < arb.Count(); j++ {
	// 		p1 := arb.body_a.p.Add(arb.contacts[j].r1)
	// 		p2 := arb.body_b.p.Add(arb.contacts[j].r2)

	// 		a := p1.Add(n.Mult(-2))
	// 		b := p2.Add(n.Mult(2))
	// 		drawSeg(a, b, options.CollisionPointColor(), data)
	// 	}
	// }
}
