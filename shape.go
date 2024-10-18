package cm

import (
	"fmt"

	"github.com/setanarut/vec"
)

type IShape interface {
	CacheData(transform Transform) BB
	PointQuery(p vec.Vec2, info *PointQueryInfo)
	SegmentQuery(a, b vec.Vec2, radius float64, info *SegmentQueryInfo)
}

const (
	ShapeTypeNum = 3
)

type Shape struct {
	Class    IShape
	UserData any
	Filter   ShapeFilter
	// You can assign types to collision shapes that triggercallbacks when objects
	// of certain types touch.
	CollisionType CollisionType
	// Sensor is a boolean value if this shape is a Sensor or not.
	// Sensors only call collision callbacks, and never generate real collisions.
	Sensor bool
	// The surface velocity of the object. Useful for creating conveyor belts or
	// players that move around. This value is only used when calculating friction,
	// not resolving the collision.
	SurfaceVelocity      vec.Vec2
	Elasticity, Friction float64
	BB                   BB
	Space                *Space
	body                 *Body
	massInfo             *ShapeMassInfo
	hashid               HashValue
}

func (s Shape) String() string {
	return fmt.Sprintf("%T", s.Class)
}

func (s *Shape) Order() int {
	switch s.Class.(type) {
	case *Circle:
		return 0
	case *Segment:
		return 1
	case *PolyShape:
		return 2
	default:
		return 3
	}
}

// // Sensor returns this shape is a sensor or not.
// func (s *Shape) Sensor() bool {
// 	return s.sensor
// }

// SetSensor wakes up sleeping or idle body then sets Shape.Sensor.
// sensor is a boolean value if this shape is a sensor or not.
// Sensors only call collision callbacks, and never generate real collisions.
func (sh *Shape) SetSensor(sensor bool) {
	sh.body.Activate()
	sh.Sensor = sensor
}

func (sh *Shape) Body() *Body {
	return sh.body
}

func (sh *Shape) MassInfo() *ShapeMassInfo {
	return sh.massInfo
}

func (sh *Shape) Mass() float64 {
	return sh.massInfo.m
}

// SetMass wakes up sleeping or idle body then sets mass
func (sh *Shape) SetMass(mass float64) {
	sh.body.Activate()
	sh.massInfo.m = mass
	sh.body.AccumulateMassFromShapes()
}

func (sh *Shape) Density() float64 {
	return sh.massInfo.m / sh.massInfo.area
}

func (sh *Shape) SetDensity(density float64) {
	sh.SetMass(density * sh.massInfo.area)
}

func (sh *Shape) Moment() float64 {
	return sh.massInfo.m * sh.massInfo.i
}

func (sh *Shape) Area() float64 {
	return sh.massInfo.area
}

func (sh *Shape) CenterOfGravity() vec.Vec2 {
	return sh.massInfo.cog
}

func (sh *Shape) HashId() HashValue {
	return sh.hashid
}

func (sh *Shape) SetHashId(hashid HashValue) {
	sh.hashid = hashid
}

// SetCollisionType sets collision type.
// You can assign types to shapes that trigger callbacks (CollisionHandler) when
// objects of certain types touch
func (sh *Shape) SetCollisionType(collisionType CollisionType) {
	sh.body.Activate()
	sh.CollisionType = collisionType
}

func (sh *Shape) SetFriction(u float64) {
	sh.body.Activate()
	sh.Friction = u
}

// SetElasticity sets elasticity (0-1 range)
func (sh *Shape) SetElasticity(e float64) {
	sh.body.Activate()
	sh.Elasticity = e
}

func (sh *Shape) SetShapeFilter(filter ShapeFilter) {
	sh.body.Activate()
	sh.Filter = filter
}

func (sh *Shape) CacheBB() BB {
	return sh.Update(sh.body.transform)
}

func (sh *Shape) Update(transform Transform) BB {
	sh.BB = sh.Class.CacheData(transform)
	return sh.BB
}

func (sh *Shape) Point(i uint32) SupportPoint {
	switch sh.Class.(type) {
	case *Circle:
		return NewSupportPoint(sh.Class.(*Circle).transformC, 0)
	case *Segment:
		seg := sh.Class.(*Segment)
		if i == 0 {
			return NewSupportPoint(seg.transformA, i)
		}
		return NewSupportPoint(seg.transformB, i)
	case *PolyShape:
		poly := sh.Class.(*PolyShape)
		// Poly shapes may change vertex count.
		var index int
		if i < uint32(poly.count) {
			index = int(i)
		}
		return NewSupportPoint(poly.planes[index].v0, uint32(index))
	default:
		return NewSupportPoint(vec.Vec2{}, 0)
	}
}

// Perform a nearest point query.
//
// It finds the closest point on the surface of shape to a specific point.
// The value returned is the distance between the points.
// A negative distance means the point is inside the shape.
func (sh *Shape) PointQuery(p vec.Vec2) PointQueryInfo {
	info := PointQueryInfo{nil, vec.Vec2{}, infinity, vec.Vec2{}}
	sh.Class.PointQuery(p, &info)
	return info
}

// Perform a segment query against a shape.
//
// info must be a pointer to a valid SegmentQueryInfo structure.
func (sh *Shape) SegmentQuery(a, b vec.Vec2, radius float64, info *SegmentQueryInfo) bool {
	blank := SegmentQueryInfo{nil, b, vec.Vec2{}, 1}
	if info != nil {
		*info = blank
	} else {
		info = &blank
	}

	var nearest PointQueryInfo
	sh.Class.PointQuery(a, &nearest)
	if nearest.Distance <= radius {
		info.Shape = sh
		info.Alpha = 0
		info.Normal = a.Sub(nearest.Point).Unit()
	} else {
		sh.Class.SegmentQuery(a, b, radius, info)
	}

	return info.Shape != nil
}

func NewShape(class IShape, body *Body, massInfo *ShapeMassInfo) *Shape {
	return &Shape{
		Class:    class,
		body:     body,
		massInfo: massInfo,

		SurfaceVelocity: vec.Vec2{},
		Filter: ShapeFilter{
			Group:      NoGroup,
			Categories: AllCategories,
			Mask:       AllCategories,
		},
	}
}

// Return contact information about two shapes.
func ShapesCollide(a, b *Shape) ContactPointSet {
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
		p1 := contacts[i].r1
		p2 := contacts[i].r2

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
