package cm

import (
	"fmt"
	"log"
)

type Shaper interface {
	Body() *Body
	MassInfo() *ShapeMassInfo
	HashId() HashValue
	SetHashId(HashValue)
	SetSpace(*Space)
	BB() BB
	SetBB(BB)
}

type ShapeClass interface {
	CacheData(transform Transform) BB
	PointQuery(p Vector, info *PointQueryInfo)
	SegmentQuery(a, b Vector, radius float64, info *SegmentQueryInfo)
}

const (
	SHAPE_TYPE_NUM = 3
)

type Shape struct {
	Class    ShapeClass
	Filter   ShapeFilter
	UserData interface{}

	space    *Space
	body     *Body
	massInfo *ShapeMassInfo
	bb       BB
	// sensor is a boolean value if this shape is a sensor or not.
	// Sensors only call collision callbacks, and never generate real collisions.
	sensor               bool
	elasticity, friction float64
	// The surface velocity of the object. Useful for creating conveyor belts or players that move around.
	// This value is only used when calculating friction, not resolving the collision.
	surfaceVelocity Vector
	// You can assign types to collision shapes that trigger callbacks when objects of certain types touch.
	collisionType CollisionType

	hashid HashValue
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

// Sensor returns this shape is a sensor or not.
func (s *Shape) Sensor() bool {
	return s.sensor
}

// SetSensor sets sensor. A boolean value if this shape is a sensor or not.
// Sensors only call collision callbacks, and never generate real collisions.
func (s *Shape) SetSensor(sensor bool) {
	s.body.Activate()
	s.sensor = sensor
}

func (s *Shape) Space() *Space {
	return s.space
}

func (s *Shape) Body() *Body {
	return s.body
}

func (s *Shape) MassInfo() *ShapeMassInfo {
	return s.massInfo
}

func (s *Shape) Mass() float64 {
	return s.massInfo.m
}

func (s *Shape) SetMass(mass float64) {
	s.body.Activate()

	s.massInfo.m = mass
	s.body.AccumulateMassFromShapes()
}

func (s *Shape) Density() float64 {
	return s.massInfo.m / s.massInfo.area
}

func (s *Shape) SetDensity(density float64) {
	s.SetMass(density * s.massInfo.area)
}

func (s *Shape) Moment() float64 {
	return s.massInfo.m * s.massInfo.i
}

func (s *Shape) Area() float64 {
	return s.massInfo.area
}

func (s *Shape) CenterOfGravity() Vector {
	return s.massInfo.cog
}

func (s *Shape) HashId() HashValue {
	return s.hashid
}

func (s *Shape) SetHashId(hashid HashValue) {
	s.hashid = hashid
}

func (s *Shape) SetSpace(space *Space) {
	s.space = space
}

// BB returns 2D axis-aligned bounding box of this shape.
func (s *Shape) BB() BB {
	return s.bb
}

func (s *Shape) SetBB(bb BB) {
	s.bb = bb
}

// SetCollisionType sets collision type.
// You can assign types to shapes that trigger callbacks (CollisionHandler) when objects of certain types touch
func (s *Shape) SetCollisionType(collisionType CollisionType) {
	s.body.Activate()
	s.collisionType = collisionType
}

func (s *Shape) Friction() float64 {
	return s.friction
}

func (s *Shape) SetFriction(u float64) {
	if s.friction < 0 {
		log.Fatalln("Friction must be positive")
	}
	s.body.Activate()
	s.friction = u
}

// SurfaceVelocity returns the surface velocity of this shape.
func (s *Shape) SurfaceVelocity() Vector {
	return s.surfaceVelocity
}

// SetSurfaceVelocity sets the surface velocity of the object. Useful for creating conveyor belts or players that move around.
// This value is only used when calculating friction, not resolving the collision.
func (s *Shape) SetSurfaceVelocity(surfaceV Vector) {
	s.surfaceVelocity = surfaceV
}

func (s *Shape) Elasticity() float64 {
	return s.elasticity
}

// SetElasticity sets elasticity (0-1 range)
func (s *Shape) SetElasticity(e float64) {
	if s.elasticity < 0 {
		log.Fatalln("Elasticity must be positive")
	}
	s.body.Activate()
	s.elasticity = e
}

func (s *Shape) SetShapeFilter(filter ShapeFilter) {
	s.body.Activate()
	s.Filter = filter
}

func (s *Shape) CacheBB() BB {
	return s.Update(s.body.transform)
}

func (s *Shape) Update(transform Transform) BB {
	s.bb = s.Class.CacheData(transform)
	return s.bb
}

func (s *Shape) Point(i uint32) SupportPoint {
	switch s.Class.(type) {
	case *Circle:
		return NewSupportPoint(s.Class.(*Circle).transformC, 0)
	case *Segment:
		seg := s.Class.(*Segment)
		if i == 0 {
			return NewSupportPoint(seg.transformA, i)
		}
		return NewSupportPoint(seg.transformB, i)
	case *PolyShape:
		poly := s.Class.(*PolyShape)
		// Poly shapes may change vertex count.
		var index int
		if i < uint32(poly.count) {
			index = int(i)
		}
		return NewSupportPoint(poly.planes[index].v0, uint32(index))
	default:
		return NewSupportPoint(Vector{}, 0)
	}
}

func (s *Shape) PointQuery(p Vector) PointQueryInfo {
	info := PointQueryInfo{nil, Vector{}, INFINITY, Vector{}}
	s.Class.PointQuery(p, &info)
	return info
}

func (shape *Shape) SegmentQuery(a, b Vector, radius float64, info *SegmentQueryInfo) bool {
	blank := SegmentQueryInfo{nil, b, Vector{}, 1}
	if info != nil {
		*info = blank
	} else {
		info = &blank
	}

	var nearest PointQueryInfo
	shape.Class.PointQuery(a, &nearest)
	if nearest.Distance <= radius {
		info.Shape = shape
		info.Alpha = 0
		info.Normal = a.Sub(nearest.Point).Normalize()
	} else {
		shape.Class.SegmentQuery(a, b, radius, info)
	}

	return info.Shape != nil
}

func NewShape(class ShapeClass, body *Body, massInfo *ShapeMassInfo) *Shape {
	return &Shape{
		Class:    class,
		body:     body,
		massInfo: massInfo,

		surfaceVelocity: Vector{},
		Filter: ShapeFilter{
			Group:      NO_GROUP,
			Categories: ALL_CATEGORIES,
			Mask:       ALL_CATEGORIES,
		},
	}
}

func ShapesCollide(a, b *Shape) ContactPointSet {
	contacts := make([]Contact, MAX_CONTACTS_PER_ARBITER)
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
