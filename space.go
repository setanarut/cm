package cm

import (
	"log"
	"math"
	"slices"
	"sync"
	"unsafe"

	"github.com/setanarut/v"
)

const (
	MaxContactsPerArbiter int = 2
	ContactsBufferSize    int = 1024
)

type Space struct {
	UserData any

	// Iterations is number of iterations to use in the impulse solver to solve
	// contacts and other constrain. Must be non-zero.
	Iterations uint

	// IdleSpeedThreshold is speed threshold for a body to be considered idle.
	// The default value of 0 means to let the space guess a good threshold based on gravity.
	IdleSpeedThreshold float64

	// SleepTimeThreshold is time a group of bodies must remain idle in order to fall asleep.
	// Enabling sleeping also implicitly enables the the contact graph.
	// The default value of INFINITY disables the sleeping algorithm.
	SleepTimeThreshold float64

	// StaticBody is the Space provided static body for a given s.
	// This is merely provided for convenience and you are not required to use it.
	StaticBody *Body

	// Gravity to pass to rigid bodies when integrating velocity.
	Gravity v.Vec

	// Damping rate expressed as the fraction of velocity bodies retain each second.
	//
	// A value of 0.9 would mean that each body's velocity will drop 10% per second.
	// The default value is 1.0, meaning no Damping is applied.
	// @note This Damping value is different than those of DampedSpring and DampedRotarySpring.
	Damping float64

	// CollisionSlop is amount of encouraged penetration between colliding shapes.
	//
	// Used to reduce oscillating contacts and keep the collision cache warm.
	// Defaults to 0.1. If you have poor simulation quality,
	// increase this number as much as possible without allowing visible amounts of overlap.
	CollisionSlop float64

	// CollisionBias determines how fast overlapping shapes are pushed apart.
	//
	// Expressed as a fraction of the error remaining after each second.
	// Defaults to math.Pow(0.9, 60) meaning that Chipmunk fixes 10% of overlap each frame at 60Hz.
	CollisionBias float64

	// Number of frames that contact information should persist.
	// Defaults to 3. There is probably never a reason to change this value.
	CollisionPersistence uint
	Arbiters             []*Arbiter
	DynamicBodies        []*Body
	StaticBodies         []*Body

	// private
	rousedBodies       []*Body
	sleepingComponents []*Body
	staticShapes       *SpatialIndex
	dynamicShapes      *SpatialIndex
	stamp              uint
	currDT             float64
	shapeIDCounter     uint
	constraints        []*Constraint
	contactBuffersHead *ContactBuffer
	cachedArbiters     *HashSet[ShapePair, *Arbiter]
	pooledArbiters     sync.Pool
	locked             bool
	usesWildcards      bool
	collisionHandlers  *HashSet[*CollisionHandler, *CollisionHandler]
	defaultHandler     *CollisionHandler
	PostStepCallbacks  []*PostStepCallback
	skipPostStep       bool
}

// NewSpace allocates and initializes a Space
func NewSpace() *Space {
	space := &Space{
		Iterations:           10,
		IdleSpeedThreshold:   0.0,
		SleepTimeThreshold:   math.MaxFloat64,
		StaticBody:           NewBody(0, 0),
		Gravity:              v.Vec{},
		Damping:              1.0,
		CollisionSlop:        0.1,
		CollisionBias:        math.Pow(0.9, 60),
		CollisionPersistence: 3,
		DynamicBodies:        []*Body{},
		StaticBodies:         []*Body{},
		Arbiters:             []*Arbiter{},
		locked:               false,
		stamp:                0,
		shapeIDCounter:       1,
		staticShapes:         NewBBTree(ShapeGetBB, nil),
		sleepingComponents:   []*Body{},
		rousedBodies:         []*Body{},
		cachedArbiters:       NewHashSet(arbiterSetEql),
		pooledArbiters:       sync.Pool{New: func() any { return &Arbiter{} }},
		constraints:          []*Constraint{},
		collisionHandlers: NewHashSet(func(a, b *CollisionHandler) bool {
			if a.TypeA == b.TypeA && a.TypeB == b.TypeB {
				return true
			}
			if a.TypeB == b.TypeA && a.TypeA == b.TypeB {
				return true
			}
			return false
		}),
		PostStepCallbacks: []*PostStepCallback{},
		defaultHandler:    &CollisionHandlerDoNothing,
	}
	for range pooledBufferSize {
		space.pooledArbiters.Put(&Arbiter{})
	}
	space.dynamicShapes = NewBBTree(ShapeGetBB, space.staticShapes)
	space.dynamicShapes.class.(*BBTree).velocityFunc = BBTreeVelocityFunc(ShapeVelocityFunc)
	space.StaticBody.SetType(Static)
	return space
}

// DynamicBodyCount returns the total number of dynamic bodies in space
func (s *Space) DynamicBodyCount() int {
	return len(s.DynamicBodies)
}

// StaticBodyCount returns the total number of static bodies in space
func (s *Space) StaticBodyCount() int {
	return len(s.StaticBodies)
}

// SetGravity sets gravity and wake up all of the sleeping bodies since the gravity changed.
func (s *Space) SetGravity(gravity v.Vec) {
	s.Gravity = gravity

	// Wake up all of the bodies since the gravity changed.
	for _, component := range s.sleepingComponents {
		component.Activate()
	}
}

func (s *Space) SetStaticBody(body *Body) {
	if s.StaticBody != nil {
		s.StaticBody.Space = nil
		panic(`Internal Error: Changing the designated static
		body while the old one still had shapes attached.`)
	}
	s.StaticBody = body
	body.Space = s
}

func (s *Space) Activate(body *Body) {

	if s.locked {
		if !Contains(s.rousedBodies, body) {
			s.rousedBodies = append(s.rousedBodies, body)
		}
		return
	}

	s.DynamicBodies = append(s.DynamicBodies, body)

	for _, shape := range body.Shapes {
		s.staticShapes.class.Remove(shape, shape.hashid)
		s.dynamicShapes.class.Insert(shape, shape.hashid)
	}

	for arbiter := body.arbiterList; arbiter != nil; arbiter = arbiter.Next(body) {
		bodyA := arbiter.bodyA

		// Arbiters are shared between two bodies that are always woken up together.
		// You only want to restore the arbiter once, so bodyA is arbitrarily chosen to own the arbiter.
		// The edge case is when static bodies are involved as the static bodies never actually sleep.
		// If the static body is bodyB then all is good. If the static body is bodyA, that can easily be checked.
		if body == bodyA || bodyA.Type() == Static {
			numContacts := arbiter.count
			contacts := arbiter.Contacts

			// Restore contact values back to the space's contact buffer memory
			arbiter.Contacts = s.ContactBufferGetArray()[:numContacts]
			copy(arbiter.Contacts, contacts)
			s.PushContacts(numContacts)

			// reinsert the arbiter into the arbiter cache
			a := arbiter.shapeA
			b := arbiter.shapeB
			shapePair := ShapePair{a, b}
			arbHashId := HashPair(HashValue(unsafe.Pointer(a)), HashValue(unsafe.Pointer(b)))
			s.cachedArbiters.Insert(arbHashId, shapePair, func(_ ShapePair) *Arbiter {
				return arbiter
			})

			// update arbiters state
			arbiter.stamp = s.stamp
			s.Arbiters = append(s.Arbiters, arbiter)
		}
	}

	for constraint := body.constraintList; constraint != nil; constraint = constraint.Next(body) {
		if body == constraint.bodyA || constraint.bodyA.Type() == Static {
			s.constraints = append(s.constraints, constraint)
		}
	}
}

func (s *Space) Deactivate(body *Body) {
	s.DynamicBodies = slices.DeleteFunc(s.DynamicBodies, func(b *Body) bool {
		return b == body
	})

	for _, shape := range body.Shapes {
		s.dynamicShapes.class.Remove(shape, shape.hashid)
		s.staticShapes.class.Insert(shape, shape.hashid)
	}

	for arb := body.arbiterList; arb != nil; arb = ArbiterNext(arb, body) {
		bodyA := arb.bodyA
		if body == bodyA || bodyA.Type() == Static {
			s.UncacheArbiter(arb)
			// Save contact values to a new block of memory so they won't time out
			contacts := make([]Contact, arb.count)
			copy(contacts, arb.Contacts[:arb.count])
			arb.Contacts = contacts

		}
	}

	for constraint := body.constraintList; constraint != nil; constraint = constraint.Next(body) {
		bodyA := constraint.bodyA
		if body == bodyA || bodyA.Type() == Static {
			s.constraints = slices.DeleteFunc(s.constraints, func(c *Constraint) bool {
				return c == constraint
			})
		}
	}
}

// AddShape adds a collision shape to the simulation.
//
// If the shape is attached to a static body, it will be added as a static shape
func (s *Space) AddShape(shape *Shape) *Shape {
	isStatic := shape.Body.Type() == Static
	if !isStatic {
		shape.Body.Activate()
	}
	// shape.Body.AppendShape(shape)

	shape.SetHashId(HashValue(s.shapeIDCounter))
	s.shapeIDCounter += 1
	shape.Update(shape.Body.transform)

	if isStatic {
		s.staticShapes.class.Insert(shape, shape.HashId())
	} else {
		s.dynamicShapes.class.Insert(shape, shape.HashId())
	}
	shape.Space = s

	return shape
}

// RemoveShape removes a collision shape from the simulation.
func (s *Space) RemoveShape(shape *Shape) {
	body := shape.Body

	isStatic := body.Type() == Static
	if isStatic {
		body.ActivateStatic(shape)
	} else {
		body.Activate()
	}

	body.RemoveShape(shape)
	s.FilterArbiters(body, shape)
	if isStatic {
		s.staticShapes.class.Remove(shape, shape.hashid)
	} else {
		s.dynamicShapes.class.Remove(shape, shape.hashid)
	}
	shape.Space = nil
	shape.hashid = 0
}

// AddBody adds body to the space.
//
// Do not add the same Body twice.
func (s *Space) AddBody(body *Body) {
	if body.Type() == Static {
		s.StaticBodies = append(s.StaticBodies, body)
	} else {
		s.DynamicBodies = append(s.DynamicBodies, body)
	}
	body.Space = s
}

// AddBodyWithShapes adds body to the space with body's shapes.
//
// Do not add the same Body twice.
func (s *Space) AddBodyWithShapes(body *Body) {
	s.AddBody(body)
	for _, shape := range body.Shapes {
		s.AddShape(shape)
	}
}

// ReindexShape re-computes the hash of the shape in both the dynamic and static list.
func (s *Space) ReindexShape(shape *Shape) {

	if s.IsLocked() {
		log.Fatalln(`You cannot manually reindex objects while the space is locked.
			 Wait until the current query or step is complete.`)
	}
	shape.CacheBB()

	// attempt to rehash the shape in both hashes
	s.dynamicShapes.class.ReindexObject(shape, shape.hashid)
	s.staticShapes.class.ReindexObject(shape, shape.hashid)
}

// RemoveBody removes a body from the simulation
func (s *Space) RemoveBody(body *Body) {
	body.Activate()

	if body.Type() == Static {
		s.StaticBodies = slices.DeleteFunc(s.StaticBodies, func(b *Body) bool {
			return b == body
		})
	} else {
		s.DynamicBodies = slices.DeleteFunc(s.DynamicBodies, func(b *Body) bool {
			return b == body
		})
	}

	body.Space = nil
}

// RemoveBodyWithShapes removes a body and body's shapes from the simulation
func (s *Space) RemoveBodyWithShapes(body *Body) {
	body.EachShape(func(shape *Shape) {
		s.RemoveShape(shape)
	})
	s.RemoveBody(body)
}

func (s *Space) AddConstraint(constraint *Constraint) *Constraint {

	a := constraint.bodyA
	b := constraint.bodyB
	a.Activate()
	b.Activate()
	s.constraints = append(s.constraints, constraint)

	// Push onto the heads of the bodies' constraint lists
	constraint.nextA = a.constraintList
	// possible nil pointer dereference (SA5011)
	a.constraintList = constraint
	constraint.nextB = b.constraintList
	b.constraintList = constraint
	constraint.space = s

	return constraint
}

func (s *Space) RemoveConstraint(constraint *Constraint) {

	constraint.bodyA.Activate()
	constraint.bodyB.Activate()
	s.constraints = slices.DeleteFunc(s.constraints, func(c *Constraint) bool {
		return c == constraint
	})

	constraint.bodyA.RemoveConstraint(constraint)
	constraint.bodyB.RemoveConstraint(constraint)
	constraint.space = nil
}

func (s *Space) FilterArbiters(body *Body, filter *Shape) {
	s.Lock()

	s.cachedArbiters.Filter(func(arb *Arbiter) bool {
		return CachedArbitersFilter(arb, s, filter, body)
	})

	s.Unlock(true)
}

func (s *Space) ContainsConstraint(constraint *Constraint) bool {
	return constraint.space == s
}

func (s *Space) ContainsShape(shape *Shape) bool {
	return shape.Space == s
}

func (s *Space) ContainsBody(body *Body) bool {
	return body.Space == s
}

func (s *Space) PushFreshContactBuffer() {
	stamp := s.stamp
	head := s.contactBuffersHead

	if head == nil {
		s.contactBuffersHead = NewContactBuffer(stamp, nil)
	} else if stamp-head.next.stamp > s.CollisionPersistence {
		tail := head.next
		s.contactBuffersHead = tail.InitHeader(stamp, tail)
	} else {
		// Allocate a new buffer and push it into the ring
		buffer := NewContactBuffer(stamp, head)
		head.next = buffer
		s.contactBuffersHead = buffer
	}
}

func (s *Space) ContactBufferGetArray() []Contact {
	if s.contactBuffersHead.numContacts+MaxContactsPerArbiter > ContactsBufferSize {
		s.PushFreshContactBuffer()
	}

	head := s.contactBuffersHead
	return head.contacts[head.numContacts : head.numContacts+MaxContactsPerArbiter]
}

func (s *Space) ProcessComponents(dt float64) {
	sleep := s.SleepTimeThreshold != infinity

	// calculate the kinetic energy of all the bodies
	if sleep {
		dv := s.IdleSpeedThreshold
		var dvsq float64
		if dv != 0 {
			dvsq = dv * dv
		} else {
			dvsq = s.Gravity.MagSq() * dt * dt
		}

		// update idling and reset component nodes
		for _, body := range s.DynamicBodies {
			if body.Type() != Dynamic {
				continue
			}

			// Need to deal with infinite mass objects
			var keThreshold float64
			if dvsq != 0 {
				keThreshold = body.mass * dvsq
			}
			if body.KineticEnergy() > keThreshold {
				body.sleepingIdleTime = 0
			} else {
				body.sleepingIdleTime += dt
			}
		}
	}

	// Awaken any sleeping bodies found and then push arbiters to the bodies' lists.
	for _, arb := range s.Arbiters {
		a := arb.bodyA
		b := arb.bodyB

		if sleep {
			if b.Type() == Kinematic || a.IsSleeping() {
				a.Activate()
			}
			if a.Type() == Kinematic || b.IsSleeping() {
				b.Activate()
			}
		}

		a.PushArbiter(arb)
		b.PushArbiter(arb)
	}

	if sleep {
		// Bodies should be held active if connected by a joint to a kinematic.
		for _, constraint := range s.constraints {
			if constraint.bodyB.Type() == Kinematic {
				constraint.bodyA.Activate()
			}
			if constraint.bodyA.Type() == Kinematic {
				constraint.bodyB.Activate()
			}
		}

		// Generate components and deactivate sleeping ones
		for i := 0; i < len(s.DynamicBodies); {
			body := s.DynamicBodies[i]

			if body.ComponentRoot() == nil {
				// Body not in a component yet. Perform a DFS to flood fill mark
				// the component in the contact graph using this body as the root.
				FloodFillComponent(body, body)

				// Check if the component should be put to sleep.
				if !ComponentActive(body, s.SleepTimeThreshold) {
					s.sleepingComponents = append(s.sleepingComponents, body)
					for item := body; item != nil; item = item.sleepingNext {
						s.Deactivate(item)
					}

					// Deactivate() removed the current body from the list.
					// Skip incrementing the index counter.
					continue
				}
			}

			i++

			// Only sleeping bodies retain their component node pointers.
			body.sleepingRoot = nil
			body.sleepingNext = nil
		}
	}
}

func (s *Space) Step(dt float64) {
	if dt == 0 {
		return
	}

	s.stamp++

	prevDT := s.currDT
	s.currDT = dt

	// reset and empty the arbiter lists
	for _, arb := range s.Arbiters {
		arb.state = ArbiterStateNormal

		// If both bodies are awake, unthread the arbiter from the contact graph.
		if !arb.bodyA.IsSleeping() && !arb.bodyB.IsSleeping() {
			arb.Unthread()
		}
	}
	s.Arbiters = s.Arbiters[:0]

	s.Lock()
	{
		// Integrate positions
		for _, body := range s.DynamicBodies {
			body.positionFunc(body, dt)
		}

		// Find colliding pairs.
		s.PushFreshContactBuffer()
		s.dynamicShapes.class.Each(ShapeUpdateFunc)
		s.dynamicShapes.class.ReindexQuery(SpaceCollideShapesFunc, s)
	}
	s.Unlock(false)

	// Rebuild the contact graph (and detect sleeping components if sleeping is enabled)
	s.ProcessComponents(dt)

	s.Lock()
	{
		// Clear out old cached arbiters and call separate callbacks
		s.cachedArbiters.Filter(func(arb *Arbiter) bool {
			return SpaceArbiterSetFilter(arb, s)
		})

		// Prestep the arbiters and constraints.
		slop := s.CollisionSlop
		biasCoef := 1 - math.Pow(s.CollisionBias, dt)
		for _, arbiter := range s.Arbiters {
			arbiter.PreStep(dt, slop, biasCoef)
		}

		for _, constraint := range s.constraints {
			if constraint.PreSolve != nil {
				constraint.PreSolve(constraint, s)
			}

			constraint.Class.PreStep(dt)
		}

		// Integrate velocities.
		damping := math.Pow(s.Damping, dt)
		gravity := s.Gravity
		for _, body := range s.DynamicBodies {
			body.velocityFunc(body, gravity, damping, dt)
		}

		// Apply cached impulses
		var dtCoef float64
		if prevDT != 0 {
			dtCoef = dt / prevDT
		}

		for _, arbiter := range s.Arbiters {
			arbiter.ApplyCachedImpulse(dtCoef)
		}

		for _, constraint := range s.constraints {
			constraint.Class.ApplyCachedImpulse(dtCoef)
		}

		// Run the impulse solver.
		var i uint
		for i = 0; i < s.Iterations; i++ {
			for _, arbiter := range s.Arbiters {
				arbiter.ApplyImpulse()
			}

			for _, constraint := range s.constraints {
				constraint.Class.ApplyImpulse(dt)
			}
		}

		// Run the constraint post-solve callbacks
		for _, constraint := range s.constraints {
			if constraint.PostSolve != nil {
				constraint.PostSolve(constraint, s)
			}
		}

		// run the post-solve callbacks
		for _, arb := range s.Arbiters {
			arb.handler.PostSolveFunc(arb, s, arb.handler)
		}
	}
	s.Unlock(true)
}

func (s *Space) Lock() {
	s.locked = true
}

// IsLocked returns true from inside a callback when objects cannot be added/removed.
func (s *Space) IsLocked() bool {
	return s.locked
}

func (s *Space) Unlock(runPostStep bool) {
	s.locked = false
	for i := range s.rousedBodies {
		s.Activate(s.rousedBodies[i])
		s.rousedBodies[i] = nil
	}
	s.rousedBodies = s.rousedBodies[:0]

	if runPostStep && !s.skipPostStep {
		s.skipPostStep = true

		for _, callback := range s.PostStepCallbacks {
			f := callback.callback

			// Mark the func as nil in case calling it calls SpaceRunPostStepCallbacks() again.
			// TODO: need more tests around this case I think.
			callback.callback = nil

			if f != nil {
				f(s, callback.key, callback.data)
			}
		}

		s.PostStepCallbacks = s.PostStepCallbacks[:0]
		s.skipPostStep = false
	}
}

func (s *Space) UncacheArbiter(arb *Arbiter) {
	a := arb.shapeA
	b := arb.shapeB
	shapePair := ShapePair{a, b}
	arbHashId := HashPair(HashValue(unsafe.Pointer(a)), HashValue(unsafe.Pointer(b)))
	s.cachedArbiters.Remove(arbHashId, shapePair)
	for i, a := range s.Arbiters {
		if a == arb {
			// leak-free delete from slice
			last := len(s.Arbiters) - 1
			s.Arbiters[i] = s.Arbiters[last]
			s.Arbiters[last] = nil
			s.Arbiters = s.Arbiters[:last]
			return
		}
	}
	panic("Arbiter not found")
}

func (s *Space) PushContacts(count int) {
	s.contactBuffersHead.numContacts += count
}

func (s *Space) PopContacts(count int) {
	s.contactBuffersHead.numContacts -= count
}

// LookupHandler finds and returns the matching a/b handler
func (s *Space) LookupHandler(a, b CollisionType, defaultHandler *CollisionHandler) *CollisionHandler {
	types := &CollisionHandler{TypeA: a, TypeB: b}
	handler := s.collisionHandlers.Find(HashPair(HashValue(a), HashValue(b)), types)
	if handler != nil {
		return handler
	}
	return defaultHandler
}

// AddCollisionHandler adds and returns the CollisionHandler for collisions between objects of type a and b.
//
// Fill the desired collision callback functions, for details see the CollisionHandler object.
//
// Whenever shapes with collision types (Shape.CollisionType) a and b collide,
// this handler will be used to process the collision events. When a new collision
// handler is created, the callbacks will all be set to builtin callbacks that perform
// the default behavior (call the wildcard handlers, and accept all collisions).
func (s *Space) AddCollisionHandler(a, b CollisionType) *CollisionHandler {
	hash := HashPair(HashValue(a), HashValue(b))
	handler := &CollisionHandler{
		a,
		b,
		DefaultBegin,
		DefaultPreSolve,
		DefaultPostSolve,
		DefaultSeparate,
		nil,
	}
	return s.collisionHandlers.Insert(
		hash,
		handler,
		func(a *CollisionHandler) *CollisionHandler { return a },
	)
}

// AddCollisionHandler adds handler to space, for details see the CollisionHandler{} struct.
func (s *Space) AddCollisionHandler2(handler *CollisionHandler) {
	hash := HashPair(HashValue(handler.TypeA), HashValue(handler.TypeB))

	s.collisionHandlers.Insert(
		hash,
		handler,
		func(a *CollisionHandler) *CollisionHandler { return a },
	)
}

// AddWildcardCollisionHandler sets a collision handler for given collision type.
// This handler will be used any time an object with this type collides with
// another object, regardless of its type. A good example is a projectile that
// should be destroyed the first time it hits anything. There may be a specific
// collision handler and two wildcard handlers. It’s up to the specific handler
// to decide if and when to call the wildcard handlers and what to do with their
// return values. When a new wildcard handler is created, the callbacks will all
// be set to builtin callbacks that perform the default behavior. (accept all
// collisions in Begin() and PreSolve(), or do nothing for PostSolve() and Separate().
func (s *Space) AddWildcardCollisionHandler(typeA CollisionType) *CollisionHandler {
	s.UseWildcardDefaultHandler()

	hash := HashPair(HashValue(typeA), HashValue(WildcardCollisionType))
	handler := &CollisionHandler{
		typeA,
		WildcardCollisionType,
		AlwaysCollide,
		AlwaysCollide,
		DoNothing,
		DoNothing,
		nil,
	}
	return s.collisionHandlers.Insert(
		hash,
		handler,
		func(a *CollisionHandler) *CollisionHandler { return a },
	)
}

func (s *Space) UseWildcardDefaultHandler() {
	if !s.usesWildcards {
		s.usesWildcards = true
		s.defaultHandler = &CollisionHandlerDefault
	}
}

func (s *Space) UseSpatialHash(dim float64, count int) {
	staticShapes := NewSpaceHash(dim, count, ShapeGetBB, nil)
	dynamicShapes := NewSpaceHash(dim, count, ShapeGetBB, staticShapes)

	s.staticShapes.class.Each(func(shape *Shape) {
		staticShapes.class.Insert(shape, shape.hashid)
	})
	s.dynamicShapes.class.Each(func(shape *Shape) {
		dynamicShapes.class.Insert(shape, shape.hashid)
	})

	s.staticShapes = staticShapes
	s.dynamicShapes = dynamicShapes
}

// EachBody calls func f for each body in the space
//
// Example:
//
//	s.EachBody(func(body *cm.Body) {
//		fmt.Println(body.Position())
//	})
func (s *Space) EachBody(f func(b *Body)) {
	s.Lock()
	defer s.Unlock(true)

	for _, b := range s.DynamicBodies {
		f(b)
	}

	for _, b := range s.StaticBodies {
		f(b)
	}

	for _, root := range s.sleepingComponents {
		b := root

		for b != nil {
			next := b.sleepingNext
			f(b)
			b = next
		}
	}
}

// EachStaticBody calls func f for each static body in the space
func (s *Space) EachStaticBody(f func(b *Body)) {
	s.Lock()
	defer s.Unlock(true)

	for _, b := range s.StaticBodies {
		f(b)
	}

}

// EachDynamicBody calls func f for each dynamic body in the space
func (s *Space) EachDynamicBody(f func(b *Body)) {
	s.Lock()
	defer s.Unlock(true)

	for _, b := range s.DynamicBodies {
		f(b)
	}

	for _, root := range s.sleepingComponents {
		b := root

		for b != nil {
			next := b.sleepingNext
			f(b)
			b = next
		}
	}
}

// EachStaticShape calls func f for each static shape in the space
func (s *Space) EachStaticShape(f func(*Shape)) {
	s.Lock()
	s.staticShapes.class.Each(func(shape *Shape) {
		f(shape)
	})
	s.Unlock(true)
}

// EachDynamicShape calls func f for each dynamic shape in the space
func (s *Space) EachDynamicShape(f func(*Shape)) {
	s.Lock()
	s.dynamicShapes.class.Each(func(shape *Shape) {
		f(shape)
	})
	s.Unlock(true)
}

func (s *Space) DynamicShapeCount() int {
	return s.dynamicShapes.class.Count()
}
func (s *Space) StaticShapeCount() int {
	return s.staticShapes.class.Count()
}
func (s *Space) ShapeCount() int {
	return s.staticShapes.class.Count() + s.dynamicShapes.class.Count()
}

// EachShape calls func f for each shape in the space
func (s *Space) EachShape(f func(*Shape)) {
	s.Lock()

	s.dynamicShapes.class.Each(func(shape *Shape) {
		f(shape)
	})
	s.staticShapes.class.Each(func(shape *Shape) {
		f(shape)
	})

	s.Unlock(true)
}

func (s *Space) EachConstraint(f func(*Constraint)) {
	s.Lock()

	for i := range s.constraints {
		f(s.constraints[i])
	}

	s.Unlock(true)
}

// Query the space at a point and return the nearest shape found. Returns nil if no shapes were found.
func (s *Space) PointQueryNearest(point v.Vec, maxDistance float64, filter ShapeFilter) *PointQueryInfo {
	info := &PointQueryInfo{nil, v.Vec{}, maxDistance, v.Vec{}}
	context := &PointQueryContext{point, maxDistance, filter, nil}

	bb := NewBBForCircle(point, math.Max(maxDistance, 0))
	s.dynamicShapes.class.Query(context, bb, NearestPointQueryNearest, info)
	s.staticShapes.class.Query(context, bb, NearestPointQueryNearest, info)

	return info
}

func (s *Space) bbQuery(obj any, shape *Shape, collisionId uint32, data any) uint32 {
	context := obj.(*BBQueryContext)
	if !shape.Filter.Reject(context.filter) && shape.BB.Intersects(context.bb) {
		context.f(shape, data)
	}
	return collisionId
}
func (s *Space) BBQuery(bb BB, filter ShapeFilter, f SpaceBBQueryFunc, data any) {
	context := BBQueryContext{bb, filter, f}
	s.staticShapes.class.Query(&context, bb, s.bbQuery, data)

	s.Lock()
	s.dynamicShapes.class.Query(&context, bb, s.bbQuery, data)
	s.Unlock(true)
}

// SliceForBodyType returns bodies of the given type in the space.
func (s *Space) SliceForBodyType(t BodyType) *[]*Body {
	if t == Static {
		return &s.StaticBodies
	}
	return &s.DynamicBodies
}

func (s *Space) SegmentQuery(start, end v.Vec, radius float64, filter ShapeFilter, f SpaceSegmentQueryFunc, data any) {
	context := SegmentQueryContext{start, end, radius, filter, f}
	s.Lock()

	s.staticShapes.class.SegmentQuery(&context, start, end, 1, segmentQuery, data)
	s.dynamicShapes.class.SegmentQuery(&context, start, end, 1, segmentQuery, data)

	s.Unlock(true)
}

func (s *Space) SegmentQueryFirst(start, end v.Vec, radius float64, filter ShapeFilter) SegmentQueryInfo {
	info := SegmentQueryInfo{nil, end, v.Vec{}, 1}
	context := &SegmentQueryContext{start, end, radius, filter, nil}
	s.staticShapes.class.SegmentQuery(context, start, end, 1, queryFirst, &info)
	s.dynamicShapes.class.SegmentQuery(context, start, end, info.Alpha, queryFirst, &info)
	return info
}

func (s *Space) TimeStep() float64 {
	return s.currDT
}

func (s *Space) PostStepCallback(key any) *PostStepCallback {
	for i := range s.PostStepCallbacks {
		callback := s.PostStepCallbacks[i]
		if callback != nil && callback.key == key {
			return callback
		}
	}
	return nil
}

// AddPostStepCallback defines a callback to be run just before s.Step() finishes.
//
// The main reason you want to define post-step callbacks is to get around
// the restriction that you cannot call the add/remove methods from a collision handler callback.
// Post-step callbacks run right before the next (or current) call to s.Step() returns when it is safe to add and remove objects.
// You can only schedule one post-step callback per key value, this prevents you from accidentally removing an object twice.
// Registering a second callback for the same key is a no-op.
//
// example:
// type PostStepCallbackFunc func(space *Space, key any, data any)
func (s *Space) AddPostStepCallback(f PostStepCallbackFunc, key, data any) bool {
	if key == nil || s.PostStepCallback(key) == nil {
		callback := &PostStepCallback{
			key:  key,
			data: data,
		}
		if f != nil {
			callback.callback = f
		} else {
			callback.callback = PostStepDoNothing
		}
		s.PostStepCallbacks = append(s.PostStepCallbacks, callback)
		return true
	}
	return false
}

// ShapeQuery queries a space for any shapes overlapping the this shape and call the callback for each shape found.
func (s *Space) ShapeQuery(shape *Shape, callback func(shape *Shape, points *ContactPointSet)) bool {
	body := shape.Body
	var bb BB
	if body != nil {
		bb = shape.Update(body.transform)
	} else {
		bb = shape.BB
	}

	var anyCollision bool

	shapeQuery := func(obj any, b *Shape, collisionId uint32, _ any) uint32 {
		a := obj.(*Shape)
		if a.Filter.Reject(b.Filter) || a == b {
			return collisionId
		}

		contactPointSet := ShapesCollideInfo(a, b)
		if contactPointSet.Count > 0 {
			if callback != nil {
				callback(b, &contactPointSet)
			}
			anyCollision = !(a.Sensor || b.Sensor)
		}

		return collisionId
	}

	s.Lock()
	{
		s.dynamicShapes.class.Query(shape, bb, shapeQuery, nil)
		s.staticShapes.class.Query(shape, bb, shapeQuery, nil)
	}
	s.Unlock(true)

	return anyCollision
}

func PostStepDoNothing(space *Space, key, data any) {}

func SpaceCollideShapesFunc(obj any, b *Shape, collisionId uint32, vspace any) uint32 {
	a := obj.(*Shape)
	space := vspace.(*Space)

	// Reject any of the simple cases
	if QueryReject(a, b) {
		return collisionId
	}

	// Narrow-phase collision detection.
	info := Collide(a, b, collisionId, space.ContactBufferGetArray())

	if info.count == 0 {
		// shapes are not colliding
		return info.collisionId
	}

	//  Push contacts
	space.PushContacts(info.count)

	// Get an arbiter from space->arbiterSet for the two shapes.
	// This is where the persistent contact magic comes from.
	shapePair := ShapePair{info.a, info.b}
	arbHashId := HashPair(HashValue(unsafe.Pointer(info.a)), HashValue(unsafe.Pointer(info.b)))
	arb := space.cachedArbiters.Insert(arbHashId, shapePair, func(shapes ShapePair) *Arbiter {
		arb := space.pooledArbiters.Get().(*Arbiter)
		arb.Init(shapes.a, shapes.b)
		return arb
	})
	arb.Update(&info, space)

	if arb.state == ArbiterStateFirstCollision && !arb.handler.BeginFunc(arb, space, arb.handler.UserData) {
		arb.Ignore()
	}

	// Ignore the arbiter if it has been flagged
	if arb.state != ArbiterStateIgnore &&
		// Call PreSolve
		arb.handler.PreSolveFunc(arb, space, arb.handler.UserData) &&
		// Check (again) in case the pre-solve() callback called ArbiterIgnored().
		arb.state != ArbiterStateIgnore &&
		// Process, but don't add collisions for sensors.
		!(a.Sensor || b.Sensor) &&
		// Don't process collisions between two infinite mass bodies.
		// This includes collisions between two kinematic bodies, or a kinematic body and a static body.
		!(a.Body.mass == infinity && b.Body.mass == infinity) {
		space.Arbiters = append(space.Arbiters, arb)
	} else {
		space.PopContacts(info.count)
		arb.Contacts = nil
		arb.count = 0

		// Normally arbiters are set as used after calling the post-solve callback.
		// However, post-solve() callbacks are not called for sensors or arbiters rejected from pre-solve.
		if arb.state != ArbiterStateIgnore {
			arb.state = ArbiterStateNormal
		}
	}

	// Time stamp the arbiter so we know it was used recently.
	arb.stamp = space.stamp
	return info.collisionId
}

// QueryReject returns true if shapes a and b reject to collide.
func QueryReject(a, b *Shape) bool {
	if a.Body == b.Body {
		return true
	}
	if a.Filter.Reject(b.Filter) {
		return true
	}
	if !a.BB.Intersects(b.BB) {
		return true
	}
	if QueryRejectConstraints(a.Body, b.Body) {
		return true
	}
	return false
}

func QueryRejectConstraints(a, b *Body) bool {
	for constraint := a.constraintList; constraint != nil; constraint = constraint.Next(a) {
		if !constraint.collideBodies && ((constraint.bodyA == a && constraint.bodyB == b) ||
			(constraint.bodyA == b && constraint.bodyB == a)) {
			return true
		}
	}

	return false
}

func ComponentActive(root *Body, threshold float64) bool {
	for item := root; item != nil; item = item.sleepingNext {
		if item.sleepingIdleTime < threshold {
			return true
		}
	}
	return false
}

func FloodFillComponent(root *Body, body *Body) {
	// Kinematic bodies cannot be put to sleep and prevent bodies they are touching from sleeping.
	// Static bodies are effectively sleeping all the time.
	if body.Type() != Dynamic {
		return
	}

	// body.sleeping.root
	otherRoot := body.ComponentRoot()
	if otherRoot == nil {
		root.ComponentAdd(body)

		for arb := body.arbiterList; arb != nil; arb = ArbiterNext(arb, body) {
			if body == arb.bodyA {
				FloodFillComponent(root, arb.bodyB)
			} else {
				FloodFillComponent(root, arb.bodyA)
			}
		}

		for constraint := body.constraintList; constraint != nil; constraint = constraint.Next(body) {
			if body == constraint.bodyA {
				FloodFillComponent(root, constraint.bodyB)
			} else {
				FloodFillComponent(root, constraint.bodyA)
			}
		}
	} else {
		if otherRoot != root {
			log.Fatalln("Inconsistency detected in the contact graph (FFC)")
		}
	}
}

func ArbiterNext(arb *Arbiter, body *Body) *Arbiter {
	if arb.bodyA == body {
		return arb.threadA.next
	}
	return arb.threadB.next
}

func Contains(bodies []*Body, body *Body) bool {
	return slices.Contains(bodies, body)
}

func NearestPointQueryNearest(obj any, shape *Shape, collisionId uint32, out any) uint32 {
	context := obj.(*PointQueryContext)
	if !shape.Filter.Reject(context.filter) && !shape.Sensor {
		info := shape.PointQuery(context.point)
		if info.Distance < out.(*PointQueryInfo).Distance {
			outp := out.(*PointQueryInfo)
			*outp = info
		}
	}

	return collisionId
}

func segmentQuery(obj any, shape *Shape, data any) float64 {
	context := obj.(*SegmentQueryContext)
	var info SegmentQueryInfo

	if !shape.Filter.Reject(context.filter) && shape.SegmentQuery(context.start, context.end, context.radius, &info) {
		context.f(shape, info.Point, info.Normal, info.Alpha, data)
	}

	return 1
}

func queryFirst(obj any, shape *Shape, data any) float64 {
	context := obj.(*SegmentQueryContext)
	out := data.(*SegmentQueryInfo)
	var info SegmentQueryInfo

	if !shape.Filter.Reject(context.filter) &&
		!shape.Sensor &&
		shape.SegmentQuery(context.start, context.end, context.radius, &info) &&
		info.Alpha < out.Alpha {
		*out = info
	}

	return out.Alpha
}

func arbiterSetEql(shapes ShapePair, arb *Arbiter) bool {
	a := shapes.a
	b := shapes.b

	return (a == arb.shapeA && b == arb.shapeB) || (b == arb.shapeA && a == arb.shapeB)
}

var ShapeVelocityFunc = func(obj any) v.Vec {
	return obj.(*Shape).Body.velocity
}

var ShapeUpdateFunc = func(shape *Shape) {
	shape.CacheBB()
}

type PostStepCallback struct {
	callback PostStepCallbackFunc
	key      any
	data     any
}

type PointQueryContext struct {
	point       v.Vec
	maxDistance float64
	filter      ShapeFilter
	f           SpacePointQueryFunc
}

type ShapePair struct {
	a, b *Shape
}

type BBQueryContext struct {
	bb     BB
	filter ShapeFilter
	f      SpaceBBQueryFunc
}

type SegmentQueryContext struct {
	start, end v.Vec
	radius     float64
	filter     ShapeFilter
	f          SpaceSegmentQueryFunc
}

type SpacePointQueryFunc func(*Shape, v.Vec, float64, v.Vec, any)
type SpaceBBQueryFunc func(shape *Shape, data any)
type SpaceSegmentQueryFunc func(shape *Shape, point, normal v.Vec, alpha float64, data any)
type PostStepCallbackFunc func(space *Space, key any, data any)
