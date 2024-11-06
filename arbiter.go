package cm

import (
	"log"
	"math"

	"github.com/setanarut/vec"
)

const WildcardCollisionType CollisionType = ^CollisionType(0)

// Arbiter struct tracks pairs of colliding shapes.
//
// They are also used in conjuction with collision handler callbacks allowing you to retrieve information on the collision or change it.
// A unique arbiter value is used for each pair of colliding objects. It persists until the shapes separate.
type Arbiter struct {
	UserData any

	shapeA, shapeB              *Shape
	bodyA, bodyB                *Body
	threadA, threadB            ArbiterThread
	e, u                        float64
	count                       int
	state                       int       // Arbiter state enum
	contacts                    []Contact // a slice onto the current buffer array of contacts
	surfaceVr                   vec.Vec2
	normal                      vec.Vec2
	handler, handlerA, handlerB *CollisionHandler // Regular, wildcard A and wildcard B collision handlers.
	swapped                     bool
	stamp                       uint
}

// Init initializes and returns Arbiter
func (arbiter *Arbiter) Init(a, b *Shape) *Arbiter {
	arbiter.handler = nil
	arbiter.swapped = false
	arbiter.handlerA = nil
	arbiter.handlerB = nil
	arbiter.e = 0
	arbiter.u = 0
	arbiter.surfaceVr = vec.Vec2{}
	arbiter.count = 0
	arbiter.contacts = nil
	arbiter.shapeA = a
	arbiter.bodyA = a.Body
	arbiter.shapeB = b
	arbiter.bodyB = b.Body
	arbiter.threadA.next = nil
	arbiter.threadB.next = nil
	arbiter.threadA.prev = nil
	arbiter.threadB.prev = nil
	arbiter.stamp = 0
	arbiter.state = ArbiterStateFirstCollision
	arbiter.UserData = nil
	return arbiter
}

type ArbiterThread struct {
	next, prev *Arbiter
}

func (node *Arbiter) Next(body *Body) *Arbiter {
	if node.bodyA == body {
		return node.threadA.next
	} else {
		return node.threadB.next
	}
}

func (arbiter *Arbiter) Unthread() {
	arbiter.unthreadHelper(arbiter.bodyA)
	arbiter.unthreadHelper(arbiter.bodyB)
}

func (arbiter *Arbiter) unthreadHelper(body *Body) {
	thread := arbiter.ThreadForBody(body)
	prev := thread.prev
	next := thread.next

	if prev != nil {
		prev.ThreadForBody(body).next = next
	} else if body.arbiterList == arbiter {
		// IFF prev is nil and body->arbiterList == arb, is arb at the head of the list.
		// This function may be called for an arbiter that was never in a list.
		// In that case, we need to protect it from wiping out the body->arbiterList pointer.
		body.arbiterList = next
	}

	if next != nil {
		next.ThreadForBody(body).prev = prev
	}

	thread.next = nil
	thread.prev = nil
}

func (arbiter *Arbiter) ThreadForBody(body *Body) *ArbiterThread {
	if arbiter.bodyA == body {
		return &arbiter.threadA
	} else {
		return &arbiter.threadB
	}
}

func (arbiter *Arbiter) ApplyCachedImpulse(dtCoef float64) {
	if arbiter.IsFirstContact() {
		return
	}

	for i := 0; i < arbiter.count; i++ {
		contact := arbiter.contacts[i]
		j := arbiter.normal.RotateComplex(vec.Vec2{contact.jnAcc, contact.jtAcc})
		applyImpulses(arbiter.bodyA, arbiter.bodyB, contact.r1, contact.r2, j.Scale(dtCoef))
	}
}

func (arbiter *Arbiter) ApplyImpulse() {
	a := arbiter.bodyA
	b := arbiter.bodyB
	n := arbiter.normal
	surfaceVR := arbiter.surfaceVr
	friction := arbiter.u

	for i := 0; i < arbiter.count; i++ {
		con := &arbiter.contacts[i]
		nMass := con.nMass
		r1 := con.r1
		r2 := con.r2

		vb1 := a.vBias.Add(r1.Perp().Scale(a.wBias))
		vb2 := b.vBias.Add(r2.Perp().Scale(b.wBias))
		vr := relativeVelocity(a, b, r1, r2).Add(surfaceVR)

		vbn := vb2.Sub(vb1).Dot(n)
		vrn := vr.Dot(n)
		vrt := vr.Dot(n.Perp())

		jbn := (con.bias - vbn) * nMass
		jbnOld := con.jBias
		con.jBias = math.Max(jbnOld+jbn, 0)

		jn := -(con.bounce + vrn) * nMass
		jnOld := con.jnAcc
		con.jnAcc = math.Max(jnOld+jn, 0)

		jtMax := friction * con.jnAcc
		jt := -vrt * con.tMass
		jtOld := con.jtAcc
		con.jtAcc = clamp(jtOld+jt, -jtMax, jtMax)

		applyBiasImpulses(a, b, r1, r2, n.Scale(con.jBias-jbnOld))
		applyImpulses(a, b, r1, r2, n.RotateComplex(vec.Vec2{
			X: con.jnAcc - jnOld,
			Y: con.jtAcc - jtOld,
		}))
	}
}

func (arbiter *Arbiter) IsFirstContact() bool {
	return arbiter.state == ArbiterStateFirstCollision
}

func (arb *Arbiter) PreStep(dt, slop, bias float64) {
	a := arb.bodyA
	b := arb.bodyB
	n := arb.normal
	bodyDelta := b.position.Sub(a.position)

	for i := 0; i < arb.count; i++ {
		con := &arb.contacts[i]

		// Calculate the mass normal and mass tangent.
		con.nMass = 1.0 / kScalar(a, b, con.r1, con.r2, n)
		con.tMass = 1.0 / kScalar(a, b, con.r1, con.r2, n.Perp())

		// Calculate the target bias velocity.
		dist := con.r2.Sub(con.r1).Add(bodyDelta).Dot(n)
		con.bias = -bias * math.Min(0, dist+slop) / dt
		con.jBias = 0.0

		// Calculate the target bounce velocity.
		con.bounce = normalRelativeVelocity(a, b, con.r1, con.r2, n) * arb.e
	}
}

func (arb *Arbiter) Update(info *CollisionInfo, space *Space) {
	a := info.a
	b := info.b

	// For collisions between two similar primitive types, the order could have
	// been swapped since the last frame.
	arb.shapeA = a
	arb.bodyA = a.Body
	arb.shapeB = b
	arb.bodyB = b.Body

	// Iterate over the possible pairs to look for hash value matches.
	for i := 0; i < info.count; i++ {
		con := &info.arr[i]

		// r1 and r2 store absolute offsets at init time.
		// Need to convert them to relative offsets.
		con.r1 = con.r1.Sub(a.Body.position)
		con.r2 = con.r2.Sub(b.Body.position)

		// Cached impulses are not zeroed at init time.
		con.jnAcc = 0
		con.jtAcc = 0

		for j := 0; j < arb.count; j++ {
			old := arb.contacts[j]

			// This could trigger false positives, but is fairly unlikely nor serious if it does.
			if con.hash == old.hash {
				// Copy the persistent contact information.
				con.jnAcc = old.jnAcc
				con.jtAcc = old.jtAcc
			}
		}
	}

	arb.contacts = info.arr[:info.count]
	arb.count = info.count
	arb.normal = info.n

	arb.e = a.Elasticity * b.Elasticity
	arb.u = a.Friction * b.Friction

	surfaceVr := b.SurfaceVelocity.Sub(a.SurfaceVelocity)
	arb.surfaceVr = surfaceVr.Sub(info.n.Scale(surfaceVr.Dot(info.n)))

	typeA := info.a.CollisionType
	typeB := info.b.CollisionType
	handler := space.LookupHandler(typeA, typeB, space.defaultHandler)
	arb.handler = handler

	// Check if the types match, but don't swap for a default handler which use the wildcard for type A.
	swapped := typeA != handler.TypeA && handler.TypeA != WildcardCollisionType
	arb.swapped = swapped

	if handler != space.defaultHandler || space.usesWildcards {
		// The order of the main handler swaps the wildcard handlers too. Uffda.
		if swapped {
			arb.handlerA = space.LookupHandler(typeB, WildcardCollisionType, &CollisionHandlerDoNothing)
			arb.handlerB = space.LookupHandler(typeA, WildcardCollisionType, &CollisionHandlerDoNothing)
		} else {
			arb.handlerA = space.LookupHandler(typeA, WildcardCollisionType, &CollisionHandlerDoNothing)
			arb.handlerB = space.LookupHandler(typeB, WildcardCollisionType, &CollisionHandlerDoNothing)
		}
	}

	// mark it as new if it's been cached
	if arb.state == ArbiterStateCached {
		arb.state = ArbiterStateFirstCollision
	}
}

// Ignore marks a collision pair to be ignored until the two objects separate.
//
// Pre-solve and post-solve callbacks will not be called, but the separate callback will be called.
func (arb *Arbiter) Ignore() bool {
	arb.state = ArbiterStateIgnore
	return false
}

// CallWildcardBeginA if you want a custom callback to invoke the wildcard callback for the first collision type, you must call this function explicitly.
//
// You must decide how to handle the wildcard's return value since it may disagree with the other wildcard handler's return value or your own.
func (arb *Arbiter) CallWildcardBeginA(space *Space) bool {
	handler := arb.handlerA
	return handler.BeginFunc(arb, space, handler.UserData)
}

// CallWildcardBeginB If you want a custom callback to invoke the wildcard callback for the second collision type, you must call this function explicitly.
func (arb *Arbiter) CallWildcardBeginB(space *Space) bool {
	handler := arb.handlerB
	arb.swapped = !arb.swapped
	retVal := handler.BeginFunc(arb, space, handler.UserData)
	arb.swapped = !arb.swapped
	return retVal
}

// CallWildcardPreSolveA If you want a custom callback to invoke the wildcard callback for the first collision type, you must call this function explicitly.
func (arb *Arbiter) CallWildcardPreSolveA(space *Space) bool {
	handler := arb.handlerA
	return handler.PreSolveFunc(arb, space, handler.UserData)
}

// CallWildcardPreSolveB If you want a custom callback to invoke the wildcard callback for the second collision type, you must call this function explicitly.
func (arb *Arbiter) CallWildcardPreSolveB(space *Space) bool {
	handler := arb.handlerB
	arb.swapped = !arb.swapped
	retval := handler.PreSolveFunc(arb, space, handler.UserData)
	arb.swapped = !arb.swapped
	return retval
}

func (arb *Arbiter) CallWildcardPostSolveA(space *Space) {
	handler := arb.handlerA
	handler.PostSolveFunc(arb, space, handler.UserData)
}

func (arb *Arbiter) CallWildcardPostSolveB(space *Space) {
	handler := arb.handlerB
	arb.swapped = !arb.swapped
	handler.PostSolveFunc(arb, space, handler.UserData)
	arb.swapped = !arb.swapped
}

func (arb *Arbiter) CallWildcardSeparateA(space *Space) {
	handler := arb.handlerA
	handler.SeparateFunc(arb, space, handler.UserData)
}

func (arb *Arbiter) CallWildcardSeparateB(space *Space) {
	handler := arb.handlerB
	arb.swapped = !arb.swapped
	handler.SeparateFunc(arb, space, handler.UserData)
	arb.swapped = !arb.swapped
}

func applyImpulses(a, b *Body, r1, r2, j vec.Vec2) {
	b.velocity.X += j.X * b.massInverse
	b.velocity.Y += j.Y * b.massInverse
	b.w += b.momentOfInertiaInverse * (r2.X*j.Y - r2.Y*j.X)

	j.X = -j.X
	j.Y = -j.Y
	a.velocity.X += j.X * a.massInverse
	a.velocity.Y += j.Y * a.massInverse
	a.w += a.momentOfInertiaInverse * (r1.X*j.Y - r1.Y*j.X)
}

func applyImpulse(body *Body, j, r vec.Vec2) {
	body.velocity.X += j.X * body.massInverse
	body.velocity.Y += j.Y * body.massInverse
	body.w += body.momentOfInertiaInverse * r.Cross(j)
}

func applyBiasImpulses(a, b *Body, r1, r2, j vec.Vec2) {
	b.vBias.X += j.X * b.massInverse
	b.vBias.Y += j.Y * b.massInverse
	b.wBias += b.momentOfInertiaInverse * (r2.X*j.Y - r2.Y*j.X)

	j.X = -j.X
	j.Y = -j.Y
	a.vBias.X += j.X * a.massInverse
	a.vBias.Y += j.Y * a.massInverse
	a.wBias += a.momentOfInertiaInverse * (r1.X*j.Y - r1.Y*j.X)
}

func relativeVelocity(a, b *Body, r1, r2 vec.Vec2) vec.Vec2 {
	return r2.Perp().Scale(b.w).Add(b.velocity).Sub(r1.Perp().Scale(a.w).Add(a.velocity))
}

var CollisionHandlerDoNothing = CollisionHandler{
	WildcardCollisionType,
	WildcardCollisionType,
	AlwaysCollide,
	AlwaysCollide,
	DoNothing,
	DoNothing,
	nil,
}

var CollisionHandlerDefault = CollisionHandler{
	WildcardCollisionType,
	WildcardCollisionType,
	DefaultBegin,
	DefaultPreSolve,
	DefaultPostSolve,
	DefaultSeparate,
	nil,
}

func AlwaysCollide(_ *Arbiter, _ *Space, _ any) bool {
	return true
}

func DoNothing(_ *Arbiter, _ *Space, _ any) {

}

func DefaultBegin(arb *Arbiter, space *Space, _ any) bool {
	return arb.CallWildcardBeginA(space) && arb.CallWildcardBeginB(space)
}

func DefaultPreSolve(arb *Arbiter, space *Space, _ any) bool {
	return arb.CallWildcardPreSolveA(space) && arb.CallWildcardPreSolveB(space)
}

func DefaultPostSolve(arb *Arbiter, space *Space, _ any) {
	arb.CallWildcardPostSolveA(space)
	arb.CallWildcardPostSolveB(space)
}

func DefaultSeparate(arb *Arbiter, space *Space, _ any) {
	arb.CallWildcardSeparateA(space)
	arb.CallWildcardSeparateB(space)
}

// TotalImpulse calculates the total impulse including the friction that was applied by this arbiter.
//
// This function should only be called from a post-solve, post-step or EachArbiter callback.
func (arb *Arbiter) TotalImpulse() vec.Vec2 {
	var sum vec.Vec2

	count := arb.Count()
	for i := 0; i < count; i++ {
		con := arb.contacts[i]
		sum = sum.Add(arb.normal.RotateComplex(vec.Vec2{con.jnAcc, con.jtAcc}))
	}

	if arb.swapped {
		return sum
	}
	return sum.Neg()
}

func (arb *Arbiter) Count() int {
	if arb.state < ArbiterStateCached {
		return int(arb.count)
	}
	return 0
}

// Shapes return the colliding shapes involved for this arbiter.
// The order of their space.CollisionType values will match the order set when the collision handler was registered.
func (arb *Arbiter) Shapes() (*Shape, *Shape) {
	if arb.swapped {
		return arb.shapeB, arb.shapeA
	} else {
		return arb.shapeA, arb.shapeB
	}
}

// Bodies returns the colliding bodies involved for this arbiter.
// The order of the space.CollisionType the bodies are associated with values will match the order set when the collision handler was registered.
func (arb *Arbiter) Bodies() (*Body, *Body) {
	shapeA, shapeB := arb.Shapes()
	return shapeA.Body, shapeB.Body
}

func (arb *Arbiter) Normal() vec.Vec2 {
	if arb.swapped {
		return arb.normal.Scale(-1)
	} else {
		return arb.normal
	}
}

// ContactPointSet wraps up the important collision data for an arbiter.
type ContactPointSet struct {
	// Count is the number of contact points in the set.
	Count int
	// Normal is the normal of the collision.
	Normal vec.Vec2

	Points [MaxContactsPerArbiter]struct {
		// The position of the contact on the surface of each shape.
		PointA, PointB vec.Vec2
		// Distance is penetration distance of the two shapes. Overlapping means it will be negative.
		//
		// This value is calculated as p2.Sub(p1).Dot(n) and is ignored by Arbiter.SetContactPointSet().
		Distance float64
	}
}

// ContactPointSet returns ContactPointSet
func (arb *Arbiter) ContactPointSet() ContactPointSet {
	var set ContactPointSet
	set.Count = arb.Count()

	swapped := arb.swapped
	n := arb.normal
	if swapped {
		set.Normal = n.Neg()
	} else {
		set.Normal = n
	}

	for i := 0; i < set.Count; i++ {
		// Contact points are relative to body CoGs;
		p1 := arb.bodyA.position.Add(arb.contacts[i].r1)
		p2 := arb.bodyB.position.Add(arb.contacts[i].r2)

		if swapped {
			set.Points[i].PointA = p2
			set.Points[i].PointB = p1
		} else {
			set.Points[i].PointA = p1
			set.Points[i].PointB = p2
		}

		set.Points[i].Distance = p2.Sub(p1).Dot(n)
	}

	return set
}

// SetContactPointSet replaces the contact point set.
//
// This can be a very powerful feature, but use it with caution!
func (arb *Arbiter) SetContactPointSet(set *ContactPointSet) {
	count := set.Count
	if count != int(arb.count) {
		log.Fatalln("contact point set count is not equal")
	}
	swapped := arb.swapped
	if swapped {
		arb.normal = set.Normal.Neg()
	} else {
		arb.normal = set.Normal
	}

	for i := 0; i < count; i++ {
		p1 := set.Points[i].PointA
		p2 := set.Points[i].PointB

		if swapped {
			arb.contacts[i].r1 = p2.Sub(arb.bodyA.position)
			arb.contacts[i].r2 = p1.Sub(arb.bodyB.position)
		} else {
			arb.contacts[i].r1 = p1.Sub(arb.bodyA.position)
			arb.contacts[i].r2 = p2.Sub(arb.bodyB.position)
		}
	}
}
