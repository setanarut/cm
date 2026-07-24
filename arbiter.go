package cm

import (
	"log"

	"github.com/setanarut/v"
)

// Arbiter states
const (
	// Arbiter is active and its the first collision.
	arbiterStateFirstCollision = iota
	// Arbiter is active and its not the first collision.
	arbiterStateNormal
	// Collision has been explicitly ignored. Either by returning false from a
	// begin collision handler or calling ArbiterIgnore().
	arbiterStateIgnore
	// Collison is no longer active. A space will cache an arbiter for up to Space.
	// CollisionPersistence more steps.
	arbiterStateCached
	// Collison arbiter is invalid because one of the shapes was removed.
	arbiterStateInvalidated
)

const Wildcard CollisionType = ^CollisionType(0)

// Arbiter struct tracks pairs of colliding shapes.
//
// They are also used in conjuction with collision handler callbacks allowing you to retrieve information on the collision or change it.
// A unique arbiter value is used for each pair of colliding objects. It persists until the shapes separate.
type Arbiter struct {
	UserData any

	shapeA, shapeB              *Shape
	bodyA, bodyB                *Body
	threadA, threadB            arbiterThread
	e, u                        float64
	count                       int
	state                       int       // Arbiter state enum
	Contacts                    []Contact // a slice onto the current buffer array of contacts
	surfaceVr                   v.Vec
	normal                      v.Vec
	handler, handlerA, handlerB *CollisionHandler // Regular, wildcard A and wildcard B collision handlers.
	swapped                     bool
	stamp                       uint
}

// Init initializes and returns Arbiter
func (a *Arbiter) Init(shapeA, shapeB *Shape) *Arbiter {
	a.shapeA = shapeA
	a.bodyA = shapeA.Body
	a.shapeB = shapeB
	a.bodyB = shapeB.Body
	a.state = arbiterStateFirstCollision
	return a
}

type arbiterThread struct {
	next, prev *Arbiter
}

func (a *Arbiter) next(body *Body) *Arbiter {
	if a.bodyA == body {
		return a.threadA.next
	} else {
		return a.threadB.next
	}
}

func (a *Arbiter) unthread() {
	a.unthreadHelper(a.bodyA)
	a.unthreadHelper(a.bodyB)
}

func (a *Arbiter) unthreadHelper(body *Body) {
	thread := a.threadForBody(body)
	prev := thread.prev
	next := thread.next

	if prev != nil {
		prev.threadForBody(body).next = next
	} else if body.arbiterList == a {
		// IFF prev is nil and body->arbiterList == arb, is arb at the head of the list.
		// This function may be called for an arbiter that was never in a list.
		// In that case, we need to protect it from wiping out the body->arbiterList pointer.
		body.arbiterList = next
	}

	if next != nil {
		next.threadForBody(body).prev = prev
	}

	thread.next = nil
	thread.prev = nil
}

func (a *Arbiter) threadForBody(body *Body) *arbiterThread {
	if a.bodyA == body {
		return &a.threadA
	} else {
		return &a.threadB
	}
}

func (a *Arbiter) applyCachedImpulse(dtCoef float64) {
	if a.IsFirstContact() {
		return
	}

	for i := range a.count {
		contact := a.Contacts[i]
		j := rotateComplex(a.normal, v.Vec{contact.jnAcc, contact.jtAcc})
		applyImpulses(a.bodyA, a.bodyB, contact.R1, contact.R2, j.Scale(dtCoef))
	}
}

func (a *Arbiter) applyImpulse() {
	bA := a.bodyA
	bB := a.bodyB
	nrm := a.normal
	surfaceVR := a.surfaceVr
	friction := a.u

	for i := range a.count {
		con := &a.Contacts[i]
		nMass := con.nMass
		r1 := con.R1
		r2 := con.R2

		vb1 := bA.vBias.Add(perp(r1).Scale(bA.wBias))
		vb2 := bB.vBias.Add(perp(r2).Scale(bB.wBias))
		vr := relativeVelocity(bA, bB, r1, r2).Add(surfaceVR)

		vbn := vb2.Sub(vb1).Dot(nrm)
		vrn := vr.Dot(nrm)
		vrt := vr.Dot(perp(nrm))

		jbn := (con.bias - vbn) * nMass
		jbnOld := con.jBias
		con.jBias = max(jbnOld+jbn, 0)

		jn := -(con.bounce + vrn) * nMass
		jnOld := con.jnAcc
		con.jnAcc = max(jnOld+jn, 0)

		jtMax := friction * con.jnAcc
		jt := -vrt * con.tMass
		jtOld := con.jtAcc
		con.jtAcc = clamp(jtOld+jt, -jtMax, jtMax)

		applyBiasImpulses(bA, bB, r1, r2, nrm.Scale(con.jBias-jbnOld))
		applyImpulses(bA, bB, r1, r2, rotateComplex(nrm, v.Vec{
			X: con.jnAcc - jnOld,
			Y: con.jtAcc - jtOld,
		}))
	}
}

func (a *Arbiter) IsFirstContact() bool {
	return a.state == arbiterStateFirstCollision
}

func (arb *Arbiter) preStep(dt, slop, bias float64) {
	a := arb.bodyA
	b := arb.bodyB
	n := arb.normal
	bodyDelta := b.position.Sub(a.position)

	for i := range arb.count {
		con := &arb.Contacts[i]

		// Calculate the mass normal and mass tangent.
		con.nMass = 1.0 / kScalar(a, b, con.R1, con.R2, n)
		con.tMass = 1.0 / kScalar(a, b, con.R1, con.R2, perp(n))

		// Calculate the target bias velocity.
		dist := con.R2.Sub(con.R1).Add(bodyDelta).Dot(n)
		con.bias = -bias * min(0, dist+slop) / dt
		con.jBias = 0.0

		// Calculate the target bounce velocity.
		con.bounce = normalRelativeVelocity(a, b, con.R1, con.R2, n) * arb.e
	}
}

func (arb *Arbiter) update(info *CollisionInfo, space *Space) {
	a := info.a
	b := info.b

	// For collisions between two similar primitive types, the order could have
	// been swapped since the last frame.
	arb.shapeA = a
	arb.bodyA = a.Body
	arb.shapeB = b
	arb.bodyB = b.Body

	// Iterate over the possible pairs to look for hash value matches.
	for i := range info.count {
		con := &info.arr[i]

		// r1 and r2 store absolute offsets at init time.
		// Need to convert them to relative offsets.
		con.R1 = con.R1.Sub(a.Body.position)
		con.R2 = con.R2.Sub(b.Body.position)

		// Cached impulses are not zeroed at init time.
		con.jnAcc = 0
		con.jtAcc = 0

		for j := range arb.count {
			old := arb.Contacts[j]

			// This could trigger false positives, but is fairly unlikely nor serious if it does.
			if con.hash == old.hash {
				// Copy the persistent contact information.
				con.jnAcc = old.jnAcc
				con.jtAcc = old.jtAcc
			}
		}
	}

	arb.Contacts = info.arr[:info.count]
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
	swapped := typeA != handler.TypeA && handler.TypeA != Wildcard
	arb.swapped = swapped

	if handler != space.defaultHandler || space.usesWildcards {
		// The order of the main handler swaps the wildcard handlers too. Uffda.
		if swapped {
			arb.handlerA = space.LookupHandler(typeB, Wildcard, &CollisionHandlerDoNothing)
			arb.handlerB = space.LookupHandler(typeA, Wildcard, &CollisionHandlerDoNothing)
		} else {
			arb.handlerA = space.LookupHandler(typeA, Wildcard, &CollisionHandlerDoNothing)
			arb.handlerB = space.LookupHandler(typeB, Wildcard, &CollisionHandlerDoNothing)
		}
	}

	// mark it as new if it's been cached
	if arb.state == arbiterStateCached {
		arb.state = arbiterStateFirstCollision
	}
}

// Ignore marks a collision pair to be ignored until the two objects separate.
//
// Pre-solve and post-solve callbacks will not be called, but the separate callback will be called.
func (a *Arbiter) Ignore() bool {
	a.state = arbiterStateIgnore
	return false
}

// CallWildcardBeginA if you want a custom callback to invoke the wildcard callback for the first collision type, you must call this function explicitly.
//
// You must decide how to handle the wildcard's return value since it may disagree with the other wildcard handler's return value or your own.
func (a *Arbiter) CallWildcardBeginA(space *Space) bool {
	handler := a.handlerA
	return handler.BeginFunc(a, space, handler.UserData)
}

// CallWildcardBeginB If you want a custom callback to invoke the wildcard callback for the second collision type, you must call this function explicitly.
func (a *Arbiter) CallWildcardBeginB(space *Space) bool {
	handler := a.handlerB
	a.swapped = !a.swapped
	retVal := handler.BeginFunc(a, space, handler.UserData)
	a.swapped = !a.swapped
	return retVal
}

// CallWildcardPreSolveA If you want a custom callback to invoke the wildcard callback for the first collision type, you must call this function explicitly.
func (a *Arbiter) CallWildcardPreSolveA(space *Space) bool {
	handler := a.handlerA
	return handler.PreSolveFunc(a, space, handler.UserData)
}

// CallWildcardPreSolveB If you want a custom callback to invoke the wildcard callback for the second collision type, you must call this function explicitly.
func (a *Arbiter) CallWildcardPreSolveB(space *Space) bool {
	handler := a.handlerB
	a.swapped = !a.swapped
	retval := handler.PreSolveFunc(a, space, handler.UserData)
	a.swapped = !a.swapped
	return retval
}

func (a *Arbiter) CallWildcardPostSolveA(space *Space) {
	handler := a.handlerA
	handler.PostSolveFunc(a, space, handler.UserData)
}

func (a *Arbiter) CallWildcardPostSolveB(space *Space) {
	handler := a.handlerB
	a.swapped = !a.swapped
	handler.PostSolveFunc(a, space, handler.UserData)
	a.swapped = !a.swapped
}

func (a *Arbiter) CallWildcardSeparateA(space *Space) {
	handler := a.handlerA
	handler.SeparateFunc(a, space, handler.UserData)
}

func (a *Arbiter) CallWildcardSeparateB(space *Space) {
	handler := a.handlerB
	a.swapped = !a.swapped
	handler.SeparateFunc(a, space, handler.UserData)
	a.swapped = !a.swapped
}

func applyImpulses(a, b *Body, r1, r2, j v.Vec) {
	b.velocity.X += j.X * b.massInverse
	b.velocity.Y += j.Y * b.massInverse
	b.w += b.momentOfInertiaInverse * (r2.X*j.Y - r2.Y*j.X)

	j.X = -j.X
	j.Y = -j.Y
	a.velocity.X += j.X * a.massInverse
	a.velocity.Y += j.Y * a.massInverse
	a.w += a.momentOfInertiaInverse * (r1.X*j.Y - r1.Y*j.X)
}

func applyImpulse(body *Body, j, r v.Vec) {
	body.velocity.X += j.X * body.massInverse
	body.velocity.Y += j.Y * body.massInverse
	body.w += body.momentOfInertiaInverse * r.Cross(j)
}

func applyBiasImpulses(a, b *Body, r1, r2, j v.Vec) {
	b.vBias.X += j.X * b.massInverse
	b.vBias.Y += j.Y * b.massInverse
	b.wBias += b.momentOfInertiaInverse * (r2.X*j.Y - r2.Y*j.X)

	j.X = -j.X
	j.Y = -j.Y
	a.vBias.X += j.X * a.massInverse
	a.vBias.Y += j.Y * a.massInverse
	a.wBias += a.momentOfInertiaInverse * (r1.X*j.Y - r1.Y*j.X)
}

func relativeVelocity(a, b *Body, r1, r2 v.Vec) v.Vec {
	return perp(r2).Scale(b.w).Add(b.velocity).Sub(perp(r1).Scale(a.w).Add(a.velocity))
}

func AlwaysCollideFunc(_ *Arbiter, _ *Space, _ any) bool {
	return true
}

func DoNothingFunc(_ *Arbiter, _ *Space, _ any) {

}

func DefaultBeginFunc(arb *Arbiter, space *Space, _ any) bool {
	return arb.CallWildcardBeginA(space) && arb.CallWildcardBeginB(space)
}

func DefaultPreSolveFunc(arb *Arbiter, space *Space, _ any) bool {
	return arb.CallWildcardPreSolveA(space) && arb.CallWildcardPreSolveB(space)
}

func DefaultPostSolveFunc(arb *Arbiter, space *Space, _ any) {
	arb.CallWildcardPostSolveA(space)
	arb.CallWildcardPostSolveB(space)
}

func DefaultSeparateFunc(arb *Arbiter, space *Space, _ any) {
	arb.CallWildcardSeparateA(space)
	arb.CallWildcardSeparateB(space)
}

// TotalImpulse calculates the total impulse including the friction that was applied by this arbiter.
//
// This function should only be called from a post-solve, post-step or EachArbiter callback.
func (a *Arbiter) TotalImpulse() v.Vec {
	var sum v.Vec

	count := a.Count()
	for i := range count {
		con := a.Contacts[i]
		sum = sum.Add(rotateComplex(a.normal, v.Vec{con.jnAcc, con.jtAcc}))
	}

	if a.swapped {
		return sum
	}
	return sum.Neg()
}

func (a *Arbiter) Count() int {
	if a.state < arbiterStateCached {
		return int(a.count)
	}
	return 0
}

// Shapes return the colliding shapes involved for this arbiter.
// The order of their space.CollisionType values will match the order set when the collision handler was registered.
func (a *Arbiter) Shapes() (*Shape, *Shape) {
	if a.swapped {
		return a.shapeB, a.shapeA
	} else {
		return a.shapeA, a.shapeB
	}
}

// Bodies returns the colliding bodies involved for this arbiter.
// The order of the space.CollisionType the bodies are associated with values will match the order set when the collision handler was registered.
func (a *Arbiter) Bodies() (*Body, *Body) {
	shapeA, shapeB := a.Shapes()
	return shapeA.Body, shapeB.Body
}

func (a *Arbiter) Normal() v.Vec {
	if a.swapped {
		return a.normal.Scale(-1)
	} else {
		return a.normal
	}
}

// ContactPointSet wraps up the important collision data for an arbiter.
type ContactPointSet struct {
	// Count is the number of contact points in the set.
	Count int
	// Normal is the normal of the collision.
	Normal v.Vec

	Points [MaxContactsPerArbiter]struct {
		// The position of the contact on the surface of each shape.
		PointA, PointB v.Vec
		// Distance is penetration distance of the two shapes. Overlapping means it will be negative.
		//
		// This value is calculated as p2.Sub(p1).Dot(n) and is ignored by Arbiter.SetContactPointSet().
		Distance float64
	}
}

// ContactPointSet returns ContactPointSet
func (a *Arbiter) ContactPointSet() ContactPointSet {
	var set ContactPointSet
	set.Count = a.Count()

	swapped := a.swapped
	n := a.normal
	if swapped {
		set.Normal = n.Neg()
	} else {
		set.Normal = n
	}

	for i := range set.Count {
		// Contact points are relative to body CoGs;
		p1 := a.bodyA.position.Add(a.Contacts[i].R1)
		p2 := a.bodyB.position.Add(a.Contacts[i].R2)

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
func (a *Arbiter) SetContactPointSet(set *ContactPointSet) {
	count := set.Count
	if count != int(a.count) {
		log.Fatalln("contact point set count is not equal")
	}
	swapped := a.swapped
	if swapped {
		a.normal = set.Normal.Neg()
	} else {
		a.normal = set.Normal
	}

	for i := range count {
		p1 := set.Points[i].PointA
		p2 := set.Points[i].PointB

		if swapped {
			a.Contacts[i].R1 = p2.Sub(a.bodyA.position)
			a.Contacts[i].R2 = p1.Sub(a.bodyB.position)
		} else {
			a.Contacts[i].R1 = p1.Sub(a.bodyA.position)
			a.Contacts[i].R2 = p2.Sub(a.bodyB.position)
		}
	}
}
