package cm

// SpaceArbiterSetFilter throws away old arbiters.
func SpaceArbiterSetFilter(arb *Arbiter, space *Space) bool {
	// TODO: should make an arbiter state for this so it doesn't require filtering arbiters for dangling body pointers on body removal.
	// Preserve arbiters on sensors and rejected arbiters for sleeping objects.
	// This prevents errant separate callbacks from happening.

	a := arb.bodyA
	b := arb.bodyB

	if (a.Type() == Static || a.IsSleeping()) && (b.Type() == Static || b.IsSleeping()) {
		return true
	}

	ticks := space.stamp - arb.stamp

	if ticks >= 1 && arb.state != ArbiterStateCached {
		arb.state = ArbiterStateCached
		handler := arb.handler
		handler.SeparateFunc(arb, space, handler.UserData)
	}

	if ticks >= space.CollisionPersistence {
		arb.contacts = nil
		arb.count = 0
		space.pooledArbiters.Put(arb)
		return false
	}

	return true
}

func CachedArbitersFilter(arb *Arbiter, space *Space, shape *Shape, body *Body) bool {
	// Match on the filter shape, or if it's nil the filter body
	if (body == arb.bodyA && (shape == arb.shapeA || shape == nil)) ||
		(body == arb.bodyB && (shape == arb.shapeB || shape == nil)) {
		// Call separate when removing shapes.
		if shape != nil && arb.state != ArbiterStateCached {
			// Invalidate the arbiter since one of the shapes was removed
			arb.state = ArbiterStateInvalidated

			handler := arb.handler
			handler.SeparateFunc(arb, space, handler.UserData)
		}

		arb.Unthread()
		for i, arbiter := range space.Arbiters {
			if arb == arbiter {
				space.Arbiters = append(space.Arbiters[:i], space.Arbiters[i+1:]...)
				break
			}
		}
		space.pooledArbiters.Put(arb)
		return false
	}

	return true
}
