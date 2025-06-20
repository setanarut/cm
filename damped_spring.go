package cm

import (
	"math"

	"github.com/setanarut/v"
)

type DampedSpringForceFunc func(spring *DampedSpring, dist float64) float64

type DampedSpring struct {
	*Constraint

	AnchorA, AnchorB               v.Vec
	RestLength, Stiffness, Damping float64
	SpringForceFunc                DampedSpringForceFunc

	targetVrn, vCoef float64

	r1, r2 v.Vec
	nMass  float64
	n      v.Vec

	jAcc float64
}

func NewDampedSpring(a, b *Body, anchorA, anchorB v.Vec, restLength, stiffness, damping float64) *Constraint {
	spring := &DampedSpring{
		AnchorA:         anchorA,
		AnchorB:         anchorB,
		RestLength:      restLength,
		Stiffness:       stiffness,
		Damping:         damping,
		SpringForceFunc: DefaultSpringForce,
		jAcc:            0,
	}
	spring.Constraint = NewConstraint(spring, a, b)
	return spring.Constraint
}

func (spring *DampedSpring) PreStep(dt float64) {
	a := spring.bodyA
	b := spring.bodyB

	spring.r1 = a.transform.ApplyVector(spring.AnchorA.Sub(a.centerOfGravity))
	spring.r2 = b.transform.ApplyVector(spring.AnchorB.Sub(b.centerOfGravity))

	delta := b.position.Add(spring.r2).Sub(a.position.Add(spring.r1))
	dist := delta.Mag()
	if dist != 0 {
		spring.n = delta.Scale(1.0 / dist)
	} else {
		spring.n = delta.Scale(1.0 / infinity)
	}

	k := kScalar(a, b, spring.r1, spring.r2, spring.n)

	// if k == 0 {
	// 	log.Fatalln("Unsolvable spring")
	// }

	spring.nMass = 1.0 / k

	spring.targetVrn = 0
	spring.vCoef = 1.0 - math.Exp(-spring.Damping*dt*k)

	fSpring := spring.SpringForceFunc(spring, dist)
	spring.jAcc = fSpring * dt
	applyImpulses(a, b, spring.r1, spring.r2, spring.n.Scale(spring.jAcc))
}

func (spring *DampedSpring) ApplyCachedImpulse(dtCoef float64) {
	// nothing to do here
}

func (spring *DampedSpring) ApplyImpulse(dt float64) {
	a := spring.bodyA
	b := spring.bodyB

	n := spring.n
	r1 := spring.r1
	r2 := spring.r2

	vrn := normalRelativeVelocity(a, b, r1, r2, n)

	vDamp := (spring.targetVrn - vrn) * spring.vCoef
	spring.targetVrn = vrn + vDamp

	jDamp := vDamp * spring.nMass
	spring.jAcc += jDamp
	applyImpulses(a, b, spring.r1, spring.r2, spring.n.Scale(jDamp))
}

func (spring *DampedSpring) GetImpulse() float64 {
	return spring.jAcc
}

func DefaultSpringForce(spring *DampedSpring, dist float64) float64 {
	return (spring.RestLength - dist) * spring.Stiffness
}
