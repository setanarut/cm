package cm

import (
	"math"

	"github.com/setanarut/v"
)

type SlideJoint struct {
	*Constraint

	AnchorA, AnchorB v.Vec
	Min, Max         float64

	r1, r2, n v.Vec
	nMass     float64

	jnAcc, bias float64
}

func NewSlideJoint(a, b *Body, anchorA, anchorB v.Vec, min, max float64) *Constraint {
	joint := &SlideJoint{
		AnchorA: anchorA,
		AnchorB: anchorB,
		Min:     min,
		Max:     max,
		jnAcc:   0,
	}
	joint.Constraint = NewConstraint(joint, a, b)
	return joint.Constraint
}

func (joint *SlideJoint) PreStep(dt float64) {
	a := joint.bodyA
	b := joint.bodyB

	joint.r1 = a.transform.ApplyVector(joint.AnchorA.Sub(a.centerOfGravity))
	joint.r2 = b.transform.ApplyVector(joint.AnchorB.Sub(b.centerOfGravity))

	delta := b.position.Add(joint.r2).Sub(a.position.Add(joint.r1))
	dist := delta.Mag()
	pdist := 0.0
	if dist > joint.Max {
		pdist = dist - joint.Max
		joint.n = delta.Unit()
	} else if dist < joint.Min {
		pdist = joint.Min - dist
		joint.n = delta.Unit().Neg()
	} else {
		joint.n = v.Vec{}
		joint.jnAcc = 0
	}

	// calculate the mass normal
	joint.nMass = 1.0 / kScalar(a, b, joint.r1, joint.r2, joint.n)

	// calculate bias velocity
	maxBias := joint.maxBias
	joint.bias = clamp(-biasCoef(joint.errorBias, dt)*pdist/dt, -maxBias, maxBias)
}

func (joint *SlideJoint) ApplyCachedImpulse(dtCoef float64) {
	a := joint.bodyA
	b := joint.bodyB

	j := joint.n.Scale(joint.jnAcc * dtCoef)
	applyImpulses(a, b, joint.r1, joint.r2, j)
}

func (joint *SlideJoint) ApplyImpulse(dt float64) {
	if joint.n.Equals(v.Vec{}) {
		return
	}

	a := joint.bodyA
	b := joint.bodyB
	n := joint.n
	r1 := joint.r1
	r2 := joint.r2

	vr := relativeVelocity(a, b, r1, r2)
	vrn := vr.Dot(n)

	jn := (joint.bias - vrn) * joint.nMass
	jnOld := joint.jnAcc
	joint.jnAcc = clamp(jnOld+jn, -joint.maxForce*dt, 0)
	jn = joint.jnAcc - jnOld

	applyImpulses(a, b, joint.r1, joint.r2, n.Scale(jn))
}

func (joint *SlideJoint) GetImpulse() float64 {
	return math.Abs(joint.jnAcc)
}
