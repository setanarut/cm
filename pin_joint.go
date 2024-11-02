package cm

import (
	"math"

	"github.com/setanarut/vec"
)

type PinJoint struct {
	*Constraint
	AnchorA, AnchorB vec.Vec2
	Dist             float64

	r1, r2, n          vec.Vec2
	nMass, jnAcc, bias float64
}

func NewPinJoint(a, b *Body, anchorA, anchorB vec.Vec2) *Constraint {
	joint := &PinJoint{
		AnchorA: anchorA,
		AnchorB: anchorB,
	}

	// static body check
	var p1, p2 vec.Vec2
	if a != nil {
		p1 = a.transform.Apply(anchorA)
	} else {
		p1 = anchorA
	}
	if b != nil {
		p2 = b.transform.Apply(anchorB)
	} else {
		p2 = anchorB
	}
	joint.Dist = p2.Sub(p1).Mag()

	// TODO: warn about joint.dist > 0 being unstable, use pivot joint

	joint.jnAcc = 0

	joint.Constraint = NewConstraint(joint, a, b)
	return joint.Constraint
}

func (joint *PinJoint) PreStep(dt float64) {
	a := joint.bodyA
	b := joint.bodyB

	joint.r1 = a.transform.ApplyVector(joint.AnchorA.Sub(a.centerOfGravity))
	joint.r2 = b.transform.ApplyVector(joint.AnchorB.Sub(b.centerOfGravity))

	delta := b.position.Add(joint.r2.Sub(a.position.Add(joint.r1)))
	dist := delta.Mag()
	if dist != 0 {
		joint.n = delta.Scale(1 / dist)
	} else {
		joint.n = delta.Scale(1 / infinity)
	}

	joint.nMass = 1 / kScalar(a, b, joint.r1, joint.r2, joint.n)

	maxBias := joint.maxBias
	joint.bias = clamp(-biasCoef(joint.errorBias, dt)*(dist-joint.Dist)/dt, -maxBias, maxBias)
}

func (joint *PinJoint) ApplyCachedImpulse(dtCoef float64) {
	j := joint.n.Scale(joint.jnAcc * dtCoef)
	applyImpulses(joint.bodyA, joint.bodyB, joint.r1, joint.r2, j)
}

func (joint *PinJoint) ApplyImpulse(dt float64) {
	a := joint.bodyA
	b := joint.bodyB
	n := joint.n

	vrn := normalRelativeVelocity(a, b, joint.r1, joint.r2, n)

	jnMax := joint.maxForce * dt

	jn := (joint.bias - vrn) * joint.nMass
	jnOld := joint.jnAcc
	joint.jnAcc = clamp(jnOld+jn, -jnMax, jnMax)
	jn = joint.jnAcc - jnOld

	applyImpulses(a, b, joint.r1, joint.r2, n.Scale(jn))
}

func (joint *PinJoint) GetImpulse() float64 {
	return math.Abs(joint.jnAcc)
}
