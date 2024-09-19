package cm

import "math"

type GearJoint struct {
	*Constraint
	phase, ratio float64
	ratioInv     float64

	iSum float64

	bias, jAcc float64
}

func NewGearJoint(a, b *Body, phase, ratio float64) *Constraint {
	joint := &GearJoint{
		phase:    phase,
		ratio:    ratio,
		ratioInv: 1.0 / ratio,
		jAcc:     0,
	}
	constraint := NewConstraint(joint, a, b)
	joint.Constraint = constraint
	return constraint
}

func (joint *GearJoint) PreStep(dt float64) {
	a := joint.bodyA
	b := joint.bodyB

	// calculate moment of inertia coefficient.
	joint.iSum = 1.0 / (a.momentOfInertiaInverse*joint.ratioInv + joint.ratio*b.momentOfInertiaInverse)

	// calculate bias velocity
	maxBias := joint.Constraint.maxBias
	joint.bias = clamp(-biasCoef(joint.errorBias, dt)*(b.angle*joint.ratio-a.angle-joint.phase)/dt, -maxBias, maxBias)
}

func (joint *GearJoint) ApplyCachedImpulse(dtCoef float64) {
	a := joint.bodyA
	b := joint.bodyB

	j := joint.jAcc * dtCoef
	a.w -= j * a.momentOfInertiaInverse * joint.ratioInv
	b.w += j * b.momentOfInertiaInverse
}

func (joint *GearJoint) ApplyImpulse(dt float64) {
	a := joint.bodyA
	b := joint.bodyB

	// compute relative rotational velocity
	wr := b.w*joint.ratio - a.w

	jMax := joint.Constraint.maxForce * dt

	// compute normal impulse
	j := (joint.bias - wr) * joint.iSum
	jOld := joint.jAcc
	joint.jAcc = clamp(jOld+j, -jMax, jMax)
	j = joint.jAcc - jOld

	// apply impulse
	a.w -= j * a.momentOfInertiaInverse * joint.ratioInv
	b.w += j * b.momentOfInertiaInverse
}

func (joint *GearJoint) GetImpulse() float64 {
	return math.Abs(joint.jAcc)
}
