package cm

import "math"

type RotaryLimitJoint struct {
	*Constraint

	Min, Max float64

	iSum, bias, jAcc float64
}

func NewRotaryLimitJoint(a, b *Body, min, max float64) *Constraint {
	joint := &RotaryLimitJoint{
		Min: min,
		Max: max,
	}
	joint.Constraint = NewConstraint(joint, a, b)
	return joint.Constraint
}

func (joint *RotaryLimitJoint) PreStep(dt float64) {
	a := joint.bodyA
	b := joint.bodyB

	dist := b.angle - a.angle
	pdist := 0.0
	if dist > joint.Max {
		pdist = joint.Max - dist
	} else if dist < joint.Min {
		pdist = joint.Min - dist
	}

	joint.iSum = 1.0 / (a.momentOfInertiaInverse + b.momentOfInertiaInverse)

	maxBias := joint.maxBias
	joint.bias = clamp(-biasCoef(joint.errorBias, dt)*pdist/dt, -maxBias, maxBias)

	if joint.bias == 0 {
		joint.jAcc = 0
	}
}

func (joint *RotaryLimitJoint) ApplyCachedImpulse(dtCoef float64) {
	a := joint.bodyA
	b := joint.bodyB

	j := joint.jAcc * dtCoef
	a.w -= j * a.momentOfInertiaInverse
	b.w += j * b.momentOfInertiaInverse
}

func (joint *RotaryLimitJoint) ApplyImpulse(dt float64) {
	if joint.bias == 0 {
		return
	}

	a := joint.bodyA
	b := joint.bodyB

	wr := b.w - a.w

	jMax := joint.maxForce * dt

	j := -(joint.bias + wr) * joint.iSum
	jOld := joint.jAcc
	if joint.bias < 0 {
		joint.jAcc = clamp(jOld+j, 0, jMax)
	} else {
		joint.jAcc = clamp(jOld+j, -jMax, 0)
	}
	j = joint.jAcc - jOld

	a.w -= j * a.momentOfInertiaInverse
	b.w += j * b.momentOfInertiaInverse
}

func (joint *RotaryLimitJoint) GetImpulse() float64 {
	return math.Abs(joint.jAcc)
}
