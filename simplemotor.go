package cm

import "math"

type SimpleMotor struct {
	*Constraint

	Rate float64

	iSum, jAcc float64
}

func NewSimpleMotor(a, b *Body, rate float64) *Constraint {
	motor := &SimpleMotor{
		Rate: rate,
	}
	motor.Constraint = NewConstraint(motor, a, b)
	return motor.Constraint
}

func (motor *SimpleMotor) PreStep(dt float64) {
	a := motor.bodyA
	b := motor.bodyB

	// moment of inertia coefficient
	motor.iSum = 1.0 / (a.momentOfInertiaInverse + b.momentOfInertiaInverse)
}

func (motor *SimpleMotor) ApplyCachedImpulse(dtCoef float64) {
	a := motor.bodyA
	b := motor.bodyB

	j := motor.jAcc * dtCoef
	a.w -= j * a.momentOfInertiaInverse
	b.w += j * b.momentOfInertiaInverse
}

func (motor *SimpleMotor) ApplyImpulse(dt float64) {
	a := motor.bodyA
	b := motor.bodyB

	wr := b.w - a.w + motor.Rate

	jMax := motor.maxForce * dt

	j := -wr * motor.iSum
	jOld := motor.jAcc
	motor.jAcc = clamp(jOld+j, -jMax, jMax)
	j = motor.jAcc - jOld

	a.w -= j * a.momentOfInertiaInverse
	b.w += j * b.momentOfInertiaInverse
}

func (motor *SimpleMotor) GetImpulse() float64 {
	return math.Abs(motor.jAcc)
}
