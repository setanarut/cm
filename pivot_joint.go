package cm

import "github.com/setanarut/v"

type PivotJoint struct {
	*Constraint
	AnchorA, AnchorB v.Vec

	r1, r2 v.Vec
	k      Mat2x2

	jAcc, bias v.Vec
}

func NewPivotJoint(a, b *Body, pivot v.Vec) *Constraint {
	var anchorA v.Vec
	var anchorB v.Vec

	if a != nil {
		anchorA = a.WorldToLocal(pivot)
	} else {
		anchorA = pivot
	}

	if b != nil {
		anchorB = b.WorldToLocal(pivot)
	} else {
		anchorB = pivot
	}

	return NewPivotJoint2(a, b, anchorA, anchorB)
}

func NewPivotJoint2(a, b *Body, anchorA, anchorB v.Vec) *Constraint {
	joint := &PivotJoint{
		AnchorA: anchorA,
		AnchorB: anchorB,
		jAcc:    v.Vec{},
	}
	constraint := NewConstraint(joint, a, b)
	joint.Constraint = constraint
	return constraint
}

func (joint *PivotJoint) PreStep(dt float64) {
	a := joint.Constraint.bodyA
	b := joint.Constraint.bodyB

	joint.r1 = a.transform.ApplyVector(joint.AnchorA.Sub(a.centerOfGravity))
	joint.r2 = b.transform.ApplyVector(joint.AnchorB.Sub(b.centerOfGravity))

	// Calculate mass tensor
	joint.k = kTensor(a, b, joint.r1, joint.r2)

	// calculate bias velocity
	delta := b.position.Add(joint.r2).Sub(a.position.Add(joint.r1))
	joint.bias = clampMag(delta.Scale(-biasCoef(joint.Constraint.errorBias, dt)/dt), joint.Constraint.maxBias)
}

func (joint *PivotJoint) ApplyCachedImpulse(dtCoef float64) {
	applyImpulses(joint.bodyA, joint.bodyB, joint.r1, joint.r2, joint.jAcc.Scale(dtCoef))
}

func (joint *PivotJoint) ApplyImpulse(dt float64) {
	a := joint.Constraint.bodyA
	b := joint.Constraint.bodyB

	r1 := joint.r1
	r2 := joint.r2

	// compute relative velocity
	vr := relativeVelocity(a, b, r1, r2)

	// compute normal impulse
	j := joint.k.Transform(joint.bias.Sub(vr))
	jOld := joint.jAcc
	joint.jAcc = clampMag(joint.jAcc.Add(j), joint.Constraint.maxForce*dt)
	j = joint.jAcc.Sub(jOld)

	applyImpulses(a, b, joint.r1, joint.r2, j)
}

func (joint *PivotJoint) GetImpulse() float64 {
	return joint.jAcc.Mag()
}
