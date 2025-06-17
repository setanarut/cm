package cm

import "github.com/setanarut/v"

type GrooveJoint struct {
	*Constraint

	GrooveN, GrooveA, GrooveB v.Vec
	AnchorB                   v.Vec

	grooveTn v.Vec
	clamp    float64
	r1, r2   v.Vec
	k        Mat2x2

	jAcc, bias v.Vec
}

func NewGrooveJoint(a, b *Body, grooveA, grooveB, anchorB v.Vec) *Constraint {
	joint := &GrooveJoint{
		GrooveA: grooveA,
		GrooveB: grooveB,
		GrooveN: perp(grooveB.Sub(grooveA).Unit()),
		AnchorB: anchorB,
		jAcc:    v.Vec{},
	}
	joint.Constraint = NewConstraint(joint, a, b)
	return joint.Constraint
}

func (joint *GrooveJoint) PreStep(dt float64) {
	a := joint.bodyA
	b := joint.bodyB

	ta := a.transform.Apply(joint.GrooveA)
	tb := a.transform.Apply(joint.GrooveB)

	n := a.transform.ApplyVector(joint.GrooveN)
	d := ta.Dot(n)

	joint.grooveTn = n
	joint.r2 = b.transform.ApplyVector(joint.AnchorB.Sub(b.centerOfGravity))

	td := b.position.Add(joint.r2).Cross(n)

	if td <= ta.Cross(n) {
		joint.clamp = 1
		joint.r1 = ta.Sub(a.position)
	} else if td >= tb.Cross(n) {
		joint.clamp = -1
		joint.r1 = tb.Sub(a.position)
	} else {
		joint.clamp = 0
		joint.r1 = perp(n).Scale(-td).Add(n.Scale(d)).Sub(a.position)
	}

	joint.k = kTensor(a, b, joint.r1, joint.r2)

	delta := b.position.Add(joint.r2).Sub(a.position.Add(joint.r1))
	joint.bias = clampMag(delta.Scale(-biasCoef(joint.errorBias, dt)/dt), joint.maxBias)
}

func (joint *GrooveJoint) ApplyCachedImpulse(dtCoef float64) {
	a := joint.bodyA
	b := joint.bodyB

	applyImpulses(a, b, joint.r1, joint.r2, joint.jAcc.Scale(dtCoef))
}

func (joint *GrooveJoint) grooveConstrain(j v.Vec, dt float64) v.Vec {
	n := joint.grooveTn
	var jClamp v.Vec
	if joint.clamp*j.Cross(n) > 0 {
		jClamp = j
	} else {
		jClamp = j.Project(n)
	}
	return clampMag(jClamp, joint.maxForce*dt)
}

func (joint *GrooveJoint) ApplyImpulse(dt float64) {
	a := joint.bodyA
	b := joint.bodyB

	r1 := joint.r1
	r2 := joint.r2

	vr := relativeVelocity(a, b, r1, r2)

	j := joint.k.Transform(joint.bias.Sub(vr))
	jOld := joint.jAcc
	joint.jAcc = joint.grooveConstrain(jOld.Add(j), dt)
	j = joint.jAcc.Sub(jOld)

	applyImpulses(a, b, joint.r1, joint.r2, j)
}

func (joint *GrooveJoint) GetImpulse() float64 {
	return joint.jAcc.Mag()
}
