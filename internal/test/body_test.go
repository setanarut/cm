package cm_test

import (
	"testing"

	"github.com/setanarut/cm"
)

func TestBodyMassFromShapes(t *testing.T) {
	body := cm.NewBody(0, 0)
	circle := cm.NewCircle(body, 5, cm.Vector{0, 0})
	circle2 := cm.NewCircle(body, 5, cm.Vector{0, 0})

	mass := 10.0
	circle.SetMass(mass)
	body.AddShape(circle)

	if body.Mass() != mass {
		t.Fail()
	}

	circle2.SetMass(mass)
	body.AddShape(circle2)

	if body.Mass() != mass*2 {
		t.Fail()
	}
}

func TestBodyCoGFromShapes(t *testing.T) {
	body := cm.NewBody(0, 0)
	circle := cm.NewCircle(body, 5, cm.Vector{0, 0})
	circle2 := cm.NewCircle(body, 5, cm.Vector{10, 0})

	mass := 10.0

	circle.SetMass(mass)
	body.AddShape(circle)

	circle2.SetMass(mass)
	body.AddShape(circle2)

	cog := body.CenterOfGravity()
	if cog.X != 5.0 || cog.Y != 0.0 {
		t.Fail()
	}

	body.RemoveShape(circle)
	cog = body.CenterOfGravity()
	if cog.X != 10.0 || cog.Y != 0.0 {
		t.Fail()
	}

}
