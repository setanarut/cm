package cm_test

import (
	"testing"

	"github.com/setanarut/cm"
	"github.com/setanarut/v"
)

func TestBodyMassFromShapes(t *testing.T) {
	body := cm.NewBody(0, 0)
	cm.NewCircleShape(body, 5, v.Vec{0, 0})
	cm.NewCircleShape(body, 5, v.Vec{0, 0})

	mass := 10.0
	body.ShapeAtIndex(0).SetMass(mass)

	if body.Mass() != mass {
		t.Fail()
	}

	body.ShapeAtIndex(1).SetMass(mass)
	if body.Mass() != mass*2 {
		t.Fail()
	}
}

func TestBodyCoGFromShapes(t *testing.T) {
	body := cm.NewBody(0, 0)
	cm.NewCircleShape(body, 5, v.Vec{0, 0})
	cm.NewCircleShape(body, 5, v.Vec{10, 0})

	mass := 10.0

	body.ShapeAtIndex(0).SetMass(mass)
	body.ShapeAtIndex(1).SetMass(mass)

	cog := body.CenterOfGravity()
	if cog.X != 5.0 || cog.Y != 0.0 {
		t.Fail()
	}

	body.RemoveShape(body.ShapeAtIndex(0))
	cog = body.CenterOfGravity()
	if cog.X != 10.0 || cog.Y != 0.0 {
		t.Fail()
	}

}
