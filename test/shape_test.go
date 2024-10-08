package cm_test

import (
	"math"
	"testing"

	"github.com/setanarut/cm"
	"github.com/setanarut/vec"
)

func TestShapeMass(t *testing.T) {
	body := cm.NewBody(0, 0)
	circle := cm.NewCircle(body, 5, vec.Vec2{0, 0})

	mass := 10.0
	circle.SetMass(mass)
	body.AddShape(circle)

	if circle.Mass() != mass {
		t.Fail()
	}
}

func TestShapeCircleArea(t *testing.T) {
	body := cm.NewBody(0, 0)
	circle := cm.NewCircle(body, 2, vec.Vec2{0, 0})

	if circle.Area() != 4*math.Pi {
		t.Fail()
	}
}

func TestShapeCircleDensity(t *testing.T) {
	body := cm.NewBody(0, 0)
	circle := cm.NewCircle(body, 1, vec.Vec2{0, 0})

	circle.SetMass(math.Pi)

	if circle.Density() != 1.0 {
		t.Fail()
	}
}
