package cm_test

import (
	"math"
	"testing"

	"github.com/setanarut/cm"
	"github.com/setanarut/vec"
)

func TestShapeMass(t *testing.T) {
	body := cm.NewBody(0, 0)
	cm.NewCircleShape(body, 5, vec.Vec2{0, 0})

	mass := 10.0
	body.ShapeAtIndex(0).SetMass(mass)

	if body.ShapeAtIndex(0).Mass() != mass {
		t.Fail()
	}
}

func TestShapeCircleArea(t *testing.T) {
	body := cm.NewBody(0, 0)
	cm.NewCircleShape(body, 2, vec.Vec2{0, 0})

	if body.ShapeAtIndex(0).Area() != 4*math.Pi {
		t.Fail()
	}
}

func TestShapeCircleDensity(t *testing.T) {
	body := cm.NewBody(0, 0)
	cm.NewCircleShape(body, 1, vec.Vec2{0, 0})
	circle := body.ShapeAtIndex(0)
	circle.SetMass(math.Pi)

	if circle.Density() != 1.0 {
		t.Fail()
	}
}

// Test function for IsShort
func TestIsShort(t *testing.T) {

	pl := &cm.PolyLine{
		Verts: []vec.Vec2{
			{X: 0, Y: 0},
			{X: 3, Y: 4}, // Distance = 5 (Pythagoras: 3^2 + 4^2 = 5^2)
			{X: 6, Y: 8}, // Another 5 units from the previous point
		},
	}

	// Test case where total length is less than the min threshold
	if !pl.IsShort(3, 0, 2, 11) {
		t.Errorf("Expected IsShort to return true, but got false")
	}

	// Test case where total length exceeds the min threshold
	if pl.IsShort(3, 0, 2, 9) {
		t.Errorf("Expected IsShort to return false, but got true")
	}

	// Test with an exact match to the min threshold
	if !pl.IsShort(3, 0, 2, 10) {
		t.Errorf("Expected IsShort to return true, but got false")
	}

	// Test with same start and end index (no distance should be counted)
	if !pl.IsShort(3, 0, 0, 1) {
		t.Errorf("Expected IsShort to return true for same start and end index")
	}
}
