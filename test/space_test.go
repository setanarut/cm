package cm_test

import (
	"testing"

	"github.com/setanarut/cm"
	"github.com/setanarut/v"
)

func TestSpaceShapeQuery(t *testing.T) {
	space := cm.NewSpace()
	sbody := cm.NewStaticBody()
	cm.NewCircleShape(sbody, 1, v.Vec{})
	circle := sbody.Shapes[0]
	space.AddShape(circle)

	space.ShapeQuery(circle, func(shape *cm.Shape, points *cm.ContactPointSet) {
		t.Fatal("Shouldn't collide with itself")
	})

	b := cm.NewBody(1, 1)
	cm.NewBoxShape(b, 1, 1, 1)
	space.AddShape(b.Shapes[0])

	var called int
	space.ShapeQuery(b.Shapes[0], func(shape *cm.Shape, points *cm.ContactPointSet) {
		called++
	})
	if called != 1 {
		t.Error("Expected box to collide with circle")
	}

	b.SetPosition(v.Vec{3, 0})

	space.ShapeQuery(b.Shapes[0], func(shape *cm.Shape, points *cm.ContactPointSet) {
		t.Error("Box should be just out of range")
	})
}
func TestSpaceAddBody(t *testing.T) {
	s := cm.NewSpace()
	b := cm.NewBody(1, 12)
	s.AddBody(b)
	if b.Space != s {
		t.Error("space is not same")
	}
}

func TestSpaceRemoveBody(t *testing.T) {
	s := cm.NewSpace()
	b := cm.NewBody(1, 12)
	s.AddBody(b)
	if s.DynamicBodyCount() != 1 {
		t.Error("should have one body")
	}
	s.RemoveBody(b)
	if s.DynamicBodyCount() != 0 {
		t.Error("should not have any bodies")
	}
}

func TestSpace_ReindexShape(t *testing.T) {
	space := cm.NewSpace()
	sb := cm.NewStaticBody()
	cm.NewCircleShape(sb, 1, v.Vec{})
	circle := sb.ShapeAtIndex(0)
	space.AddBodyWithShapes(sb)
	bb1 := circle.BB
	space.ReindexShape(circle)
	bb2 := circle.BB
	// check unchanged
	if got, want := bb1.String(), bb2.String(); got != want {
		t.Errorf("got [%[1]v:%[1]T] want [%[2]v:%[2]T]", got, want)
	}
	circle.Body.SetPosition(v.Vec{X: 12.0, Y: 34.0})
	space.ReindexShape(circle)
	bb3 := circle.BB
	// check changed
	if got, want := bb2.String(), bb3.String(); got == want {
		t.Errorf("got [%[1]v:%[1]T] want [%[2]v:%[2]T]", got, want)
	}
}
