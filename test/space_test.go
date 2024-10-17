package cm_test

import (
	"testing"

	"github.com/setanarut/cm"
	"github.com/setanarut/vec"
)

func TestSpaceShapeQuery(t *testing.T) {
	space := cm.NewSpace()
	circle := space.AddShape(cm.NewCircle(space.StaticBody, 1, vec.Vec2{}))
	space.ShapeQuery(circle, func(shape *cm.Shape, points *cm.ContactPointSet) {
		t.Fatal("Shouldn't collide with itself")
	})
	box := cm.NewBox(cm.NewBody(1, 1), 1, 1, 1)

	var called int
	space.ShapeQuery(box, func(shape *cm.Shape, points *cm.ContactPointSet) {
		called++
	})
	if called != 1 {
		t.Error("Expected box to collide with circle")
	}

	box.Body().SetPosition(vec.Vec2{3, 0})
	space.ShapeQuery(box, func(shape *cm.Shape, points *cm.ContactPointSet) {
		t.Error("Box should be just out of range")
	})
}
func TestSpaceAddBody(t *testing.T) {
	s := cm.NewSpace()
	b := s.AddBody(cm.NewBody(1, 12))
	s.AddBody(b)

}

func TestSpace_ReindexShape(t *testing.T) {
	space := cm.NewSpace()
	circle := space.AddShape(cm.NewCircle(space.StaticBody, 1, vec.Vec2{}))
	bb1 := circle.BB()
	space.ReindexShape(circle)
	bb2 := circle.BB()
	// check unchanged
	if got, want := bb1.String(), bb2.String(); got != want {
		t.Errorf("got [%[1]v:%[1]T] want [%[2]v:%[2]T]", got, want)
	}
	circle.Body().SetPosition(vec.Vec2{X: 12.0, Y: 34.0})
	space.ReindexShape(circle)
	bb3 := circle.BB()
	// check changed
	if got, want := bb2.String(), bb3.String(); got == want {
		t.Errorf("got [%[1]v:%[1]T] want [%[2]v:%[2]T]", got, want)
	}
}
