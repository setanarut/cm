package cm

import (
	"math"

	"github.com/setanarut/vec"
)

// Transform represents a 2D affine transformation using a 2x3 matrix.
// This structure can be used to perform various transformations such as
// translation, rotation, scaling, and shearing of 2D objects.
//
// The transformation matrix is represented as follows:
//
//	| a  b  tx |   -> X' = a * X + b * Y + tx
//	| c  d  ty |   -> Y' = c * X + d * Y + ty
//
// Where:
//   - a: Scaling factor in the X direction and component for rotation.
//   - b: Shearing factor in the X direction (affects X based on Y).
//   - c: Shearing factor in the Y direction (affects Y based on X).
//   - d: Scaling factor in the Y direction and component for rotation.
//   - tx: Translation in the X direction.
//   - ty: Translation in the Y direction.
//
// The Transform type can be used to compose multiple transformations,
// enabling complex transformations by multiplying matrices together.
type Transform struct {
	a, b, c, d, tx, ty float64
}

// NewTransformIdentity creates and returns an identity transformation.
// The identity transformation is a special case of affine transformation
// that leaves the object's coordinates unchanged.
//
// The identity matrix in 2D transforms points according to the following:
//
//	| 1  0  0 |   -> X' = 1 * X + 0 * Y + 0 = X
//	| 0  1  0 |   -> Y' = 0 * X + 1 * Y + 0 = Y
//
// The identity transform is useful as a starting point for composing
// transformations, as applying it to any object will yield the same
// object without any modifications.
func NewTransformIdentity() Transform {
	return Transform{1, 0, 0, 1, 0, 0}
}

// NewTransform returns a new transform matrix.
//
// Parameters:
//   - (a, b) is the x basis vector.
//   - (c, d) is the y basis vector.
//   - (tx, ty) is the translation.
func NewTransform(a, c, tx, b, d, ty float64) Transform {
	return Transform{a, b, c, d, tx, ty}
}

// NewTransformTranspose returns a new transformation matrix in transposed order.
func NewTransformTranspose(a, c, tx, b, d, ty float64) Transform {
	return Transform{a, b, c, d, tx, ty}
}

// Inverse returns the inverse of this matrix t.
func (t Transform) Inverse() Transform {
	invDet := 1.0 / (t.a*t.d - t.c*t.b)
	return NewTransformTranspose(
		t.d*invDet, -t.c*invDet, (t.c*t.ty-t.tx*t.d)*invDet,
		-t.b*invDet, t.a*invDet, (t.tx*t.b-t.a*t.ty)*invDet,
	)
}

// Mult multiplies this and t2
//
// Parameters:
//
//   - t2 - The Transform to be multiplied with the receiver.
//
// Returns:
//
//   - A new Transform representing the result of the multiplication.
func (t Transform) Mult(t2 Transform) Transform {
	return NewTransformTranspose(
		t.a*t2.a+t.c*t2.b, t.a*t2.c+t.c*t2.d, t.a*t2.tx+t.c*t2.ty+t.tx,
		t.b*t2.a+t.d*t2.b, t.b*t2.c+t.d*t2.d, t.b*t2.tx+t.d*t2.ty+t.ty,
	)
}

// NewTransformTranslate returns a new transformation matrix with translation
func NewTransformTranslate(translate vec.Vec2) Transform {
	return NewTransformTranspose(
		1, 0, translate.X,
		0, 1, translate.Y,
	)
}

// NewTransformScale returns a new transformation with scaling
func NewTransformScale(scaleX, scaleY float64) Transform {
	return NewTransformTranspose(
		scaleX, 0, 0,
		0, scaleY, 0,
	)
}

// NewTransformRotate returns a new rigid transformation with rotation
func NewTransformRotate(rotation float64) Transform {
	rot := vec.ForAngle(rotation)
	return NewTransformTranspose(
		rot.X, -rot.Y, 0,
		rot.Y, rot.X, 0,
	)
}

// NewTransformRigid creates a new rigid transformation that combines
// translation and rotation.
//
// Rigid transformation, or rigid motion, refers to a transformation that
// preserves the shape and size of objects while allowing them to change
// position and orientation in space. It includes translation, rotation,
// and reflection, maintaining distances and angles within the object.
//
// Parameters:
//   - translate: A 2D vector specifying the translation component.
//   - rotation: The angle of rotation in radians.
//
// Returns:
//   - A Transform representing the combined translation and rotation.
func NewTransformRigid(translate vec.Vec2, rotation float64) Transform {
	rot := vec.ForAngle(rotation)
	return NewTransformTranspose(
		rot.X, -rot.Y, translate.X,
		rot.Y, rot.X, translate.Y,
	)
}

// NewTransformRigidInverse returns the inverse of a given rigid transformation.
//
// Rigid transformation, or rigid motion, refers to a transformation that
// preserves the shape and size of objects while allowing them to change
// position and orientation in space. It includes translation, rotation,
// and reflection, maintaining distances and angles within the object.
//
// Parameters:
//   - t: The original Transform to invert.
//
// Returns:
//   - The inverted Transform.
func NewTransformRigidInverse(t Transform) Transform {
	return NewTransformTranspose(
		t.d, -t.c, t.c*t.ty-t.tx*t.d,
		-t.b, t.a, t.tx*t.b-t.a*t.ty,
	)
}

// Apply applies the transformation to a given abs point `p` and returns the transformed point.
// This transformation involves scaling, rotation, and translation based on the matrix values.
func (t Transform) Apply(p vec.Vec2) vec.Vec2 {
	return vec.Vec2{
		X: t.a*p.X + t.c*p.Y + t.tx,
		Y: t.b*p.X + t.d*p.Y + t.ty,
	}
}

// ApplyVector applies the transformation matrix (t) to a vector (v).
// This transformation modifies the vector's direction based on the
// current transformation without affecting its position.
//
// Parameters:
//   - v: the vector (Vec2) to be transformed, represented by X and Y coordinates.
//
// Returns:
//   - Vec2: the transformed vector after applying the matrix transformation.
func (t Transform) ApplyVector(v vec.Vec2) vec.Vec2 {
	return vec.Vec2{
		t.a*v.X + t.c*v.Y,
		t.b*v.X + t.d*v.Y,
	}
}

// BB applies the current transformation (t) to a bounding box (BB).
// It scales and rotates the bounding box around its center to create a
// transformed bounding box that accounts for the dimensions after transformation.
//
// Parameters:
//   - bb: the bounding box (BB) to be transformed, defined by its left (L), right (R),
//     top (T), and bottom (B) boundaries.
//
// Returns:
//   - BB: the transformed bounding box, with dimensions adjusted based on the
//     transformation applied.
func (t Transform) BB(bb BB) BB {
	hw := (bb.R - bb.L) * 0.5
	hh := (bb.T - bb.B) * 0.5

	a := t.a * hw
	b := t.c * hh
	d := t.b * hw
	e := t.d * hh
	hwMax := math.Max(math.Abs(a+b), math.Abs(a-b))
	hhMax := math.Max(math.Abs(d+e), math.Abs(d-e))
	return NewBBForExtents(t.Apply(bb.Center()), hwMax, hhMax)
}

// Wrap applies the inverse of the current transformation (t) around
// another transformation (inner). This wraps inner with the inverse
// of t, then applies t, effectively creating a nested transformation.
//
// Parameters:
//   - inner: the Transform to wrap with the inverse of the current transformation.
//
// Returns:
//   - Transform: the resulting transformation after wrapping.
func (t Transform) Wrap(inner Transform) Transform {
	return t.Inverse().Mult(inner.Mult(t))
}

// Ortho creates an orthographic transformation matrix based on the given bounding box (bb).
// It performs scaling and translation to fit coordinates within the bounding box.
//
// Parameters:
// - bb: A bounding box with left (L), right (R), bottom (B), and top (T) boundaries.
//
// Returns:
// - A Transform that represents the orthographic projection transformation matrix.
func (t Transform) Ortho(bb BB) Transform {
	return NewTransformTranspose(
		2.0/(bb.R-bb.L), 0.0, -(bb.R+bb.L)/(bb.R-bb.L),
		0.0, 2.0/(bb.T-bb.B), -(bb.T+bb.B)/(bb.T-bb.B),
	)
}

// BoneScale creates a scaling and rotation transformation matrix based on a bone vector
// defined by the start and end points v0 and v1. It translates to v0 and scales along
// the direction to v1.
//
// Parameters:
// - v0: The starting vector of the bone.
// - v1: The ending vector of the bone.
//
// Returns:
// - A Transform that represents the scaling and rotation transformation matrix.
func (t Transform) BoneScale(v0, v1 vec.Vec2) Transform {
	d := v1.Sub(v0)
	return NewTransformTranspose(
		d.X, -d.Y, v0.X,
		d.Y, d.X, v0.Y,
	)
}

// Creates a transformation matrix for scaling along a given axis.
//
// This function generates a transformation matrix that scales along a specified axis
// around a pivot point. If the scaling factor (scale) is different from 1.0,
// the transformation matrix will apply scaling along the direction of the axis.
//
// Parameters:
//   - axis: A vector representing the axis along which scaling will be applied.
//   - pivot: The pivot point around which scaling is applied.
//   - scale: The scaling factor. Values other than 1.0 scale the axis direction accordingly.
//
// Returns:
//   - Transform: Axial scale transformation matrix.
func (t Transform) AxialScale(axis, pivot vec.Vec2, scale float64) Transform {
	A := axis.X * axis.Y * (scale - 1.0)
	B := axis.Dot(pivot) * (1.0 - scale)

	return NewTransformTranspose(
		scale*axis.X*axis.X+axis.Y*axis.Y, A, axis.X*B,
		A, axis.X*axis.X+scale*axis.Y*axis.Y, axis.Y*B,
	)
}

// Scale matrix inplace
func (t *Transform) Scale(x, y float64) {
	t.a *= x
	t.d *= y
}
