package cm

const (
	// Value for group signifying that a shape is in no group.
	NoGroup uint = 0
	// Value for Shape layers signifying that a shape is in every layer.
	AllCategories uint = ^uint(0)
)

// ShapeFilterAll is s collision filter value for a shape that will collide with
// anything except ShapeFilterNone.
var ShapeFilterAll = ShapeFilter{NoGroup, AllCategories, AllCategories}

// ShapeFilterNone is a collision filter value for a shape that does not collide
// with anything.
var ShapeFilterNone = ShapeFilter{NoGroup, ^AllCategories, ^AllCategories}

// ShapeFilter is fast collision filtering type that is used to determine if two objects collide before calling collision or query callbacks.
type ShapeFilter struct {
	// Two objects with the same non-zero group value do not collide.
	// This is generally used to group objects in a composite object together to disable self collisions.
	Group uint
	// A bitmask of user definable categories that this object belongs to.
	// The category/mask combinations of both objects in a collision must agree for a collision to occur.
	Categories uint
	// A bitmask of user definable category types that this object object collides with.
	// The category/mask combinations of both objects in a collision must agree for a collision to occur.
	Mask uint
}

// Reject checks whether two ShapeFilter objects should be considered incompatible.
// It returns true if the filters should be rejected based on the following conditions:
// - If both filters belong to the same group (and the group is not 0).
// - If the category/mask combination of either filter does not match the other.
//   - Specifically, it checks if the categories of the first filter don't match
//     the mask of the second, or vice versa.
//
// Returns true if the filters are considered incompatible, otherwise false.
func (sf ShapeFilter) Reject(other ShapeFilter) bool {
	return (sf.Group != 0 && sf.Group == other.Group) ||
		(sf.Categories&other.Mask) == 0 ||
		(other.Categories&sf.Mask) == 0
}

// CollideWith enables collision with the specified category mask.
func (sf *ShapeFilter) CollideWith(mask uint) {
	sf.Mask |= mask
}

// DontCollideWith disables collision with the specified category mask.
func (sf *ShapeFilter) DontCollideWith(mask uint) {
	sf.Mask &= ^mask
}
