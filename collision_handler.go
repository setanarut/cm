package cm

var CollisionHandlerDoNothing = CollisionHandler{
	WildcardCollisionType,
	WildcardCollisionType,
	AlwaysCollide,
	AlwaysCollide,
	DoNothing,
	DoNothing,
	nil,
}

var CollisionHandlerDefault = CollisionHandler{
	WildcardCollisionType,
	WildcardCollisionType,
	DefaultBegin,
	DefaultPreSolve,
	DefaultPostSolve,
	DefaultSeparate,
	nil,
}

// CollisionBeginFunc is collision begin event function callback type.
// Returning false from a begin callback causes the collision to be ignored
// until the the separate callback is called when the objects stop colliding.
type CollisionBeginFunc func(arb *Arbiter, space *Space, userData any) bool

// CollisionPreSolveFunc is collision pre-solve event function callback type.
//
// Returning false from a pre-step callback causes the collision to be ignored until the next step.
type CollisionPreSolveFunc func(arb *Arbiter, space *Space, userData any) bool

// CollisionPostSolveFunc is collision post-solve event function callback type.
type CollisionPostSolveFunc func(arb *Arbiter, space *Space, userData any)

// CollisionSeparateFunc is collision separate event function callback type.
type CollisionSeparateFunc func(arb *Arbiter, space *Space, userData any)

// CollisionHandler is struct that holds function callback pointers to configure custom collision handling.
// Collision handlers have a pair of types; when a collision occurs between two shapes that have these types, the collision handler functions are triggered.
type CollisionHandler struct {
	// Collision type identifier of the first shape that this handler recognizes.
	// In the collision handler callback, the shape with this type will be the first argument. Read only.
	TypeA CollisionType
	// Collision type identifier of the second shape that this handler recognizes.
	// In the collision handler callback, the shape with this type will be the second argument. Read only.
	TypeB CollisionType
	// This function is called when two shapes with types that match this collision handler begin colliding.
	BeginFunc CollisionBeginFunc
	// This function is called each step when two shapes with types that match this collision handler are colliding.
	// It's called before the collision solver runs so that you can affect a collision's outcome.
	PreSolveFunc CollisionPreSolveFunc
	// This function is called each step when two shapes with types that match this collision handler are colliding.
	// It's called after the collision solver runs so that you can read back information about the collision to trigger events in your game.
	PostSolveFunc CollisionPostSolveFunc
	// This function is called when two shapes with types that match this collision handler stop colliding.
	SeparateFunc CollisionSeparateFunc
	// This is a user definable context pointer that is passed to all of the collision handler functions.
	UserData any
}
