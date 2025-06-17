package cm

import "github.com/setanarut/v"

type SpatialIndexBB func(obj *Shape) BB
type SpatialIndexIterator func(obj *Shape)
type SpatialIndexQuery func(obj1 any, obj2 *Shape, collisionId uint32, data any) uint32
type SpatialIndexSegmentQuery func(obj1 any, obj2 *Shape, data any) float64

// SpatialIndexer is an interface for spatial indexing that provides
// methods to manage spatial objects efficiently. It is implemented by
// a BBTree structure, which organizes objects in a bounding volume tree.
type SpatialIndexer interface {
	// Count returns the number of objects currently stored in the index.
	Count() int

	// Each iterates over all objects in the spatial index, applying
	// the provided iterator function `f` to each object.
	Each(f SpatialIndexIterator)

	// Contains checks if a given object `obj` with the specified
	// `hashId` exists in the spatial index.
	Contains(obj *Shape, hashId HashValue) bool

	// Insert adds a new object `obj` with the specified `hashId` to
	// the spatial index.
	Insert(obj *Shape, hashId HashValue)

	// Remove deletes the object `obj` with the specified `hashId`
	// from the spatial index, if it exists.
	Remove(obj *Shape, hashId HashValue)

	// Reindex rebuilds the spatial index from the existing objects
	// to optimize query performance.
	Reindex()

	// ReindexObject updates the spatial position of the given object
	// `obj` in the index using the specified `hashId`.
	ReindexObject(obj *Shape, hashId HashValue)

	// ReindexQuery performs a reindexing query using the given
	// spatial query function `f` and additional `data`.
	ReindexQuery(f SpatialIndexQuery, data any)

	// Query allows querying the spatial index for objects that intersect
	// or are contained within the bounding box `bb` using the provided
	// query function `f` and additional `data`.
	Query(obj any, bb BB, f SpatialIndexQuery, data any)

	// SegmentQuery performs a segment-based query using the line segment
	// defined by points `a` and `b`. It allows querying objects affected
	// by the segment based on the `tExit` parameter, applying the
	// provided segment query function `f` and additional `data`.
	SegmentQuery(obj any, a, b v.Vec, tExit float64, f SpatialIndexSegmentQuery, data any)
}

func ShapeGetBB(obj *Shape) BB {
	return obj.BB
}

type SpatialIndex struct {
	class                     SpatialIndexer
	bbfunc                    SpatialIndexBB
	staticIndex, dynamicIndex *SpatialIndex
}

func NewSpatialIndex(klass SpatialIndexer, bbfunc SpatialIndexBB, staticIndex *SpatialIndex) *SpatialIndex {
	index := &SpatialIndex{
		class:       klass,
		bbfunc:      bbfunc,
		staticIndex: staticIndex,
	}

	if staticIndex != nil {
		staticIndex.dynamicIndex = index
	}

	return index
}

func (index *SpatialIndex) GetTree() *BBTree {
	if index == nil {
		return nil
	}
	return index.class.(*BBTree)
}

func (index *SpatialIndex) GetRootIfTree() *Node {
	if index == nil {
		return nil
	}
	return index.class.(*BBTree).root
}

func (dynamicIndex *SpatialIndex) CollideStatic(staticIndex *SpatialIndex, f SpatialIndexQuery, data any) {
	if staticIndex != nil && staticIndex.class.Count() > 0 {
		dynamicIndex.class.Each(func(obj *Shape) {
			staticIndex.class.Query(obj, dynamicIndex.bbfunc(obj), f, data)
		})
	}
}
