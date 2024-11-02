package cm

import (
	"math"

	"github.com/setanarut/vec"
)

type MarkContext struct {
	tree       *BBTree
	staticRoot *Node
	f          SpatialIndexQuery
	data       any
}

type Children struct {
	a, b *Node
}

type Leaf struct {
	stamp uint
	pairs *Pair
}

type Pair struct {
	a, b        Thread
	collisionId uint32
}

type Thread struct {
	prev, next *Pair
	leaf       *Node
}

func (thread *Thread) Unlink() {
	next := thread.next
	prev := thread.prev

	if next != nil {
		if next.a.leaf == thread.leaf {
			next.a.prev = prev
		} else {
			next.b.prev = prev
		}
	}

	if prev != nil {
		if prev.a.leaf == thread.leaf {
			prev.a.next = next
		} else {
			prev.b.next = next
		}
	} else {
		thread.leaf.pairs = next
	}
}

type BBTreeVelocityFunc func(obj any) vec.Vec2

// BBTree represents a bounding box tree used for
// spatial indexing and collision detection.
type BBTree struct {
	// spatialIndex holds a reference to the spatial index used for querying the tree.
	spatialIndex *SpatialIndex
	// velocityFunc is a function that calculates the velocity of bounding boxes for updates.
	velocityFunc BBTreeVelocityFunc
	// leaves is a set that contains the shapes (or nodes) that fall under this tree's spatial index.
	leaves *HashSet[*Shape, *Node]
	// root is the root node of the bounding box tree.
	root *Node
	// pooledNodes is a reusable pool of nodes to optimize memory usage and allocation.
	pooledNodes *Node
	// pooledPairs is a reusable pool for pairs in bounding box checks to avoid frequent allocations.
	pooledPairs *Pair
	// stamp is a timestamp used to manage updates to the tree for efficient processing of dynamic changes.
	stamp uint
}

func NewBBTree(bbfunc SpatialIndexBB, staticIndex *SpatialIndex) *SpatialIndex {
	bbtree := &BBTree{
		leaves: NewHashSet(leafSetEql),
	}
	bbtree.spatialIndex = NewSpatialIndex(bbtree, bbfunc, staticIndex)
	return bbtree.spatialIndex
}

func (bbt *BBTree) Count() int {
	return int(bbt.leaves.Count())
}

func (bbt *BBTree) Each(f SpatialIndexIterator) {
	bbt.leaves.Each(func(node *Node) {
		f(node.obj)
	})
}

func (bbt *BBTree) Contains(obj *Shape, hashId HashValue) bool {
	return bbt.leaves.Find(hashId, obj) != nil
}

func (bbt *BBTree) Insert(obj *Shape, hashId HashValue) {
	leaf := bbt.leaves.Insert(hashId, obj, func(obj *Shape) *Node {
		return bbt.NewLeaf(obj)
	})

	root := bbt.root
	bbt.root = bbt.SubtreeInsert(root, leaf)

	leaf.stamp = bbt.GetMasterTree().stamp
	bbt.LeafAddPairs(leaf)
	bbt.IncrementStamp()
}

func (tree *BBTree) IncrementStamp() {
	dynamicTree := tree.spatialIndex.dynamicIndex.GetTree()
	if dynamicTree != nil {
		dynamicTree.stamp++
	} else {
		tree.stamp++
	}
}

func (tree *BBTree) LeafAddPairs(leaf *Node) {
	dynamicIndex := tree.spatialIndex.dynamicIndex
	if dynamicIndex != nil {
		dynamicRoot := dynamicIndex.GetRootIfTree()
		if dynamicRoot != nil {
			dynamicTree := dynamicIndex.GetTree()
			context := &MarkContext{dynamicTree, nil, nil, nil}
			dynamicRoot.MarkLeafQuery(leaf, true, context)
		}
	} else {
		staticRoot := tree.spatialIndex.staticIndex.GetRootIfTree()
		context := &MarkContext{tree, staticRoot, VoidQueryFunc, nil}
		leaf.MarkLeaf(context)
	}
}

func (tree *BBTree) PairInsert(a *Node, b *Node) {
	nextA := a.pairs
	nextB := b.pairs
	pair := tree.PairFromPool()
	pair.a = Thread{prev: nil, next: nextA, leaf: a}
	pair.b = Thread{prev: nil, next: nextB, leaf: b}
	pair.collisionId = 0

	a.pairs = pair
	b.pairs = pair

	if nextA != nil {
		if nextA.a.leaf == a {
			nextA.a.prev = pair
		} else {
			nextA.b.prev = pair
		}
	}

	if nextB != nil {
		if nextB.a.leaf == b {
			nextB.a.prev = pair
		} else {
			nextB.b.prev = pair
		}
	}
}

func (bbt *BBTree) PairFromPool() *Pair {
	tree := bbt.GetMasterTree()

	pair := tree.pooledPairs

	if pair != nil {
		tree.pooledPairs = pair.a.next
		return pair
	}

	// Pool is exhausted make more
	for i := 0; i < pooledBufferSize; i++ {
		tree.RecyclePair(&Pair{})
	}

	return &Pair{}
}

func (bbt *BBTree) RecyclePair(pair *Pair) {
	master := bbt.GetMasterTree()
	pair.a.next = master.pooledPairs
	master.pooledPairs = pair
}

func (bbt *BBTree) SubtreeInsert(subtree *Node, leaf *Node) *Node {
	if subtree == nil {
		return leaf
	}
	if subtree.IsLeaf() {
		return bbt.NewNode(leaf, subtree)
	}

	costA := subtree.b.bb.Area() + subtree.a.bb.MergedArea(leaf.bb)
	costB := subtree.a.bb.Area() + subtree.b.bb.MergedArea(leaf.bb)

	if costA == costB {
		costA = subtree.a.bb.Proximity(leaf.bb)
		costB = subtree.b.bb.Proximity(leaf.bb)
	}

	if costB < costA {
		NodeSetB(subtree, bbt.SubtreeInsert(subtree.b, leaf))
	} else {
		NodeSetA(subtree, bbt.SubtreeInsert(subtree.a, leaf))
	}

	subtree.bb = subtree.bb.Merge(leaf.bb)
	return subtree
}

func (tree *BBTree) SubtreeRemove(subtree *Node, leaf *Node) *Node {
	if leaf == subtree {
		return nil
	}

	parent := leaf.parent
	if parent == subtree {
		other := subtree.Other(leaf)
		other.parent = subtree.parent
		tree.RecycleNode(subtree)
		return other
	}

	tree.ReplaceChild(parent.parent, parent, parent.Other(leaf))
	return subtree
}

func (tree *BBTree) ReplaceChild(parent, child, value *Node) {
	if parent.a == child {
		tree.RecycleNode(parent.a)
		NodeSetA(parent, value)
	} else {
		tree.RecycleNode(parent.b)
		NodeSetB(parent, value)
	}

	for node := parent; node != nil; node = node.parent {
		node.bb = node.a.bb.Merge(node.b.bb)
	}
}

func (tree *BBTree) Remove(obj *Shape, hashId HashValue) {
	leaf := tree.leaves.Remove(hashId, obj)

	tree.root = tree.SubtreeRemove(tree.root, leaf)
	tree.PairsClear(leaf)
	tree.RecycleNode(leaf)
}

func (tree *BBTree) Reindex() {
	panic("implement me")
}

func (bbt *BBTree) ReindexObject(obj *Shape, hashId HashValue) {
	leaf := bbt.leaves.Find(hashId, obj)
	if leaf != nil {
		if bbt.LeafUpdate(leaf) {
			bbt.LeafAddPairs(leaf)
		}
		bbt.IncrementStamp()
	}
}

func (tree *BBTree) ReindexQuery(f SpatialIndexQuery, data any) {
	if tree.root == nil {
		return
	}

	// LeafUpdate() may modify tree->root. Don't cache it.
	tree.leaves.Each(func(leaf *Node) {
		tree.LeafUpdate(leaf)
	})

	staticIndex := tree.spatialIndex.staticIndex
	var staticRoot *Node
	if staticIndex != nil {
		staticRoot = staticIndex.class.(*BBTree).root
	}

	context := &MarkContext{tree, staticRoot, f, data}
	tree.root.MarkSubtree(context)

	if staticIndex != nil && staticRoot == nil {
		tree.spatialIndex.CollideStatic(staticIndex, f, data)
	}

	tree.IncrementStamp()
}

func (tree *BBTree) LeafUpdate(leaf *Node) bool {
	root := tree.root
	bb := tree.spatialIndex.bbfunc(leaf.obj)

	if !leaf.bb.Contains(bb) {
		leaf.bb = tree.GetBB(leaf.obj)

		root = tree.SubtreeRemove(root, leaf)
		tree.root = tree.SubtreeInsert(root, leaf)

		tree.PairsClear(leaf)
		leaf.stamp = tree.GetMasterTree().stamp
		return true
	}

	return false
}
func (tree *BBTree) PairsClear(leaf *Node) {
	pair := leaf.pairs
	leaf.pairs = nil

	for pair != nil {
		if pair.a.leaf == leaf {
			next := pair.a.next
			pair.b.Unlink()
			tree.RecyclePair(pair)
			pair = next
		} else {
			next := pair.b.next
			pair.a.Unlink()
			tree.RecyclePair(pair)
			pair = next
		}
	}
}

func (bbt *BBTree) Query(obj any, bb BB, f SpatialIndexQuery, data any) {
	if bbt.root != nil {
		bbt.root.SubtreeQuery(obj, bb, f, data)
	}
}

func (bbt *BBTree) SegmentQuery(obj any, a, b vec.Vec2, tExit float64, f SpatialIndexSegmentQuery, data any) {
	root := bbt.root
	if root != nil {
		root.SubtreeSegmentQuery(obj, a, b, tExit, f, data)
	}
}

func (bbt *BBTree) GetBB(obj *Shape) BB {
	bb := bbt.spatialIndex.bbfunc(obj)
	if bbt.velocityFunc != nil {
		coef := 0.1
		x := (bb.R - bb.L) * coef
		y := (bb.T - bb.B) * coef

		v := bbt.velocityFunc(obj).Scale(0.1)
		return BB{
			bb.L + math.Min(-x, v.X),
			bb.B + math.Min(-y, v.Y),
			bb.R + math.Max(x, v.X),
			bb.T + math.Max(y, v.Y),
		}
	}

	return bb
}

func (bbt *BBTree) NewNode(a, b *Node) *Node {
	node := bbt.NodeFromPool()
	node.obj = nil
	node.bb = a.bb.Merge(b.bb)
	node.parent = nil

	NodeSetA(node, a)
	NodeSetB(node, b)
	return node
}

func (tree *BBTree) NewLeaf(obj *Shape) *Node {
	node := tree.NodeFromPool()
	node.obj = obj
	node.bb = tree.GetBB(obj)
	node.parent = nil
	node.stamp = 0
	node.pairs = nil

	return node
}

func (tree *BBTree) NodeFromPool() *Node {
	node := tree.pooledNodes

	if node != nil {
		tree.pooledNodes = node.parent
		return node
	}

	// Pool is exhausted make more
	for i := 0; i < pooledBufferSize; i++ {
		tree.RecycleNode(&Node{})
	}

	return &Node{
		parent: tree.pooledNodes,
	}
}

func (tree *BBTree) RecycleNode(node *Node) {
	node.parent = tree.pooledNodes
	tree.pooledNodes = node
}

func (tree *BBTree) GetMasterTree() *BBTree {
	dynamicTree := tree.spatialIndex.dynamicIndex.GetTree()
	if dynamicTree != nil {
		return dynamicTree
	}
	return tree
}

type Node struct {
	obj    *Shape
	bb     BB
	parent *Node

	Children
	Leaf
}

func NodeSetA(node, value *Node) {
	node.a = value
	value.parent = node
}

func NodeSetB(node, value *Node) {
	node.b = value
	value.parent = node
}

func (leaf *Node) MarkLeaf(context *MarkContext) {
	tree := context.tree
	if leaf.stamp == tree.GetMasterTree().stamp {
		staticRoot := context.staticRoot
		if staticRoot != nil {
			staticRoot.MarkLeafQuery(leaf, false, context)
		}

		for node := leaf; node.parent != nil; node = node.parent {
			if node == node.parent.a {
				node.parent.b.MarkLeafQuery(leaf, true, context)
			} else {
				node.parent.a.MarkLeafQuery(leaf, false, context)
			}
		}
	} else {
		pair := leaf.pairs
		for pair != nil {
			if leaf == pair.b.leaf {
				pair.collisionId = context.f(pair.a.leaf.obj, leaf.obj, pair.collisionId, context.data)
				pair = pair.b.next
			} else {
				pair = pair.a.next
			}
		}
	}
}

func (subtree *Node) MarkLeafQuery(leaf *Node, left bool, context *MarkContext) {
	if leaf.bb.Intersects(subtree.bb) {
		if subtree.IsLeaf() {
			if left {
				context.tree.PairInsert(leaf, subtree)
			} else {
				if subtree.stamp < leaf.stamp {
					context.tree.PairInsert(subtree, leaf)
				}
				context.f(leaf.obj, subtree.obj, 0, context.data)
			}
		} else {
			subtree.a.MarkLeafQuery(leaf, left, context)
			subtree.b.MarkLeafQuery(leaf, left, context)
		}
	}
}

func (node *Node) Other(child *Node) *Node {
	if node.a == child {
		return node.b
	}
	return node.a
}

func (node *Node) IsLeaf() bool {
	return node.obj != nil
}

func (subtree *Node) SubtreeQuery(obj any, bb BB, query SpatialIndexQuery, data any) {
	if subtree.bb.Intersects(bb) {
		if subtree.IsLeaf() {
			query(obj, subtree.obj, 0, data)
		} else {
			subtree.a.SubtreeQuery(obj, bb, query, data)
			subtree.b.SubtreeQuery(obj, bb, query, data)
		}
	}
}
func (subtree *Node) MarkSubtree(context *MarkContext) {
	if subtree.IsLeaf() {
		subtree.MarkLeaf(context)
	} else {
		subtree.a.MarkSubtree(context)
		subtree.b.MarkSubtree(context)
	}
}

func (subtree *Node) SubtreeSegmentQuery(obj any, a, b vec.Vec2, tExit float64, f SpatialIndexSegmentQuery, data any) float64 {
	if subtree.IsLeaf() {
		return f(obj, subtree.obj, data)
	}

	tA := subtree.a.bb.SegmentQuery(a, b)
	tB := subtree.b.bb.SegmentQuery(a, b)

	if tA < tB {
		if tA < tExit {
			tExit = math.Min(tExit, subtree.a.SubtreeSegmentQuery(obj, a, b, tExit, f, data))
		}
		if tB < tExit {
			tExit = math.Min(tExit, subtree.b.SubtreeSegmentQuery(obj, a, b, tExit, f, data))
		}
	} else {
		if tB < tExit {
			tExit = math.Min(tExit, subtree.b.SubtreeSegmentQuery(obj, a, b, tExit, f, data))
		}
		if tA < tExit {
			tExit = math.Min(tExit, subtree.a.SubtreeSegmentQuery(obj, a, b, tExit, f, data))
		}
	}

	return tExit
}

func VoidQueryFunc(obj1 any, obj2 *Shape, collisionId uint32, data any) uint32 {
	return collisionId
}

func leafSetEql(obj *Shape, node *Node) bool {
	return obj == node.obj
}
