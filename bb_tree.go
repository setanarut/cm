package cm

import (
	"github.com/setanarut/v"
)

type markContext struct {
	tree       *bBTree
	staticRoot *node
	f          SpatialIndexQuery
	data       any
}

type children struct {
	a, b *node
}

type leaf struct {
	stamp uint
	pairs *pair
}

type pair struct {
	a, b        thread
	collisionId uint32
}

type thread struct {
	prev, next *pair
	leaf       *node
}

func (thread *thread) Unlink() {
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

type bBTreeVelocityFunc func(obj any) v.Vec

// bBTree represents a bounding box tree used for
// spatial indexing and collision detection.
type bBTree struct {
	// spatialIndex holds a reference to the spatial index used for querying the tree.
	spatialIndex *spatialIndex
	// velocityFunc is a function that calculates the velocity of bounding boxes for updates.
	velocityFunc bBTreeVelocityFunc
	// leaves is a set that contains the shapes (or nodes) that fall under this tree's spatial index.
	leaves *hashSet[*Shape, *node]
	// root is the root node of the bounding box tree.
	root *node
	// pooledNodes is a reusable pool of nodes to optimize memory usage and allocation.
	pooledNodes *node
	// pooledPairs is a reusable pool for pairs in bounding box checks to avoid frequent allocations.
	pooledPairs *pair
	// stamp is a timestamp used to manage updates to the tree for efficient processing of dynamic changes.
	stamp uint
}

func newBBTree(bbfunc SpatialIndexBB, staticIndex *spatialIndex) *spatialIndex {
	bbtree := &bBTree{
		leaves: newHashSet(leafSetEql),
	}
	bbtree.spatialIndex = NewSpatialIndex(bbtree, bbfunc, staticIndex)
	return bbtree.spatialIndex
}

func (btr *bBTree) Count() int {
	return int(btr.leaves.Count())
}

func (btr *bBTree) Each(f SpatialIndexIterator) {
	btr.leaves.Each(func(n *node) {
		f(n.obj)
	})
}

func (btr *bBTree) Contains(obj *Shape, hashId HashValue) bool {
	return btr.leaves.Find(hashId, obj) != nil
}

func (btr *bBTree) Insert(obj *Shape, hashId HashValue) {
	leaf := btr.leaves.Insert(hashId, obj, func(obj *Shape) *node {
		return btr.NewLeaf(obj)
	})

	root := btr.root
	btr.root = btr.SubtreeInsert(root, leaf)

	leaf.stamp = btr.GetMasterTree().stamp
	btr.LeafAddPairs(leaf)
	btr.IncrementStamp()
}

func (btr *bBTree) IncrementStamp() {
	dynamicTree := btr.spatialIndex.dynamicIndex.GetTree()
	if dynamicTree != nil {
		dynamicTree.stamp++
	} else {
		btr.stamp++
	}
}

func (btr *bBTree) LeafAddPairs(leaf *node) {
	dynamicIndex := btr.spatialIndex.dynamicIndex
	if dynamicIndex != nil {
		dynamicRoot := dynamicIndex.GetRootIfTree()
		if dynamicRoot != nil {
			dynamicTree := dynamicIndex.GetTree()
			context := &markContext{dynamicTree, nil, nil, nil}
			dynamicRoot.MarkLeafQuery(leaf, true, context)
		}
	} else {
		staticRoot := btr.spatialIndex.staticIndex.GetRootIfTree()
		context := &markContext{btr, staticRoot, VoidQueryFunc, nil}
		leaf.markLeaf(context)
	}
}

func (btr *bBTree) PairInsert(a *node, b *node) {
	nextA := a.pairs
	nextB := b.pairs
	pr := btr.PairFromPool()
	pr.a = thread{prev: nil, next: nextA, leaf: a}
	pr.b = thread{prev: nil, next: nextB, leaf: b}
	pr.collisionId = 0

	a.pairs = pr
	b.pairs = pr

	if nextA != nil {
		if nextA.a.leaf == a {
			nextA.a.prev = pr
		} else {
			nextA.b.prev = pr
		}
	}

	if nextB != nil {
		if nextB.a.leaf == b {
			nextB.a.prev = pr
		} else {
			nextB.b.prev = pr
		}
	}
}

func (btr *bBTree) PairFromPool() *pair {
	tree := btr.GetMasterTree()

	pr := tree.pooledPairs

	if pr != nil {
		tree.pooledPairs = pr.a.next
		return pr
	}

	// Pool is exhausted make more
	for range pooledBufferSize {
		tree.RecyclePair(&pair{})
	}

	return &pair{}
}

func (btr *bBTree) RecyclePair(p *pair) {
	master := btr.GetMasterTree()
	p.a.next = master.pooledPairs
	master.pooledPairs = p
}

func (btr *bBTree) SubtreeInsert(subtree *node, leaf *node) *node {
	if subtree == nil {
		return leaf
	}
	if subtree.IsLeaf() {
		return btr.NewNode(leaf, subtree)
	}

	costA := subtree.b.bb.Area() + subtree.a.bb.MergedArea(leaf.bb)
	costB := subtree.a.bb.Area() + subtree.b.bb.MergedArea(leaf.bb)

	if costA == costB {
		costA = subtree.a.bb.Proximity(leaf.bb)
		costB = subtree.b.bb.Proximity(leaf.bb)
	}

	if costB < costA {
		nodeSetB(subtree, btr.SubtreeInsert(subtree.b, leaf))
	} else {
		nodeSetA(subtree, btr.SubtreeInsert(subtree.a, leaf))
	}

	subtree.bb = subtree.bb.Merge(leaf.bb)
	return subtree
}

func (btr *bBTree) SubtreeRemove(subtree *node, leaf *node) *node {
	if leaf == subtree {
		return nil
	}

	parent := leaf.parent
	if parent == subtree {
		other := subtree.Other(leaf)
		other.parent = subtree.parent
		btr.RecycleNode(subtree)
		return other
	}

	btr.ReplaceChild(parent.parent, parent, parent.Other(leaf))
	return subtree
}

func (btr *bBTree) ReplaceChild(parent, child, value *node) {
	if parent.a == child {
		btr.RecycleNode(parent.a)
		nodeSetA(parent, value)
	} else {
		btr.RecycleNode(parent.b)
		nodeSetB(parent, value)
	}

	for nd := parent; nd != nil; nd = nd.parent {
		nd.bb = nd.a.bb.Merge(nd.b.bb)
	}
}

func (btr *bBTree) Remove(obj *Shape, hashId HashValue) {
	leaf := btr.leaves.Remove(hashId, obj)

	btr.root = btr.SubtreeRemove(btr.root, leaf)
	btr.PairsClear(leaf)
	btr.RecycleNode(leaf)
}

func (btr *bBTree) Reindex() {
	panic("implement me")
}

func (btr *bBTree) ReindexObject(obj *Shape, hashId HashValue) {
	leaf := btr.leaves.Find(hashId, obj)
	if leaf != nil {
		if btr.LeafUpdate(leaf) {
			btr.LeafAddPairs(leaf)
		}
		btr.IncrementStamp()
	}
}

func (btr *bBTree) ReindexQuery(f SpatialIndexQuery, data any) {
	if btr.root == nil {
		return
	}

	// LeafUpdate() may modify tree->root. Don't cache it.
	btr.leaves.Each(func(leaf *node) {
		btr.LeafUpdate(leaf)
	})

	staticIndex := btr.spatialIndex.staticIndex
	var staticRoot *node
	if staticIndex != nil {
		staticRoot = staticIndex.class.(*bBTree).root
	}

	context := &markContext{btr, staticRoot, f, data}
	btr.root.MarkSubtree(context)

	if staticIndex != nil && staticRoot == nil {
		btr.spatialIndex.CollideStatic(staticIndex, f, data)
	}

	btr.IncrementStamp()
}

func (btr *bBTree) LeafUpdate(leaf *node) bool {
	root := btr.root
	bb := btr.spatialIndex.bbfunc(leaf.obj)

	if !leaf.bb.Contains(bb) {
		leaf.bb = btr.GetBB(leaf.obj)

		root = btr.SubtreeRemove(root, leaf)
		btr.root = btr.SubtreeInsert(root, leaf)

		btr.PairsClear(leaf)
		leaf.stamp = btr.GetMasterTree().stamp
		return true
	}

	return false
}
func (btr *bBTree) PairsClear(leaf *node) {
	pr := leaf.pairs
	leaf.pairs = nil

	for pr != nil {
		if pr.a.leaf == leaf {
			next := pr.a.next
			pr.b.Unlink()
			btr.RecyclePair(pr)
			pr = next
		} else {
			next := pr.b.next
			pr.a.Unlink()
			btr.RecyclePair(pr)
			pr = next
		}
	}
}

func (btr *bBTree) Query(obj any, bb BB, f SpatialIndexQuery, data any) {
	if btr.root != nil {
		btr.root.SubtreeQuery(obj, bb, f, data)
	}
}

func (btr *bBTree) SegmentQuery(obj any, a, b v.Vec, tExit float64, f SpatialIndexSegmentQuery, data any) {
	root := btr.root
	if root != nil {
		root.SubtreeSegmentQuery(obj, a, b, tExit, f, data)
	}
}

func (btr *bBTree) GetBB(obj *Shape) BB {
	bb := btr.spatialIndex.bbfunc(obj)
	if btr.velocityFunc != nil {
		coef := 0.1
		x := (bb.R - bb.L) * coef
		y := (bb.T - bb.B) * coef

		v := btr.velocityFunc(obj).Scale(0.1)
		return BB{
			bb.L + min(-x, v.X),
			bb.B + min(-y, v.Y),
			bb.R + max(x, v.X),
			bb.T + max(y, v.Y),
		}
	}

	return bb
}

func (btr *bBTree) NewNode(a, b *node) *node {
	nd := btr.NodeFromPool()
	nd.obj = nil
	nd.bb = a.bb.Merge(b.bb)
	nd.parent = nil

	nodeSetA(nd, a)
	nodeSetB(nd, b)
	return nd
}

func (btr *bBTree) NewLeaf(obj *Shape) *node {
	nd := btr.NodeFromPool()
	nd.obj = obj
	nd.bb = btr.GetBB(obj)
	nd.parent = nil
	nd.stamp = 0
	nd.pairs = nil

	return nd
}

func (btr *bBTree) NodeFromPool() *node {
	nd := btr.pooledNodes

	if nd != nil {
		btr.pooledNodes = nd.parent
		return nd
	}

	// Pool is exhausted make more
	for range pooledBufferSize {
		btr.RecycleNode(&node{})
	}

	return &node{
		parent: btr.pooledNodes,
	}
}

func (btr *bBTree) RecycleNode(nd *node) {
	nd.parent = btr.pooledNodes
	btr.pooledNodes = nd
}

func (btr *bBTree) GetMasterTree() *bBTree {
	dynamicTree := btr.spatialIndex.dynamicIndex.GetTree()
	if dynamicTree != nil {
		return dynamicTree
	}
	return btr
}

type node struct {
	obj    *Shape
	bb     BB
	parent *node

	children
	leaf
}

func nodeSetA(nd, value *node) {
	nd.a = value
	value.parent = nd
}

func nodeSetB(n, value *node) {
	n.b = value
	value.parent = n
}

func (n *node) markLeaf(context *markContext) {
	tree := context.tree
	if n.stamp == tree.GetMasterTree().stamp {
		staticRoot := context.staticRoot
		if staticRoot != nil {
			staticRoot.MarkLeafQuery(n, false, context)
		}

		for nd := n; nd.parent != nil; nd = nd.parent {
			if nd == nd.parent.a {
				nd.parent.b.MarkLeafQuery(n, true, context)
			} else {
				nd.parent.a.MarkLeafQuery(n, false, context)
			}
		}
	} else {
		pr := n.pairs
		for pr != nil {
			if n == pr.b.leaf {
				pr.collisionId = context.f(pr.a.leaf.obj, n.obj, pr.collisionId, context.data)
				pr = pr.b.next
			} else {
				pr = pr.a.next
			}
		}
	}
}

func (n *node) MarkLeafQuery(leaf *node, left bool, context *markContext) {
	if leaf.bb.Intersects(n.bb) {
		if n.IsLeaf() {
			if left {
				context.tree.PairInsert(leaf, n)
			} else {
				if n.stamp < leaf.stamp {
					context.tree.PairInsert(n, leaf)
				}
				context.f(leaf.obj, n.obj, 0, context.data)
			}
		} else {
			n.a.MarkLeafQuery(leaf, left, context)
			n.b.MarkLeafQuery(leaf, left, context)
		}
	}
}

func (n *node) Other(child *node) *node {
	if n.a == child {
		return n.b
	}
	return n.a
}

func (n *node) IsLeaf() bool {
	return n.obj != nil
}

func (n *node) SubtreeQuery(obj any, bb BB, query SpatialIndexQuery, data any) {
	if n.bb.Intersects(bb) {
		if n.IsLeaf() {
			query(obj, n.obj, 0, data)
		} else {
			n.a.SubtreeQuery(obj, bb, query, data)
			n.b.SubtreeQuery(obj, bb, query, data)
		}
	}
}
func (n *node) MarkSubtree(context *markContext) {
	if n.IsLeaf() {
		n.markLeaf(context)
	} else {
		n.a.MarkSubtree(context)
		n.b.MarkSubtree(context)
	}
}

func (n *node) SubtreeSegmentQuery(obj any, a, b v.Vec, tExit float64, f SpatialIndexSegmentQuery, data any) float64 {
	if n.IsLeaf() {
		return f(obj, n.obj, data)
	}

	tA := n.a.bb.SegmentQuery(a, b)
	tB := n.b.bb.SegmentQuery(a, b)

	if tA < tB {
		if tA < tExit {
			tExit = min(tExit, n.a.SubtreeSegmentQuery(obj, a, b, tExit, f, data))
		}
		if tB < tExit {
			tExit = min(tExit, n.b.SubtreeSegmentQuery(obj, a, b, tExit, f, data))
		}
	} else {
		if tB < tExit {
			tExit = min(tExit, n.b.SubtreeSegmentQuery(obj, a, b, tExit, f, data))
		}
		if tA < tExit {
			tExit = min(tExit, n.a.SubtreeSegmentQuery(obj, a, b, tExit, f, data))
		}
	}

	return tExit
}

func VoidQueryFunc(obj1 any, obj2 *Shape, collisionId uint32, data any) uint32 {
	return collisionId
}

func leafSetEql(obj *Shape, n *node) bool {
	return obj == n.obj
}
