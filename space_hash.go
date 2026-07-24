package cm

import (
	"math"
	"sync"

	"github.com/setanarut/v"
)

type spaceHash struct {
	*SpatialIndex

	numCells int
	celldim  float64

	table     []*spaceHashBin
	handleSet *hashSet[*Shape, *handle]

	pooledBins    *spaceHashBin
	pooledHandles sync.Pool

	stamp uint
}

func newSpaceHash(celldim float64, num int, bbfunc SpatialIndexBB, staticIndex *SpatialIndex) *SpatialIndex {
	spaceHash := &spaceHash{
		celldim:  celldim,
		numCells: num,
		table:    make([]*spaceHashBin, num),
		handleSet: NewHashSet(func(obj *Shape, elt *handle) bool {
			return obj == elt.obj
		}),
		stamp:         1,
		pooledHandles: sync.Pool{New: func() any { return &handle{} }},
	}
	for range pooledBufferSize {
		spaceHash.pooledHandles.Put(&handle{})
	}
	spatialIndex := NewSpatialIndex(spaceHash, bbfunc, staticIndex)
	spaceHash.SpatialIndex = spatialIndex
	return spatialIndex
}

func (hash *spaceHash) hashHandle(hand *handle, bb BB) {
	dim := hash.celldim

	// TODO: chipmunk said floor is slow, use custom floor
	l := floor(bb.L / dim)
	r := floor(bb.R / dim)
	b := floor(bb.B / dim)
	t := floor(bb.T / dim)

	n := hash.numCells
	for i := l; i <= r; i++ {
		for j := b; j <= t; j++ {
			idx := hashFunc(HashValue(i), HashValue(j), HashValue(n))
			bin := hash.table[idx]

			if bin.containsHandle(hand) {
				continue
			}

			hand.retain()
			newBin := hash.getEmptyBin()
			newBin.handle = hand
			newBin.next = bin
			hash.table[idx] = newBin
		}
	}
}

func (hash *spaceHash) Count() int {
	return int(hash.handleSet.Count())
}

func (hash *spaceHash) Each(f SpatialIndexIterator) {
	hash.handleSet.Each(func(elt *handle) {
		f(elt.obj)
	})
}

func (hash *spaceHash) Contains(obj *Shape, hashId HashValue) bool {
	return hash.handleSet.Find(hashId, obj) != nil
}

func (hash *spaceHash) Insert(obj *Shape, hashId HashValue) {
	hand := hash.handleSet.Insert(hashId, obj, func(obj *Shape) *handle {
		hand := hash.pooledHandles.Get().(*handle)
		hand.init(obj)
		hand.retain()
		return hand
	})
	hash.hashHandle(hand, hash.bbfunc(obj))
}

func (hash *spaceHash) Remove(obj *Shape, hashId HashValue) {
	hand := hash.handleSet.Remove(hashId, obj)

	if hand != nil {
		hand.obj = nil
		hand.release(&hash.pooledHandles)
	}
}

func (hash *spaceHash) Reindex() {
	hash.clearTable()
	hash.handleSet.Each(func(hand *handle) {
		hash.bbfunc(hand.obj)
	})
}

func (hash *spaceHash) ReindexObject(obj *Shape, hashId HashValue) {
	hand := hash.handleSet.Remove(hashId, obj)

	if hand != nil {
		hand.obj = nil
		hand.release(&hash.pooledHandles)

		hash.Insert(obj, hashId)
	}
}

func (hash *spaceHash) removeOrphanedHandles(binPtr **spaceHashBin) {
	bin := *binPtr
	for bin != nil {
		hand := bin.handle
		next := bin.next

		if hand.obj == nil {
			// orphaned handle
			*binPtr = bin.next
			hash.recycleBin(bin)

			hand.release(&hash.pooledHandles)
		} else {
			binPtr = &bin.next
		}

		bin = next
	}
}

func (hash *spaceHash) queryHelper(binPtr **spaceHashBin, obj any, f SpatialIndexQuery, data any) {
restart:
	for bin := *binPtr; bin != nil; bin = bin.next {
		hand := bin.handle
		other := hand.obj

		if hand.stamp == hash.stamp || obj == other {
			continue
		} else if other != nil {
			f(obj, other, 0, data)
			hand.stamp = hash.stamp
		} else {
			hash.removeOrphanedHandles(binPtr)
			goto restart
		}
	}
}

func floor(f float64) int {
	i := int(f)
	if f < 0 && float64(i) != f {
		return i - 1
	}
	return i
}

func (hash *spaceHash) ReindexQuery(f SpatialIndexQuery, data any) {
	hash.clearTable()

	hash.handleSet.Each(func(hand *handle) {
		// queryRehashHelper

		bb := hash.SpatialIndex.bbfunc(hand.obj)

		l := floor(bb.L / hash.celldim)
		r := floor(bb.R / hash.celldim)
		b := floor(bb.B / hash.celldim)
		t := floor(bb.T / hash.celldim)

		for i := l; i <= r; i++ {
			for j := b; j <= t; j++ {
				idx := hashFunc(HashValue(i), HashValue(j), HashValue(hash.numCells))
				bin := hash.table[idx]

				if bin.containsHandle(hand) {
					continue
				}

				hand.retain()
				hash.queryHelper(&bin, hand.obj, f, data)

				newBin := hash.getEmptyBin()
				newBin.handle = hand
				newBin.next = bin
				hash.table[idx] = newBin
			}
		}

		hash.stamp++
	})

	hash.CollideStatic(hash.staticIndex, f, data)
}

func (hash *spaceHash) Query(obj any, bb BB, f SpatialIndexQuery, data any) {
	dim := hash.celldim
	l := floor(bb.L / dim)
	r := floor(bb.R / dim)
	b := floor(bb.B / dim)
	t := floor(bb.T / dim)

	n := hash.numCells

	for i := l; i <= r; i++ {
		for j := b; j <= t; j++ {
			hash.queryHelper(&hash.table[hashFunc(HashValue(i), HashValue(j), HashValue(n))], obj, f, data)
		}
	}

	hash.stamp++
}

func (hash *spaceHash) segmentQueryHelper(binPtr **spaceHashBin, obj any, f SpatialIndexSegmentQuery, data any) float64 {
	t := 1.0

restart:
	for bin := *binPtr; bin != nil; bin = bin.next {
		hand := bin.handle
		other := hand.obj

		if hand.stamp == hash.stamp {
			continue
		} else if other != nil {
			t = min(t, f(obj, other, data))
			hand.stamp = hash.stamp
		} else {
			hash.removeOrphanedHandles(binPtr)
			goto restart
		}
	}

	return t
}

// modified from http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
func (hash *spaceHash) SegmentQuery(obj any, a, b v.Vec, tExit float64, f SpatialIndexSegmentQuery, data any) {
	a = a.Scale(1.0 / hash.celldim)
	b = b.Scale(1.0 / hash.celldim)

	cellX := int(floor(a.X))
	cellY := int(floor(a.Y))

	t := 0.0

	var xInc, yInc int
	var tempV, tempH float64

	if b.X > a.X {
		xInc = 1
		tempH = math.Floor(a.X+1.0) - a.X
	} else {
		xInc = -1
		tempH = a.X - math.Floor(a.X)
	}

	if b.Y > a.Y {
		yInc = 1
		tempV = math.Floor(a.Y+1.0) - a.Y
	} else {
		yInc = -1
		tempV = a.Y - math.Floor(a.Y)
	}

	dx := math.Abs(b.X - a.X)
	dy := math.Abs(b.Y - a.Y)
	var dtdx, dtdy float64
	if dx != 0 {
		dtdx = 1.0 / dx
	} else {
		dtdx = infinity
	}

	if dy != 0 {
		dtdy = 1.0 / dy
	} else {
		dtdy = infinity
	}

	var nextH, nextV float64
	if tempH != 0 {
		nextH = tempH * dtdx
	} else {
		nextH = dtdx
	}
	if tempV != 0 {
		nextV = tempV * dtdy
	} else {
		nextV = dtdy
	}

	for t < tExit {
		idx := hashFunc(HashValue(cellX), HashValue(cellY), HashValue(hash.numCells))
		tExit = min(tExit, hash.segmentQueryHelper(&hash.table[idx], obj, f, data))

		if nextV < nextH {
			cellY += yInc
			t = nextV
			nextV += dtdy
		} else {
			cellX += xInc
			t = nextH
			nextH += dtdx
		}
	}

	hash.stamp++
}

type spaceHashBin struct {
	handle *handle
	next   *spaceHashBin
}

func (bin *spaceHashBin) containsHandle(hand *handle) bool {
	for item := bin; item != nil; item = item.next {
		if item.handle == hand {
			return true
		}
	}

	return false
}

func hashFunc(x, y, n HashValue) HashValue {
	return (x*1640531513 ^ y*2654435789) % n
}

type handle struct {
	obj     *Shape
	retains int
	stamp   uint
}

func (hand *handle) init(obj *Shape) {
	hand.obj = obj
	hand.retains = 0
	hand.stamp = 0
}

func (hand *handle) retain() {
	hand.retains++
}

func (hand *handle) release(pooledHandles *sync.Pool) {
	hand.retains--
	if hand.retains == 0 {
		pooledHandles.Put(hand)
	}
}

func (hash *spaceHash) recycleBin(bin *spaceHashBin) {
	bin.next = hash.pooledBins
	hash.pooledBins = bin
}

func (hash *spaceHash) clearTableCell(idx int) {
	bin := hash.table[idx]
	for bin != nil {
		next := bin.next

		bin.handle.release(&hash.pooledHandles)
		hash.recycleBin(bin)

		bin = next
	}

	hash.table[idx] = nil
}

func (hash *spaceHash) clearTable() {
	for i := 0; i < hash.numCells; i++ {
		hash.clearTableCell(i)
	}
}

func (hash *spaceHash) getEmptyBin() *spaceHashBin {
	bin := hash.pooledBins

	if bin != nil {
		hash.pooledBins = bin.next
		return bin
	}

	// pool is exhausted, make more
	for range pooledBufferSize {
		hash.recycleBin(&spaceHashBin{})
	}
	return &spaceHashBin{}
}
