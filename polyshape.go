package cm

import (
	"math"

	"github.com/setanarut/v"
)

type PolyShape struct {
	*Shape
	Radius float64
	count  int
	// The untransformed Planes are appended at the end of the transformed Planes.
	Planes []SplittingPlane
}

func (ps *PolyShape) CacheData(transform Transform) BB {
	count := ps.count
	dst := ps.Planes[0:count]
	src := ps.Planes[count:]

	l := infinity
	r := -infinity
	b := infinity
	t := -infinity

	for i := range count {
		v := transform.Apply(src[i].V0)
		n := transform.ApplyVector(src[i].N)

		dst[i].V0 = v
		dst[i].N = n

		l = math.Min(l, v.X)
		r = math.Max(r, v.X)
		b = math.Min(b, v.Y)
		t = math.Max(t, v.Y)
	}

	radius := ps.Radius
	ps.Shape.BB = BB{l - radius, b - radius, r + radius, t + radius}
	return ps.Shape.BB
}

func (ps *PolyShape) PointQuery(p v.Vec, info *PointQueryInfo) {
	count := ps.count
	planes := ps.Planes
	r := ps.Radius

	v0 := planes[count-1].V0
	minDist := infinity
	closestPoint := v.Vec{}
	closestNormal := v.Vec{}
	outside := false

	for i := range count {
		v1 := planes[i].V0
		if !outside {
			outside = planes[i].N.Dot(p.Sub(v1)) > 0
		}

		closest := closestPointOnSegment(p, v0, v1)

		dist := p.Dist(closest)
		if dist < minDist {
			minDist = dist
			closestPoint = closest
			closestNormal = planes[i].N
		}

		v0 = v1
	}

	var dist float64
	if outside {
		dist = minDist
	} else {
		dist = -minDist
	}
	g := p.Sub(closestPoint).Scale(1.0 / dist)

	info.Shape = ps.Shape
	info.Point = closestPoint.Add(g.Scale(r))
	info.Distance = dist - r

	if minDist > magicEpsilon {
		info.Gradient = g
	} else {
		info.Gradient = closestNormal
	}
}

func (ps *PolyShape) SegmentQuery(a, b v.Vec, r2 float64, info *SegmentQueryInfo) {
	planes := ps.Planes
	count := ps.count
	r := ps.Radius
	rsum := r + r2

	for i := range count {
		n := planes[i].N
		an := a.Dot(n)
		d := an - planes[i].V0.Dot(n) - rsum
		if d < 0 {
			continue
		}

		bn := b.Dot(n)
		t := d / (an - bn)
		if t < 0 || 1 < t {
			continue
		}

		point := a.Lerp(b, t)
		dt := n.Cross(point)
		dtMin := n.Cross(planes[(i-1+count)%count].V0)
		dtMax := n.Cross(planes[i].V0)

		if dtMin <= dt && dt <= dtMax {
			info.Shape = ps.Shape
			info.Point = a.Lerp(b, t).Sub(n.Scale(r2))
			info.Normal = n
			info.Alpha = t
		}
	}

	// Also check against the beveled vertexes
	if rsum > 0 {
		for i := range count {
			circleInfo := SegmentQueryInfo{nil, b, v.Vec{}, 1}
			CircleSegmentQuery(ps.Shape, planes[i].V0, r, a, b, r2, &circleInfo)
			if circleInfo.Alpha < info.Alpha {
				*info = circleInfo
			}
		}
	}
}

func (ps *PolyShape) Count() int {
	return ps.count
}

func (ps *PolyShape) Vert(vertIndex int) v.Vec {
	return ps.Planes[vertIndex+ps.count].V0
}

func (ps *PolyShape) TransformVert(i int) v.Vec {
	return ps.Planes[i].V0
}

func (ps *PolyShape) SetVerts(count int, verts []v.Vec) {
	ps.count = count
	ps.Planes = make([]SplittingPlane, count*2)

	for i := range count {
		a := verts[(i-1+count)%count]
		b := verts[i]
		n := reversePerp(b.Sub(a)).Unit()

		ps.Planes[i+count].V0 = b
		ps.Planes[i+count].N = n
	}
}

func (ps *PolyShape) SetVertsUnsafe(count int, verts []v.Vec, transform Transform) {
	hullVerts := make([]v.Vec, count)

	for i := range count {
		hullVerts[i] = transform.Apply(verts[i])
	}

	hullCount := convexHull(count, hullVerts, nil, 0)
	ps.SetVertsRaw(hullCount, hullVerts)
}

func (p *PolyShape) SetVertsRaw(count int, verts []v.Vec) {
	p.SetVerts(count, verts)
	mass := p.massInfo.m
	p.massInfo = PolyShapeMassInfo(p.massInfo.m, count, verts, p.Radius)
	if mass > 0 {
		p.Body.AccumulateMassFromShapes()
	}
}

// QuickHull seemed like a neat algorithm, and efficient-ish for large input sets.
// My implementation performs an in place reduction using the result array as scratch space.
func convexHull(count int, verts []v.Vec, first *int, tol float64) int {
	start, end := loopIndexes(verts, count)
	if start == end {
		if first != nil {
			*first = 0
		}
		return 1
	}

	verts[0], verts[start] = verts[start], verts[0]
	if end == 0 {
		verts[1], verts[start] = verts[start], verts[1]
	} else {
		verts[1], verts[end] = verts[end], verts[1]
	}

	a := verts[0]
	b := verts[1]

	if first != nil {
		*first = start
	}

	return QHullReduce(tol, verts[2:], count-2, a, b, a, verts[1:]) + 1
}

func loopIndexes(verts []v.Vec, count int) (int, int) {
	start := 0
	end := 0

	min := verts[0]
	max := min

	for i := 1; i < count; i++ {
		v := verts[i]

		if v.X < min.X || (v.X == min.X && v.Y < min.Y) {
			min = v
			start = i
		} else if v.X > max.X || (v.X == max.X && v.Y > max.Y) {
			max = v
			end = i
		}
	}

	return start, end
}

func QHullReduce(tol float64, verts []v.Vec, count int, a, pivot, b v.Vec, result []v.Vec) int {
	if count == 0 {
		result[0] = pivot
		return 1
	}

	leftCount := QHullPartition(verts, count, a, pivot, tol)
	var index int
	if leftCount-1 >= 0 {
		index = QHullReduce(tol, verts[1:], leftCount-1, a, verts[0], pivot, result)
	}

	result[index] = pivot
	index++

	rightCount := QHullPartition(verts[leftCount:], count-leftCount, pivot, b, tol)
	if rightCount-1 < 0 {
		return index
	}
	return index + QHullReduce(tol, verts[leftCount+1:], rightCount-1, pivot, verts[leftCount], b, result[index:])
}

func QHullPartition(verts []v.Vec, count int, a, b v.Vec, tol float64) int {
	if count == 0 {
		return 0
	}

	max := 0.0
	pivot := 0

	delta := b.Sub(a)
	valueTol := tol * delta.Mag()

	head := 0
	for tail := count - 1; head <= tail; {
		value := verts[head].Sub(a).Cross(delta)
		if value > valueTol {
			if value > max {
				max = value
				pivot = head
			}

			head++
		} else {
			verts[head], verts[tail] = verts[tail], verts[head]
			tail--
		}
	}

	// move the new pivot to the front if it's not already there.
	if pivot != 0 {
		verts[0], verts[pivot] = verts[pivot], verts[0]
	}
	return head
}

func PolyShapeMassInfo(mass float64, count int, verts []v.Vec, r float64) *ShapeMassInfo {
	centroid := CentroidForPoly(count, verts)
	return &ShapeMassInfo{
		m:    mass,
		i:    MomentForPoly(1, count, verts, centroid.Neg(), r),
		cog:  centroid,
		area: AreaForPoly(count, verts, r),
	}
}
