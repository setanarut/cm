package cm

import "github.com/setanarut/v"

// Segment is a segment Shape
type Segment struct {
	*Shape
	a, b, n                            v.Vec
	transformA, transformB, transformN v.Vec
	radius                             float64
	aTangent, bTangent                 v.Vec
}

func (seg *Segment) CacheData(transform Transform) BB {
	seg.transformA = transform.Apply(seg.a)
	seg.transformB = transform.Apply(seg.b)
	seg.transformN = transform.ApplyVector(seg.n)

	var l, r, b, t float64

	if seg.transformA.X < seg.transformB.X {
		l = seg.transformA.X
		r = seg.transformB.X
	} else {
		l = seg.transformB.X
		r = seg.transformA.X
	}

	if seg.transformA.Y < seg.transformB.Y {
		b = seg.transformA.Y
		t = seg.transformB.Y
	} else {
		b = seg.transformB.Y
		t = seg.transformA.Y
	}

	rad := seg.radius
	return BB{l - rad, b - rad, r + rad, t + rad}
}

func (seg *Segment) PointQuery(p v.Vec, info *PointQueryInfo) {
	closest := closestPointOnSegment(p, seg.transformA, seg.transformB)

	delta := p.Sub(closest)
	d := delta.Mag()
	r := seg.radius
	g := delta.Scale(1 / d)

	info.Shape = seg.Shape
	if d != 0 {
		info.Point = closest.Add(g.Scale(r))
	} else {
		info.Point = closest
	}
	info.Distance = d - r

	// Use the segment's normal if the distance is very small.
	if d > magicEpsilon {
		info.Gradient = g
	} else {
		info.Gradient = seg.n
	}
}

func (seg *Segment) SegmentQuery(a, b v.Vec, r2 float64, info *SegmentQueryInfo) {
	n := seg.transformN
	d := seg.transformA.Sub(a).Dot(n)
	r := seg.radius + r2

	var flippedN v.Vec
	if d > 0 {
		flippedN = n.Neg()
	} else {
		flippedN = n
	}
	segOffset := flippedN.Scale(r).Sub(a)

	// Make the endpoints relative to 'a' and move them by the thickness of the segment.
	segA := seg.transformA.Add(segOffset)
	segB := seg.transformB.Add(segOffset)
	delta := b.Sub(a)

	if delta.Cross(segA)*delta.Cross(segB) <= 0 {
		dOffset := d
		if d > 0 {
			dOffset -= r
		} else {
			dOffset += r
		}
		ad := -dOffset
		bd := delta.Dot(n) - dOffset

		if ad*bd < 0 {
			t := ad / (ad - bd)

			info.Shape = seg.Shape
			info.Point = a.Lerp(b, t).Sub(flippedN.Scale(r2))
			info.Normal = flippedN
			info.Alpha = t
		}
	} else if r != 0 {
		info1 := SegmentQueryInfo{nil, b, v.Vec{}, 1}
		info2 := SegmentQueryInfo{nil, b, v.Vec{}, 1}
		CircleSegmentQuery(seg.Shape, seg.transformA, seg.radius, a, b, r2, &info1)
		CircleSegmentQuery(seg.Shape, seg.transformB, seg.radius, a, b, r2, &info2)

		if info1.Alpha < info2.Alpha {
			*info = info1
		} else {
			*info = info2
		}
	}
}

func (seg *Segment) SetRadius(r float64) {
	seg.radius = r

	mass := seg.massInfo.m
	seg.massInfo = NewSegmentMassInfo(seg.massInfo.m, seg.a, seg.b, seg.radius)
	if mass > 0 {
		seg.Body.AccumulateMassFromShapes()
	}
}

func (seg *Segment) Radius() float64 {
	return seg.radius
}

func (seg *Segment) TransformA() v.Vec {
	return seg.transformA
}

func (seg *Segment) TransformB() v.Vec {
	return seg.transformB
}

func (seg *Segment) SetEndpoints(a, b v.Vec) {
	seg.a = a
	seg.b = b
	seg.n = perp(b.Sub(a).Unit())

	mass := seg.massInfo.m
	seg.massInfo = NewSegmentMassInfo(seg.massInfo.m, seg.a, seg.b, seg.radius)
	if mass > 0 {
		seg.Body.AccumulateMassFromShapes()
	}
}

func (seg *Segment) Normal() v.Vec {
	return seg.n
}

func (seg *Segment) A() v.Vec {
	return seg.a
}

func (seg *Segment) B() v.Vec {
	return seg.b
}

func NewSegmentMassInfo(mass float64, a, b v.Vec, r float64) *ShapeMassInfo {
	return &ShapeMassInfo{
		m:    mass,
		i:    MomentForBox(1, a.Dist(b)+2*r, 2*r),
		cog:  a.Lerp(b, 0.5),
		area: AreaForSegment(a, b, r),
	}
}
