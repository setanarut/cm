package cm

import "github.com/setanarut/vec"

// This is a user defined function that gets passed in to the Marching process
// the user establishes a PolyLineSet, passes a pointer to their function, and they
// populate it. In most cases you want to use PolyLineCollectSegment instead of defining your own
type MarchSegFunc func(v0 vec.Vec2, v1 vec.Vec2, segmentData *PolyLineSet)

// This is a user defined function that gets passed every single point from thebounding
// box the user passes into the March process - you can use this to sample an image and
// check for alpha values or really any 2d matrix you define like a tile map.
// NOTE: I could not determine a use case for the sampleData pointer from the original code
// so I removed it here - open to adding it back in if there is a reason.
type MarchSampleFunc func(point vec.Vec2) float64

type MarchCellFunc func(
	t, a, b, c, d, x0, x1, y0, y1 float64,
	marchSegment MarchSegFunc,
	segmentData *PolyLineSet,
)

// The looping and sample caching code is shared between MarchHard() and MarchSoft().
func MarchCells(
	bb BB,
	xSamples, ySamples int64,
	t float64,
	marchSegment MarchSegFunc,
	marchSample MarchSampleFunc,
	marchCell MarchCellFunc,
) *PolyLineSet {
	var xDenom, yDenom float64
	xDenom = 1.0 / float64(xSamples-1)
	yDenom = 1.0 / float64(ySamples-1)

	buffer := make([]float64, xSamples)
	var i, j int64
	for i = 0; i < xSamples; i++ {
		buffer[i] = marchSample(vec.Vec2{lerp(bb.L, bb.R, float64(i)*xDenom), bb.B})
	}
	segmentData := &PolyLineSet{}

	for j = 0; j < ySamples-1; j++ {
		y0 := lerp(bb.B, bb.T, float64(j+0)*yDenom)
		y1 := lerp(bb.B, bb.T, float64(j+1)*yDenom)

		// a := buffer[0] // unused variable ?
		b := buffer[0]
		c := marchSample(vec.Vec2{bb.L, y1})
		d := c
		buffer[0] = d

		for i = 0; i < xSamples-1; i++ {
			x0 := lerp(bb.L, bb.R, float64(i+0)*xDenom)
			x1 := lerp(bb.L, bb.R, float64(i+1)*xDenom)

			a := b // = -> :=
			b = buffer[i+1]
			c = d
			d = marchSample(vec.Vec2{x1, y1})
			buffer[i+1] = d

			marchCell(t, a, b, c, d, x0, x1, y0, y1, marchSegment, segmentData)
		}
	}

	return segmentData
}

func seg(v0, v1 vec.Vec2, marchSeg MarchSegFunc, segmentData *PolyLineSet) {
	if !v0.Equal(v1) {
		marchSeg(v1, v0, segmentData)
	}
}

func midlerp(x0, x1, s0, s1, t float64) float64 {
	return lerp(x0, x1, (t-s0)/(s1-s0))
}

func MarchCellSoft(
	t, a, b, c, d, x0, x1, y0, y1 float64,
	marchSeg MarchSegFunc,
	segData *PolyLineSet,
) {
	at := 0
	bt := 0
	ct := 0
	dt := 0
	if a > t {
		at = 1
	}
	if b > t {
		bt = 1
	}
	if c > t {
		ct = 1
	}
	if d > t {
		dt = 1
	}

	switch (at)<<0 | (bt)<<1 | (ct)<<2 | (dt)<<3 {
	case 0x1:
		seg(
			vec.Vec2{x0, midlerp(y0, y1, a, c, t)},
			vec.Vec2{midlerp(x0, x1, a, b, t), y0},
			marchSeg,
			segData,
		)
	case 0x2:
		seg(
			vec.Vec2{midlerp(x0, x1, a, b, t), y0},
			vec.Vec2{x1, midlerp(y0, y1, b, d, t)},
			marchSeg,
			segData,
		)
	case 0x3:
		seg(
			vec.Vec2{x0, midlerp(y0, y1, a, c, t)},
			vec.Vec2{x1, midlerp(y0, y1, b, d, t)},
			marchSeg,
			segData,
		)
	case 0x4:
		seg(
			vec.Vec2{midlerp(x0, x1, c, d, t), y1},
			vec.Vec2{x0, midlerp(y0, y1, a, c, t)},
			marchSeg,
			segData,
		)
	case 0x5:
		seg(
			vec.Vec2{midlerp(x0, x1, c, d, t), y1},
			vec.Vec2{midlerp(x0, x1, a, b, t), y0},
			marchSeg,
			segData,
		)
	case 0x6:
		seg(
			vec.Vec2{midlerp(x0, x1, a, b, t), y0},
			vec.Vec2{x1, midlerp(y0, y1, b, d, t)},
			marchSeg,
			segData,
		)
		seg(
			vec.Vec2{midlerp(x0, x1, c, d, t), y1},
			vec.Vec2{x0, midlerp(y0, y1, a, c, t)},
			marchSeg,
			segData,
		)
	case 0x7:
		seg(
			vec.Vec2{midlerp(x0, x1, c, d, t), y1},
			vec.Vec2{x1, midlerp(y0, y1, b, d, t)},
			marchSeg,
			segData,
		)
	case 0x8:
		seg(
			vec.Vec2{x1, midlerp(y0, y1, b, d, t)},
			vec.Vec2{midlerp(x0, x1, c, d, t), y1},
			marchSeg,
			segData,
		)
	case 0x9:
		seg(
			vec.Vec2{x0, midlerp(y0, y1, a, c, t)},
			vec.Vec2{midlerp(x0, x1, a, b, t), y0},
			marchSeg,
			segData,
		)
		seg(
			vec.Vec2{x1, midlerp(y0, y1, b, d, t)},
			vec.Vec2{midlerp(x0, x1, c, d, t), y1},
			marchSeg,
			segData,
		)
	case 0xA:
		seg(
			vec.Vec2{midlerp(x0, x1, a, b, t), y0},
			vec.Vec2{midlerp(x0, x1, c, d, t), y1},
			marchSeg,
			segData,
		)
	case 0xB:
		seg(
			vec.Vec2{x0, midlerp(y0, y1, a, c, t)},
			vec.Vec2{midlerp(x0, x1, c, d, t), y1},
			marchSeg,
			segData,
		)
	case 0xC:
		seg(
			vec.Vec2{x1, midlerp(y0, y1, b, d, t)},
			vec.Vec2{x0, midlerp(y0, y1, a, c, t)},
			marchSeg,
			segData,
		)
	case 0xD:
		seg(
			vec.Vec2{x1, midlerp(y0, y1, b, d, t)},
			vec.Vec2{midlerp(x0, x1, a, b, t), y0},
			marchSeg,
			segData,
		)
	case 0xE:
		seg(
			vec.Vec2{midlerp(x0, x1, a, b, t), y0},
			vec.Vec2{x0, midlerp(y0, y1, a, c, t)},
			marchSeg,
			segData,
		)
	}
}

// Trace an anti-aliased contour of an image along a particular threshold.
// The given number of samples will be taken and spread across the bounding box area using the sampling function and context.
// The segment function will be called for each segment detected that lies along the density contour for @c threshold.
func MarchSoft(bb BB, xSamples, ySamples int64, t float64, marchSeg MarchSegFunc, marchSample MarchSampleFunc) *PolyLineSet {
	return MarchCells(bb, xSamples, ySamples, t, marchSeg, marchSample, MarchCellSoft)
}

func segs(a, b, c vec.Vec2, marchSegment MarchSegFunc, segmentData *PolyLineSet) {
	seg(b, c, marchSegment, segmentData)
	seg(a, b, marchSegment, segmentData)
}

func MarchCellHard(t, a, b, c, d, x0, x1, y0, y1 float64, marchSeg MarchSegFunc, segData *PolyLineSet) {
	xm := lerp(x0, x1, 0.5)
	ym := lerp(y0, y1, 0.5)

	at := 0
	bt := 0
	ct := 0
	dt := 0
	if a > t {
		at = 1
	}
	if b > t {
		bt = 1
	}
	if c > t {
		ct = 1
	}
	if d > t {
		dt = 1
	}

	switch (at)<<0 | (bt)<<1 | (ct)<<2 | (dt)<<3 {
	case 0x1:
		segs(vec.Vec2{x0, ym}, vec.Vec2{xm, ym}, vec.Vec2{xm, y0}, marchSeg, segData)
	case 0x2:
		segs(vec.Vec2{xm, y0}, vec.Vec2{xm, ym}, vec.Vec2{x1, ym}, marchSeg, segData)
	case 0x3:
		seg(vec.Vec2{x0, ym}, vec.Vec2{x1, ym}, marchSeg, segData)
	case 0x4:
		segs(vec.Vec2{xm, y1}, vec.Vec2{xm, ym}, vec.Vec2{x0, ym}, marchSeg, segData)
	case 0x5:
		seg(vec.Vec2{xm, y1}, vec.Vec2{xm, y0}, marchSeg, segData)
	case 0x6:
		segs(vec.Vec2{xm, y0}, vec.Vec2{xm, ym}, vec.Vec2{x0, ym}, marchSeg, segData)
		segs(vec.Vec2{xm, y1}, vec.Vec2{xm, ym}, vec.Vec2{x1, ym}, marchSeg, segData)
	case 0x7:
		segs(vec.Vec2{xm, y1}, vec.Vec2{xm, ym}, vec.Vec2{x1, ym}, marchSeg, segData)
	case 0x8:
		segs(vec.Vec2{x1, ym}, vec.Vec2{xm, ym}, vec.Vec2{xm, y1}, marchSeg, segData)
	case 0x9:
		segs(vec.Vec2{x1, ym}, vec.Vec2{xm, ym}, vec.Vec2{xm, y0}, marchSeg, segData)
		segs(vec.Vec2{x0, ym}, vec.Vec2{xm, ym}, vec.Vec2{xm, y1}, marchSeg, segData)
	case 0xA:
		seg(vec.Vec2{xm, y0}, vec.Vec2{xm, y1}, marchSeg, segData)
	case 0xB:
		segs(vec.Vec2{x0, ym}, vec.Vec2{xm, ym}, vec.Vec2{xm, y1}, marchSeg, segData)
	case 0xC:
		seg(vec.Vec2{x1, ym}, vec.Vec2{x0, ym}, marchSeg, segData)
	case 0xD:
		segs(vec.Vec2{x1, ym}, vec.Vec2{xm, ym}, vec.Vec2{xm, y0}, marchSeg, segData)
	case 0xE:
		segs(vec.Vec2{xm, y0}, vec.Vec2{xm, ym}, vec.Vec2{x0, ym}, marchSeg, segData)
	}
}

// Trace an aliased curve of an image along a particular threshold.
// The given number of samples will be taken and spread across the bounding box area using the sampling function and context.
// The segment function will be called for each segment detected that lies along the density contour for @c threshold.
func MarchHard(bb BB, xSamples, ySamples int64, t float64, marchSegment MarchSegFunc, marchSample MarchSampleFunc) *PolyLineSet {
	return MarchCells(bb, xSamples, ySamples, t, marchSegment, marchSample, MarchCellHard)
}
