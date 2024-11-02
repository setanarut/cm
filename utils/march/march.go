package march

import (
	"github.com/setanarut/cm"
	"github.com/setanarut/vec"
)

// This is a user defined function that gets passed in to the Marching process
// the user establishes a PolyLineSet, passes a pointer to their function, and they
// populate it. In most cases you want to use PolyLineCollectSegment instead of defining your own
type MarchSegFunc func(v0 vec.Vec2, v1 vec.Vec2, segmentData *cm.PolyLineSet)

// This is a user defined function that gets passed every single point from thebounding
// box the user passes into the March process - you can use this to sample an image and
// check for alpha values or really any 2d matrix you define like a tile map.
// NOTE: I could not determine a use case for the sampleData pointer from the original code
// so I removed it here - open to adding it back in if there is a reason.
type MarchSampleFunc func(point vec.Vec2) float64

type MarchCellFunc func(
	t, a, b, c, d, x0, x1, y0, y1 float64,
	marchSegment MarchSegFunc,
	segmentData *cm.PolyLineSet,
)

// MarchCells samples and processes a grid of cells within the specified bounding box (bb).
// It interpolates values across the grid based on the provided sample count in the x and y
// directions. The function uses the given marching segment, sample, and cell functions
// to compute segments for each cell in the grid.
//
// Parameters:
//   - bb: The bounding box defining the area to be processed.
//   - xSamples: The number of samples to take along the x-axis.
//   - ySamples: The number of samples to take along the y-axis.
//   - t: A parameter (e.g., time) used for the computation, influencing the results of
//     marching operations.
//   - marchSegment: A function that handles operations for a segment based on cell values.
//   - marchSample: A function that computes sample values at specific coordinates.
//   - marchCell: A function that processes individual cells based on their corner values.
//
// Returns:
// - A pointer to a PolyLineSet containing the computed segments across the grid.
func MarchCells(
	bb cm.BB,
	xSamples, ySamples int64,
	t float64,
	marchSegment MarchSegFunc,
	marchSample MarchSampleFunc,
	marchCell MarchCellFunc,
) *cm.PolyLineSet {
	var xDenom, yDenom float64
	xDenom = 1.0 / float64(xSamples-1)
	yDenom = 1.0 / float64(ySamples-1)

	buffer := make([]float64, xSamples)
	var i, j int64
	for i = 0; i < xSamples; i++ {
		buffer[i] = marchSample(vec.Vec2{lerp(bb.L, bb.R, float64(i)*xDenom), bb.B})
	}
	segmentData := &cm.PolyLineSet{}

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

func Seg(v0, v1 vec.Vec2, marchSeg MarchSegFunc, segmentData *cm.PolyLineSet) {
	if !v0.Equal(v1) {
		marchSeg(v1, v0, segmentData)
	}
}

func Midlerp(x0, x1, s0, s1, t float64) float64 {
	return lerp(x0, x1, (t-s0)/(s1-s0))
}

func MarchCellSoft(
	t, a, b, c, d, x0, x1, y0, y1 float64,
	marchSeg MarchSegFunc,
	segData *cm.PolyLineSet,
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
		Seg(
			vec.Vec2{x0, Midlerp(y0, y1, a, c, t)},
			vec.Vec2{Midlerp(x0, x1, a, b, t), y0},
			marchSeg,
			segData,
		)
	case 0x2:
		Seg(
			vec.Vec2{Midlerp(x0, x1, a, b, t), y0},
			vec.Vec2{x1, Midlerp(y0, y1, b, d, t)},
			marchSeg,
			segData,
		)
	case 0x3:
		Seg(
			vec.Vec2{x0, Midlerp(y0, y1, a, c, t)},
			vec.Vec2{x1, Midlerp(y0, y1, b, d, t)},
			marchSeg,
			segData,
		)
	case 0x4:
		Seg(
			vec.Vec2{Midlerp(x0, x1, c, d, t), y1},
			vec.Vec2{x0, Midlerp(y0, y1, a, c, t)},
			marchSeg,
			segData,
		)
	case 0x5:
		Seg(
			vec.Vec2{Midlerp(x0, x1, c, d, t), y1},
			vec.Vec2{Midlerp(x0, x1, a, b, t), y0},
			marchSeg,
			segData,
		)
	case 0x6:
		Seg(
			vec.Vec2{Midlerp(x0, x1, a, b, t), y0},
			vec.Vec2{x1, Midlerp(y0, y1, b, d, t)},
			marchSeg,
			segData,
		)
		Seg(
			vec.Vec2{Midlerp(x0, x1, c, d, t), y1},
			vec.Vec2{x0, Midlerp(y0, y1, a, c, t)},
			marchSeg,
			segData,
		)
	case 0x7:
		Seg(
			vec.Vec2{Midlerp(x0, x1, c, d, t), y1},
			vec.Vec2{x1, Midlerp(y0, y1, b, d, t)},
			marchSeg,
			segData,
		)
	case 0x8:
		Seg(
			vec.Vec2{x1, Midlerp(y0, y1, b, d, t)},
			vec.Vec2{Midlerp(x0, x1, c, d, t), y1},
			marchSeg,
			segData,
		)
	case 0x9:
		Seg(
			vec.Vec2{x0, Midlerp(y0, y1, a, c, t)},
			vec.Vec2{Midlerp(x0, x1, a, b, t), y0},
			marchSeg,
			segData,
		)
		Seg(
			vec.Vec2{x1, Midlerp(y0, y1, b, d, t)},
			vec.Vec2{Midlerp(x0, x1, c, d, t), y1},
			marchSeg,
			segData,
		)
	case 0xA:
		Seg(
			vec.Vec2{Midlerp(x0, x1, a, b, t), y0},
			vec.Vec2{Midlerp(x0, x1, c, d, t), y1},
			marchSeg,
			segData,
		)
	case 0xB:
		Seg(
			vec.Vec2{x0, Midlerp(y0, y1, a, c, t)},
			vec.Vec2{Midlerp(x0, x1, c, d, t), y1},
			marchSeg,
			segData,
		)
	case 0xC:
		Seg(
			vec.Vec2{x1, Midlerp(y0, y1, b, d, t)},
			vec.Vec2{x0, Midlerp(y0, y1, a, c, t)},
			marchSeg,
			segData,
		)
	case 0xD:
		Seg(
			vec.Vec2{x1, Midlerp(y0, y1, b, d, t)},
			vec.Vec2{Midlerp(x0, x1, a, b, t), y0},
			marchSeg,
			segData,
		)
	case 0xE:
		Seg(
			vec.Vec2{Midlerp(x0, x1, a, b, t), y0},
			vec.Vec2{x0, Midlerp(y0, y1, a, c, t)},
			marchSeg,
			segData,
		)
	}
}

// Trace an anti-aliased contour of an image along a particular threshold.
// The given number of samples will be taken and spread across the bounding box
// area using the sampling function and context. The segment function will be
// called for each segment detected that lies along the density contour for @c threshold.
func MarchSoft(
	bb cm.BB,
	xSamples, ySamples int64,
	t float64,
	marchSeg MarchSegFunc,
	marchSample MarchSampleFunc,
) *cm.PolyLineSet {
	return MarchCells(bb, xSamples, ySamples, t, marchSeg, marchSample, MarchCellSoft)
}

func Segs(a, b, c vec.Vec2, marchSegment MarchSegFunc, segmentData *cm.PolyLineSet) {
	Seg(b, c, marchSegment, segmentData)
	Seg(a, b, marchSegment, segmentData)
}

func MarchCellHard(
	t, a, b, c, d, x0, x1, y0, y1 float64,
	marchSeg MarchSegFunc,
	segData *cm.PolyLineSet,
) {
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
		Segs(vec.Vec2{x0, ym}, vec.Vec2{xm, ym}, vec.Vec2{xm, y0}, marchSeg, segData)
	case 0x2:
		Segs(vec.Vec2{xm, y0}, vec.Vec2{xm, ym}, vec.Vec2{x1, ym}, marchSeg, segData)
	case 0x3:
		Seg(vec.Vec2{x0, ym}, vec.Vec2{x1, ym}, marchSeg, segData)
	case 0x4:
		Segs(vec.Vec2{xm, y1}, vec.Vec2{xm, ym}, vec.Vec2{x0, ym}, marchSeg, segData)
	case 0x5:
		Seg(vec.Vec2{xm, y1}, vec.Vec2{xm, y0}, marchSeg, segData)
	case 0x6:
		Segs(vec.Vec2{xm, y0}, vec.Vec2{xm, ym}, vec.Vec2{x0, ym}, marchSeg, segData)
		Segs(vec.Vec2{xm, y1}, vec.Vec2{xm, ym}, vec.Vec2{x1, ym}, marchSeg, segData)
	case 0x7:
		Segs(vec.Vec2{xm, y1}, vec.Vec2{xm, ym}, vec.Vec2{x1, ym}, marchSeg, segData)
	case 0x8:
		Segs(vec.Vec2{x1, ym}, vec.Vec2{xm, ym}, vec.Vec2{xm, y1}, marchSeg, segData)
	case 0x9:
		Segs(vec.Vec2{x1, ym}, vec.Vec2{xm, ym}, vec.Vec2{xm, y0}, marchSeg, segData)
		Segs(vec.Vec2{x0, ym}, vec.Vec2{xm, ym}, vec.Vec2{xm, y1}, marchSeg, segData)
	case 0xA:
		Seg(vec.Vec2{xm, y0}, vec.Vec2{xm, y1}, marchSeg, segData)
	case 0xB:
		Segs(vec.Vec2{x0, ym}, vec.Vec2{xm, ym}, vec.Vec2{xm, y1}, marchSeg, segData)
	case 0xC:
		Seg(vec.Vec2{x1, ym}, vec.Vec2{x0, ym}, marchSeg, segData)
	case 0xD:
		Segs(vec.Vec2{x1, ym}, vec.Vec2{xm, ym}, vec.Vec2{xm, y0}, marchSeg, segData)
	case 0xE:
		Segs(vec.Vec2{xm, y0}, vec.Vec2{xm, ym}, vec.Vec2{x0, ym}, marchSeg, segData)
	}
}

// Trace an aliased curve of an image along a particular threshold.
// The given number of samples will be taken and spread across the bounding box
// area using the sampling function and context. The segment function will be called
// for each segment detected that lies along the density contour for @c threshold.
func MarchHard(
	bb cm.BB,
	xSamples, ySamples int64,
	t float64,
	marchSegment MarchSegFunc,
	marchSample MarchSampleFunc,
) *cm.PolyLineSet {
	return MarchCells(bb, xSamples, ySamples, t, marchSegment, marchSample, MarchCellHard)
}

func lerp(f1, f2, t float64) float64 {
	return f1*(1.0-t) + f2*t
}
