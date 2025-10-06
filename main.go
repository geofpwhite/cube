package main

import (
	"context"
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/draw"
	"math"
	"sort"

	"fortio.org/terminal/ansipixels"
)

type faceInfo struct {
	idx  int
	avgZ float64
}

var vertices = [8][3]float64{
	{-1, -1, -1},
	{-1, 1, -1},
	{1, 1, -1},
	{1, -1, -1},
	{-1, -1, 1},
	{-1, 1, 1},
	{1, 1, 1},
	{1, -1, 1},
}

var edges = [12][2]int{
	{0, 1},
	{1, 2},
	{2, 3},
	{3, 0},
	{4, 5},
	{5, 6},
	{6, 7},
	{7, 4},
	{0, 4},
	{1, 5},
	{2, 6},
	{3, 7},
}

var faces = [6][4]int{
	{0, 1, 2, 3}, // back  (z = -1)
	{0, 1, 5, 4}, // left  (x = -1)
	{3, 2, 6, 7}, // right (x = +1)
	{1, 2, 6, 5}, // top   (y = +1)
	{0, 3, 7, 4}, // bottom(y = -1)
	{4, 5, 6, 7}, // front (z = +1)
}

var faceColors = [6]color.NRGBA{
	{200, 50, 50, 255},  // back - red
	{50, 200, 50, 255},  // front - green
	{50, 50, 200, 255},  // left - blue
	{200, 200, 50, 255}, // right - yellow
	{200, 50, 200, 255}, // top - magenta
	{50, 200, 200, 255}, // bottom - cyan
}

var lightSource = [3]float64{2, 0, -3}

func dot(a, b [3]float64) float64 {
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
}

func normalize(v [3]float64) [3]float64 {
	l := math.Sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
	if l == 0 {
		return v
	}
	return [3]float64{v[0] / l, v[1] / l, v[2] / l}
}

func faceNormal(f [4]int) [3]float64 {
	v0 := vertices[f[0]]
	v1 := vertices[f[1]]
	v2 := vertices[f[2]]
	e1 := [3]float64{v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]}
	e2 := [3]float64{v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]}
	n := [3]float64{
		e1[1]*e2[2] - e1[2]*e2[1],
		e1[2]*e2[0] - e1[0]*e2[2],
		e1[0]*e2[1] - e1[1]*e2[0],
	}
	return normalize(n)
}

// computeVertexNormals calculates a normalized normal vector for each vertex by
// averaging the normals of all faces that include the vertex. It uses the
// current (rotated) `vertices` positions, so call this after you transform the
// cube each frame. Degenerate or zero-length normals are detected with a small
// epsilon and skipped to avoid NaNs.
func computeVertexNormals() [8][3]float64 {
	var vnorms [8][3]float64
	var counts [8]int

	// accumulate face normals into each vertex
	for _, f := range faces {
		n := faceNormal(f)
		// if normal is degenerate, skip adding it
		if math.Sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]) < 1e-9 {
			continue
		}
		for _, vi := range f {
			vnorms[vi][0] += n[0]
			vnorms[vi][1] += n[1]
			vnorms[vi][2] += n[2]
			counts[vi]++
		}
	}

	// normalize averaged normals, fallback to face-based normal if zero
	for i := 0; i < len(vnorms); i++ {
		if counts[i] == 0 {
			// fallback: search any face that contains this vertex and use its normal
			for _, f := range faces {
				for _, vi := range f {
					if vi == i {
						vnorms[i] = faceNormal(f)
						counts[i] = 1
						break
					}
				}
				if counts[i] > 0 {
					break
				}
			}
		}
		// normalize final vector
		vnorms[i] = normalize(vnorms[i])
	}

	return vnorms
}

func shadeColor(c color.NRGBA, brightness float64) color.NRGBA {
	if brightness < 0 {
		brightness = 0
	}
	if brightness > 1 {
		brightness = 1
	}
	return color.NRGBA{
		R: uint8(float64(c.R) * brightness),
		G: uint8(float64(c.G) * brightness),
		B: uint8(float64(c.B) * brightness),
		A: c.A,
	}
}

// fillPolygon fills a convex polygon (or any simple polygon) on an NRGBA image using a scanline algorithm.
// AI generated bc I got stuck
func fillPolygon(img *image.NRGBA, poly [][2]int, col color.NRGBA) {
	if len(poly) < 3 {
		return
	}
	minY := poly[0][1]
	maxY := poly[0][1]
	for _, p := range poly {
		if p[1] < minY {
			minY = p[1]
		}
		if p[1] > maxY {
			maxY = p[1]
		}
	}
	if minY > img.Rect.Max.Y-1 || maxY < img.Rect.Min.Y {
		return
	}
	if minY < img.Rect.Min.Y {
		minY = img.Rect.Min.Y
	}
	if maxY > img.Rect.Max.Y-1 {
		maxY = img.Rect.Max.Y - 1
	}

	for y := minY; y <= maxY; y++ {
		var xs []float64
		n := len(poly)
		for i := 0; i < n; i++ {
			y := float64(y)
			x0 := float64(poly[i][0])
			y0 := float64(poly[i][1])
			x1 := float64(poly[(i+1)%n][0])
			y1 := float64(poly[(i+1)%n][1])
			// include scanline on lower endpoint, exclude on upper to avoid double count
			if (y >= y0 && y < y1) || (y >= y1 && y < y0) {
				xf := x0 + (y-y0)*(x1-x0)/(y1-y0)
				xs = append(xs, xf)
			}
		}
		if len(xs) < 2 {
			continue
		}
		sort.Float64s(xs)
		for i := 0; i < len(xs)-1; i += 2 {
			xStart := int(math.Ceil(xs[i]))
			xEnd := int(math.Floor(xs[i+1]))
			if xStart > img.Rect.Max.X-1 || xEnd < img.Rect.Min.X {
				continue
			}
			if xStart < img.Rect.Min.X {
				xStart = img.Rect.Min.X
			}
			if xEnd > img.Rect.Max.X-1 {
				xEnd = img.Rect.Max.X - 1
			}
			for x := xStart; x <= xEnd; x++ {
				img.SetNRGBA(x, y, col)
			}
		}
	}
	for i := range poly {
		x0, y0 := poly[i][0], poly[i][1]
		x1, y1 := poly[(i+1)%len(poly)][0], poly[(i+1)%len(poly)][1]
		ansipixels.DrawLine(img, float64(x0), float64(y0), float64(x1), float64(y1), color.NRGBA{255, 255, 255, 255})
	}
}

func rotateY(v [3]float64, ay float64) [3]float64 {
	cosa, sina := math.Cos(ay), math.Sin(ay)
	x := v[0]*cosa + v[2]*sina
	z := -v[0]*sina + v[2]*cosa
	return [3]float64{x, v[1], z}
}

func rotateX(v [3]float64, ax float64) [3]float64 {
	cosa, sina := math.Cos(ax), math.Sin(ax)
	y := v[1]*cosa - v[2]*sina
	z := v[1]*sina + v[2]*cosa
	return [3]float64{v[0], y, z}
}

func rotateZ(v [3]float64, az float64) [3]float64 {
	cosa, sina := math.Cos(az), math.Sin(az)
	y := v[1]*cosa - v[0]*sina
	x := v[0]*cosa + v[1]*sina
	return [3]float64{x, y, v[2]}
}

// interpolate linear between a and b by t
func lerp(a, b float64, t float64) float64 { return a + (b-a)*t }

// drawTriangleGouraud rasterizes a triangle with per-vertex colors (c0,c1,c2) using
// simple linear interpolation across scanlines. This is not perspective-correct
// but is sufficient for small scenes and orthographic-ish projection.
func drawTriangleGouraud(img *image.NRGBA, p0, p1, p2 [2]int, c0, c1, c2 color.NRGBA) {
	// sort vertices by y
	type pv struct {
		x, y int
		c    color.NRGBA
	}
	pts := []pv{{p0[0], p0[1], c0}, {p1[0], p1[1], c1}, {p2[0], p2[1], c2}}
	sort.Slice(pts, func(i, j int) bool { return pts[i].y < pts[j].y })

	// helper to interpolate color between two colors
	interpCol := func(a, b color.NRGBA, t float64) color.NRGBA {
		return color.NRGBA{
			R: uint8(lerp(float64(a.R), float64(b.R), t)),
			G: uint8(lerp(float64(a.G), float64(b.G), t)),
			B: uint8(lerp(float64(a.B), float64(b.B), t)),
			A: uint8(lerp(float64(a.A), float64(b.A), t)),
		}
	}

	xa, ya, ca := pts[0].x, pts[0].y, pts[0].c
	xb, yb, cb := pts[1].x, pts[1].y, pts[1].c
	xc, yc, cc := pts[2].x, pts[2].y, pts[2].c

	if ya == yc { // degenerate
		return
	}

	// rasterize from y=ya to y=yc
	for y := ya; y <= yc; y++ {
		if y < img.Rect.Min.Y || y > img.Rect.Max.Y-1 {
			continue
		}
		// compute x on long edge (a->c)
		tLong := float64(y-ya) / float64(yc-ya)
		xLong := int(math.Round(lerp(float64(xa), float64(xc), tLong)))
		cLong := interpCol(ca, cc, tLong)

		var xShort int
		var cShort color.NRGBA

		if y <= yb && yb != ya { // upper half (a->b)
			tShort := float64(y-ya) / float64(yb-ya)
			xShort = int(math.Round(lerp(float64(xa), float64(xb), tShort)))
			cShort = interpCol(ca, cb, tShort)
		} else if yb != yc { // lower half (b->c)
			tShort := float64(y-yb) / float64(yc-yb)
			xShort = int(math.Round(lerp(float64(xb), float64(xc), tShort)))
			cShort = interpCol(cb, cc, tShort)
		}
		xStart := xShort
		xEnd := xLong
		colStart := cShort
		colEnd := cLong
		if xStart > xEnd {
			xStart, xEnd = xEnd, xStart
			colStart, colEnd = colEnd, colStart
		}

		if xStart < img.Rect.Min.X {
			xStart = img.Rect.Min.X
		}
		if xEnd > img.Rect.Max.X-1 {
			xEnd = img.Rect.Max.X - 1
		}

		width := xEnd - xStart
		if width < 0 {
			continue
		}
		for x := xStart; x <= xEnd; x++ {
			var t float64
			if width == 0 {
				t = 0
			} else {
				t = float64(x-xStart) / float64(width)
			}
			col := interpCol(colStart, colEnd, t)
			img.SetNRGBA(x, y, col)
		}
	}
}

func project(v [3]float64, width, height int, scale float64) (int, int) {
	distance := 3.0
	scale = float64(width) / scale
	x := v[0] / (v[2] + distance) * scale
	y := v[1] / (v[2] + distance) * scale

	xc := int(x) + width/2
	yc := int(y) + height
	return xc, yc
}

func main() { //nolint:gocognit,gocyclo,funlen,lll // this handles the main drawing loop and handles input, it's going to get a little hairy
	fpsFlag := flag.Float64("fps", 60, "set the fps for the animation")
	colorFlag := flag.Bool("color", false, "color the faces of the cube")
	flag.Parse()
	ap := ansipixels.NewAnsiPixels(*fpsFlag)
	ap.HideCursor()
	if ap.Open() != nil {
		return
	}
	errMessage := ""
	shades := []string{}
	defer func() {
		ap.ShowCursor()
		ap.MouseTrackingOff()
		ap.MouseClickOff()
		ap.ClearScreen()
		ap.Restore()
		if len(errMessage) > 0 {
			fmt.Println(errMessage)
		}
		fmt.Println(shades)
		fmt.Println(computeVertexNormals())
	}()
	ap.MouseTrackingOn()
	ap.ClearScreen()
	xSpeed, ySpeed, zSpeed := .5, .5, .5
	var (
		width, height = ap.W, ap.H
		frames        = 60
	)
	scale := 5.
	img := image.NewNRGBA(image.Rect(0, 0, width, height*2))
	prevMousePosition := [2]int{}
	angle := math.Pi / float64(frames)
	ap.OnResize = func() error {
		width, height = ap.W, ap.H
		img = image.NewNRGBA(image.Rect(0, 0, width, height*2))
		return nil
	}

	drawCube := func(pts [][2]int) {
		// finfos := make([]faceInfo, 0, len(faces))
		// for i, f := range faces {
		// 	sum := 0.0
		// 	for _, vi := range f {
		// 		sum += vertices[vi][2]
		// 	}
		// 	finfos = append(finfos, faceInfo{i, sum / float64(len(f))})
		// }
		// sort.Slice(finfos, func(i, j int) bool { return finfos[i].avgZ > finfos[j].avgZ })
		// for _, fi := range finfos {
		// 	f := faces[fi.idx]
		// 	poly := make([][2]int, 0, 4)
		// 	for _, vi := range f {
		// 		poly = append(poly, pts[vi])
		// 	}
		// 	fillPolygon(img, poly, faceColors[fi.idx])
		// }
		finfos := make([]faceInfo, 0, len(faces))
		for i, f := range faces {
			sum := 0.0
			for _, vi := range f {
				sum += vertices[vi][2]
			}
			finfos = append(finfos, faceInfo{i, sum / float64(len(f))})
		}
		sort.Slice(finfos, func(i, j int) bool { return finfos[i].avgZ > finfos[j].avgZ })

		// prepare light direction (normalized)
		ld := normalize(lightSource)

		// compute per-vertex normals once (averaged across adjacent faces)
		vnorms := computeVertexNormals()
		ambient := 0.25
		diffuse := 0.75

		// draw faces as two triangles each with Gouraud shading, but use the face's
		// base color for all vertex colors for that face. This preserves clear face
		// colors while allowing brightness to vary per-vertex (smooth shading within a face).
		for _, fi := range finfos[len(finfos)-3:] {
			f := faces[fi.idx]
			// optionally back-face cull using face normal
			// compute per-vertex shaded colors for this face using the face base color
			var c0, c1, c2, c3 color.NRGBA
			ndotl := dot(vnorms[f[0]], ld)
			if ndotl < 0 {
				ndotl *= -1
			}
			c0 = shadeColor(faceColors[fi.idx], ambient+(diffuse*ndotl))

			ndotl = dot(vnorms[f[1]], ld)
			if ndotl < 0 {
				ndotl *= -1
			}
			c1 = shadeColor(faceColors[fi.idx], ambient+(diffuse*ndotl))

			ndotl = dot(vnorms[f[2]], ld)
			if ndotl < 0 {
				ndotl *= -1
			}
			c2 = shadeColor(faceColors[fi.idx], ambient+(diffuse*ndotl))

			ndotl = dot(vnorms[f[3]], ld)
			// if ndotl < 0 {
			// 	ndotl *= -1
			// }
			c3 = shadeColor(faceColors[fi.idx], ambient+(diffuse*ndotl))

			// triangle 1: f0, f1, f2
			drawTriangleGouraud(img, pts[f[0]], pts[f[1]], pts[f[2]], c0, c1, c2)
			// triangle 2: f0, f2, f3
			drawTriangleGouraud(img, pts[f[0]], pts[f[2]], pts[f[3]], c0, c2, c3)
		}
	}
	if !*colorFlag {
		drawCube = func(pts [][2]int) {
			for _, e := range edges {
				x0, y0 := pts[e[0]][0], pts[e[0]][1]
				x1, y1 := pts[e[1]][0], pts[e[1]][1]
				ansipixels.DrawLine(img, float64(x0), float64(y0), float64(x1), float64(y1), color.NRGBA{255, 255, 255, 255})
			}
		}
	}
	// ap.RequestBackgroundColor()
	ap.SyncBackgroundColor()
	err := ap.FPSTicks(
		context.Background(),
		func(context.Context) bool {
			clear(img.Pix)
			draw.Draw(img, image.Rect(
				0,
				0,
				ap.W,
				ap.H*2,
			), &image.Uniform{color.RGBA{ap.Background.R, ap.Background.G, ap.Background.B, 255}}, image.Point{}, draw.Over)

			barWidth := img.Bounds().Dx() / 20
			barHeightImg := img.Bounds().Dy() / 3
			barHeightAp := ap.H / 3
			pts := make([][2]int, len(vertices))
			for j, v := range vertices {
				rv := rotateX(v, angle*xSpeed)
				rv = rotateY(rv, angle*ySpeed)
				rv = rotateZ(rv, angle*zSpeed)
				pts[j][0], pts[j][1] = project(rv, width, height, scale)
				vertices[j] = rv
			}
			lc, ld := ap.LeftClick(), ap.LeftDrag()
			switch {
			case (lc || ld) && ap.Mx >= 0 && ap.Mx <= barWidth+1:
				if ap.My >= ap.H-barHeightAp {
					xSpeed = float64(ap.H-ap.My) / float64(barHeightAp)
				}
			case (lc || ld) && ap.Mx > barWidth+1 && ap.Mx <= barWidth*2+1:
				if ap.My >= ap.H-barHeightAp {
					ySpeed = float64(ap.H-ap.My) / float64(barHeightAp)
				}
			case (lc || ld) && ap.Mx > barWidth*2+1 && ap.Mx <= barWidth*3+1:
				if ap.My >= ap.H-barHeightAp {
					zSpeed = float64(ap.H-ap.My) / float64(barHeightAp)
				}
			case ld:
				xSpeed, ySpeed, zSpeed = 0, 0, 0
				if prevMousePosition[0] != ap.Mx {
					for j, v := range vertices {
						rv := rotateY(v, -angle*.7*(float64(ap.Mx-(prevMousePosition[0]))))
						pts[j][0], pts[j][1] = project(rv, width, height, scale)
						vertices[j] = rv
					}
				}
				if prevMousePosition[1] != ap.My {
					for j, v := range vertices {
						rv := rotateX(v, angle*.7*(float64(ap.My-(prevMousePosition[1]))))
						pts[j][0], pts[j][1] = project(rv, width, height, scale)
						vertices[j] = rv
					}
				}
			}
			if ap.RightDrag() {
				xSpeed, ySpeed, zSpeed = 0, 0, 0
				if prevMousePosition[0] != ap.Mx {
					for j, v := range vertices {
						rv := rotateZ(v, -angle*.7*(float64(ap.Mx-(prevMousePosition[0]))))
						pts[j][0], pts[j][1] = project(rv, width, height, scale)
						vertices[j] = rv
					}
				}
			}
			if ap.MouseWheelUp() {
				scale *= .9
			}
			if ap.MouseWheelDown() {
				scale /= .9
			}
			prevMousePosition = [2]int{ap.Mx, ap.My}
			drawCube(pts)
			err := drawImageAndSliders(ap, img, barWidth, barHeightImg, barHeightAp, xSpeed, ySpeed, zSpeed)
			if err != nil {
				return false
			}
			if len(ap.Data) > 0 && ap.Data[0] == 'q' {
				return false
			}
			return true
		},
	)
	if err != nil {
		errMessage = err.Error()
	}
}

func drawImageAndSliders(
	ap *ansipixels.AnsiPixels,
	img *image.NRGBA,
	barWidth, barHeightImg, barHeightAp int,
	xSpeed, ySpeed, zSpeed float64,
) error {
	draw.Draw(img, image.Rect(
		1,
		img.Bounds().Dy()-(int(xSpeed*float64(barHeightImg)))-2,
		barWidth,
		img.Bounds().Dy()-2,
	), &image.Uniform{color.RGBA{145, 0, 0, 255}}, image.Point{}, draw.Over)
	draw.Draw(
		img,
		image.Rect(barWidth+2,
			img.Bounds().Dy()-(int(ySpeed*float64(barHeightImg)))-2,
			(barWidth*2)+1, img.Bounds().Dy()-2), &image.Uniform{color.RGBA{0, 145, 0, 255}}, image.Point{}, draw.Over)
	draw.Draw(img, image.Rect(
		(barWidth*2)+3,
		img.Bounds().Dy()-(int(zSpeed*float64(barHeightImg)))-2,
		(barWidth*2)+barWidth+2,
		img.Bounds().Dy()-2,
	), &image.Uniform{color.RGBA{0, 0, 145, 255}}, image.Point{}, draw.Over)
	ap.StartSyncMode()
	if ap.ColorOutput.TrueColor {
		err := ap.DrawTrueColorImage(0, 0, &image.RGBA{Pix: img.Pix, Stride: img.Stride, Rect: img.Rect})
		if err != nil {
			return err
		}
	} else {
		err := ap.Draw216ColorImage(0, 0, &image.RGBA{Pix: img.Pix, Stride: img.Stride, Rect: img.Rect})
		if err != nil {
			return err
		}
	}
	ap.WriteBg(ap.Background.Color())
	ap.DrawRoundBox(0, ap.H-barHeightAp, barWidth+1, barHeightAp)
	ap.DrawRoundBox(barWidth+1, ap.H-barHeightAp, barWidth+1, barHeightAp)
	ap.DrawRoundBox((barWidth*2)+2, ap.H-barHeightAp, barWidth+1, barHeightAp)
	ap.EndSyncMode()
	return nil
}
