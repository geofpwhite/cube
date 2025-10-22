package main

import (
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/draw"
	"math"
	"slices"

	"fortio.org/terminal/ansipixels"
)

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

// shadeColorTrue applies brightness in linear light (sRGB <-> linear) which
// preserves perceived contrast when using truecolor terminals.
func shadeColorTrue(c color.NRGBA, brightness float64) color.NRGBA {
	if brightness < 0 {
		brightness = 0
	}
	if brightness > 1 {
		brightness = 1
	}

	srgbToLinear := func(v uint8) float64 {
		x := float64(v) / 255.0
		if x <= 0.04045 {
			return x / 12.92
		}
		return math.Pow((x+0.055)/1.055, 2.4)
	}
	linearToSrgb := func(x float64) uint8 {
		if x <= 0.0 {
			return 0
		}
		if x >= 1.0 {
			return 255
		}
		var s float64
		if x <= 0.0031308 {
			s = x * 12.92
		} else {
			s = 1.055*math.Pow(x, 1.0/2.4) - 0.055
		}
		return uint8(math.Round(s * 255.0))
	}

	lr := srgbToLinear(c.R)
	lg := srgbToLinear(c.G)
	lb := srgbToLinear(c.B)

	lr *= brightness
	lg *= brightness
	lb *= brightness

	return color.NRGBA{
		R: linearToSrgb(lr),
		G: linearToSrgb(lg),
		B: linearToSrgb(lb),
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
		for i := range n {
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
		slices.Sort(xs)
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

// drawTriangleGouraud rasterizes a triangle with per-vertex colors (c0,c1,c2) using
// simple linear interpolation across scanlines. This is not perspective-correct
// but is sufficient for small scenes and orthographic-ish projection.
func drawTriangleGouraud(img *image.NRGBA, p0, p1, p2 [2]int, c0, c1, c2 color.NRGBA) {
	type pv struct {
		x, y int
		c    color.NRGBA
	}
	pts := []pv{{p0[0], p0[1], c0}, {p1[0], p1[1], c1}, {p2[0], p2[1], c2}}
	slices.SortFunc(pts, func(i, j pv) int { return i.y - j.y })

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

		xStart = max(xStart, img.Rect.Min.X)
		xEnd = min(img.Rect.Max.X-1, xEnd)

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

// phongIntensity computes Blinn-Phong intensity (0..1) for a point.
func phongIntensity(pos, n, viewPos [3]float64) float64 {
	ambientStrength := 0.1
	ks := 0.9
	shininess := 64.

	L := normalize([3]float64{LightSource[0] - pos[0], LightSource[1] - pos[1], LightSource[2] - pos[2]})
	V := normalize([3]float64{viewPos[0] - pos[0], viewPos[1] - pos[1], viewPos[2] - pos[2]})
	H := normalize([3]float64{(L[0] + V[0]), (L[1] + V[1]), (L[2] + V[2])})

	ndotl := dot(n, L)
	if ndotl < 0 {
		ndotl *= 0
	}
	diffuse := ndotl

	ndoth := dot(n, H)
	if ndoth < 0 {
		ndoth = 0
	}
	spec := ks * math.Pow(ndoth, shininess)

	d := distanceFromSource(pos[0], pos[1], pos[2])
	attenuation := 1.0 / (1.0 + 0.1*d + 0.01*d*d)
	intensity := ambientStrength + attenuation*(diffuse+spec)
	// intensity := ambientStrength + attenuation*(diffuse)

	// intensity := ambientStrength + (diffuse)
	if intensity < 0 {
		intensity *= 0
	}
	return intensity
}
func naiveIntensity(pos [3]float64) float64 {
	d := distanceFromSource(pos[0], pos[1], pos[2])
	intensity := (-(d - 4.45) / 6.)
	if intensity < .05 {
		intensity = .05
	}
	return max(intensity, 0.)
}

func drawTriangleNaive(img *image.NRGBA, p0, p1, p2 [2]int, pos0, pos1, pos2, n0, n1, n2 [3]float64, base color.NRGBA) {
	minX := max(min(p0[0], p1[0]), img.Rect.Min.X)
	maxX := min(max(p0[0], p1[0]), img.Rect.Max.X-1)
	if p2[0] < minX {
		minX = max(min(p2[0], minX), img.Rect.Min.X)
	}
	if p2[0] > maxX {
		maxX = min(max(p2[0], maxX), img.Rect.Max.X-1)
	}
	minY := max(min(p0[1], p1[1]), img.Rect.Min.Y)
	maxY := min(max(p0[1], p1[1]), img.Rect.Max.Y-1)
	if p2[1] < minY {
		minY = max(min(p2[1], minY), img.Rect.Min.Y)
	}
	if p2[1] > maxY {
		maxY = min(max(p2[1], maxY), img.Rect.Max.Y-1)
	}

	ax, ay := float64(p0[0]), float64(p0[1])
	bx, by := float64(p1[0]), float64(p1[1])
	cx, cy := float64(p2[0]), float64(p2[1])

	denom := (by-cy)*(ax-cx) + (cx-bx)*(ay-cy)
	if denom == 0 {
		return
	}

	for y := minY; y <= maxY; y++ {
		for x := minX; x <= maxX; x++ {
			fx, fy := float64(x), float64(y)
			w0 := ((by-cy)*(fx-cx) + (cx-bx)*(fy-cy)) / denom
			w1 := ((cy-ay)*(fx-cx) + (ax-cx)*(fy-cy)) / denom
			w2 := 1 - w0 - w1
			if w0 < 0 || w1 < 0 || w2 < 0 {
				continue
			}
			pos := [3]float64{
				pos0[0]*w0 + pos1[0]*w1 + pos2[0]*w2,
				pos0[1]*w0 + pos1[1]*w1 + pos2[1]*w2,
				pos0[2]*w0 + pos1[2]*w1 + pos2[2]*w2,
			}
			intensity := naiveIntensity(pos)
			col := shadeColorTrue(base, intensity)
			img.SetNRGBA(x, y, col)
		}
	}
}
func drawTrianglePhong(img *image.NRGBA, p0, p1, p2 [2]int, pos0, pos1, pos2, n0, n1, n2 [3]float64, base color.NRGBA) {
	minX := max(min(p0[0], p1[0]), img.Rect.Min.X)
	maxX := min(max(p0[0], p1[0]), img.Rect.Max.X-1)
	if p2[0] < minX {
		minX = max(min(p2[0], minX), img.Rect.Min.X)
	}
	if p2[0] > maxX {
		maxX = min(max(p2[0], maxX), img.Rect.Max.X-1)
	}
	minY := max(min(p0[1], p1[1]), img.Rect.Min.Y)
	maxY := min(max(p0[1], p1[1]), img.Rect.Max.Y-1)
	if p2[1] < minY {
		minY = max(min(p2[1], minY), img.Rect.Min.Y)
	}
	if p2[1] > maxY {
		maxY = min(max(p2[1], maxY), img.Rect.Max.Y-1)
	}

	ax, ay := float64(p0[0]), float64(p0[1])
	bx, by := float64(p1[0]), float64(p1[1])
	cx, cy := float64(p2[0]), float64(p2[1])

	denom := (by-cy)*(ax-cx) + (cx-bx)*(ay-cy)
	if denom == 0 {
		return
	}

	for y := minY; y <= maxY; y++ {
		for x := minX; x <= maxX; x++ {
			fx, fy := float64(x), float64(y)
			w0 := ((by-cy)*(fx-cx) + (cx-bx)*(fy-cy)) / denom
			w1 := ((cy-ay)*(fx-cx) + (ax-cx)*(fy-cy)) / denom
			w2 := 1 - w0 - w1
			if w0 < 0 || w1 < 0 || w2 < 0 {
				continue
			}

			// interpolate position and normal
			pos := [3]float64{
				pos0[0]*w0 + pos1[0]*w1 + pos2[0]*w2,
				pos0[1]*w0 + pos1[1]*w1 + pos2[1]*w2,
				pos0[2]*w0 + pos1[2]*w1 + pos2[2]*w2,
			}
			n := [3]float64{
				n0[0]*w0 + n1[0]*w1 + n2[0]*w2,
				n0[1]*w0 + n1[1]*w1 + n2[1]*w2,
				n0[2]*w0 + n1[2]*w1 + n2[2]*w2,
			}
			n = normalize(n)

			// intensity := naiveIntensity(pos)
			intensity := phongIntensity(pos, n, [3]float64{0, 0, 0})
			col := shadeColorTrue(base, intensity)
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

func drawImageAndSliders(
	ap *ansipixels.AnsiPixels,
	img *image.NRGBA,
	barWidth, barHeightImg, barHeightAp int,
	xSpeed, ySpeed, zSpeed float64,
) {
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
	ap.ClearScreen()
	dst := image.NewRGBA(img.Bounds())
	draw.Draw(dst, img.Bounds(), img, image.Point{}, draw.Src)
	_ = ap.ShowScaledImage(dst)
	ap.DrawRoundBox(0, ap.H-barHeightAp, barWidth+1, barHeightAp)
	ap.DrawRoundBox(barWidth+1, ap.H-barHeightAp, barWidth+1, barHeightAp)
	ap.DrawRoundBox((barWidth*2)+2, ap.H-barHeightAp, barWidth+1, barHeightAp)
}

type config struct {
	trueColor bool
	fps       float64
	border    bool
	color     bool
}

func configure() config {
	trueColorFlag := flag.Bool("truecolor", ansipixels.DetectColorMode().TrueColor,
		"use truecolor colors instead of the 216 color mode")
	fpsFlag := flag.Float64("fps", 60, "set the fps for the animation")
	wireFlag := flag.Bool("wire", false, "make the faces of the cube transparent and show a wireframe")
	borderFlag := flag.Bool("border", false, "show a white border around visible cube edges in color mode (does nothing without -color flag being used)") //nolint:lll // line is long because description is long
	flag.Parse()
	return config{
		trueColor: *trueColorFlag,
		color:     !*wireFlag,
		border:    *borderFlag,
		fps:       *fpsFlag,
	}
}

func main() { //nolint:gocognit,gocyclo,funlen,lll,maintidx // this handles the main drawing loop and handles input, it's going to get a little hairy
	c := configure()
	ap := ansipixels.NewAnsiPixels(c.fps)
	ap.TrueColor = c.trueColor
	ap.HideCursor()
	if ap.Open() != nil {
		return
	}
	errMessage := ""
	defer func() {
		ap.ShowCursor()
		ap.MouseTrackingOff()
		ap.MouseClickOff()
		ap.Restore()
		if len(errMessage) > 0 {
			fmt.Println(errMessage)
		}
	}()
	ap.MouseTrackingOn()
	xSpeed, ySpeed, zSpeed := .5, .85, .2
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

	// choose shading function depending on terminal color capability
	shadeFn := func(c color.NRGBA, b float64) color.NRGBA {
		if ap.TrueColor {
			return shadeColorTrue(c, b)
		}
		return shadeColor(c, b)
	}

	drawCube := func(pts [][2]int) {
		finfos := make([]faceInfo, 0, len(Faces))
		for i, f := range Faces {
			sum := 0.0
			for _, vi := range f {
				sum += vertices[vi][2]
			}
			finfos = append(finfos, faceInfo{i, sum / float64(len(f))})
		}
		slices.SortFunc(finfos, func(i, j faceInfo) int { return -int((i.avgZ - j.avgZ) * 100) })

		if ap.TrueColor {
			// per-pixel Phong: compute per-vertex normals and rasterize with per-pixel shading
			vnorms := computeVertexNormals()

			for _, fi := range finfos[len(finfos)-3:] {
				f := Faces[fi.idx]

				// triangle 1: f0, f1, f2
				drawTriangleNaive(img, pts[f[0]], pts[f[1]], pts[f[2]],
					vertices[f[0]], vertices[f[1]], vertices[f[2]],
					vnorms[f[0]], vnorms[f[1]], vnorms[f[2]],
					FaceColors[fi.idx])

				// triangle 2: f0, f2, f3
				drawTriangleNaive(img, pts[f[0]], pts[f[2]], pts[f[3]],
					vertices[f[0]], vertices[f[2]], vertices[f[3]],
					vnorms[f[0]], vnorms[f[2]], vnorms[f[3]],
					FaceColors[fi.idx])
			}
		} else {
			ambient := 0.25
			diffuse := 0.75
			for _, fi := range finfos {
				f := Faces[fi.idx]

				var c [4]color.NRGBA
				for i := range 4 {
					intensity := 1.6 / distanceFromSource(vertices[f[i]][0], vertices[f[i]][1], vertices[f[i]][2])
					c[i] = shadeFn(FaceColors[fi.idx], ambient+(diffuse*intensity))
				}

				// triangle 1: f0, f1, f2
				drawTriangleGouraud(img, pts[f[0]], pts[f[1]], pts[f[2]], c[0], c[1], c[2])
				// triangle 2: f0, f2, f3
				drawTriangleGouraud(img, pts[f[0]], pts[f[2]], pts[f[3]], c[0], c[2], c[3])
			}
		}
	}
	if !c.color {
		drawCube = func(pts [][2]int) {
			for _, e := range Edges {
				x0, y0 := pts[e[0]][0], pts[e[0]][1]
				x1, y1 := pts[e[1]][0], pts[e[1]][1]
				ansipixels.DrawLine(img, float64(x0), float64(y0), float64(x1), float64(y1), color.NRGBA{255, 255, 255, 255})
			}
		}
	}
	ap.SyncBackgroundColor()
	err := ap.FPSTicks(
		func() bool {
			clear(img.Pix)
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
			drawImageAndSliders(ap, img, barWidth, barHeightImg, barHeightAp, xSpeed, ySpeed, zSpeed)

			if len(ap.Data) > 0 && ap.Data[0] == 'q' {
				return false
			}
			ap.WriteAt(1, 1, "%v", LightSource)
			return true
		},
	)
	if err != nil {
		errMessage = err.Error()
	}
}
