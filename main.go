package main

import (
	"context"
	"image"
	"image/color"
	"image/draw"
	"math"

	"fortio.org/terminal/ansipixels"
)

// Cube definition: 8 vertices
var vertices = [8][3]float64{
	{-1, -1, -1},
	{-1, +1, -1},
	{+1, +1, -1},
	{+1, -1, -1},
	{-1, -1, +1},
	{-1, +1, +1},
	{+1, +1, +1},
	{+1, -1, +1},
}

// Edges: each pair indexes into the vertices array
var edges = [12][2]int{
	{0, 1}, {1, 2}, {2, 3}, {3, 0},
	{4, 5}, {5, 6}, {6, 7}, {7, 4},
	{0, 4}, {1, 5}, {2, 6}, {3, 7},
}

// Line drawing: Bresenham’s algorithm
func drawLine(img *image.Paletted, colIndex uint8, x0, y0, x1, y1 int) {
	dx := abs(x1 - x0)
	dy := abs(y1 - y0)
	sx, sy := 1, 1
	if x0 > x1 {
		sx = -1
	}
	if y0 > y1 {
		sy = -1
	}

	err := dx - dy
	for {
		img.SetColorIndex(x0, y0, colIndex)
		if x0 == x1 && y0 == y1 {
			break
		}
		e2 := err
		if e2 > -dy {
			err -= dy
			x0 += sx
		}
		if e2 < dx {
			err += dx
			y0 += sy
		}
	}
}

func abs(a int) int {
	if a < 0 {
		return -a
	}
	return a
}

// Rotate a point around X and Y axes
func rotateY(v [3]float64, ay float64) [3]float64 {

	cosa, sina := math.Cos(ay), math.Sin(ay)
	x := v[0]*cosa + v[2]*sina
	z := -v[0]*sina + v[2]*cosa

	return [3]float64{x, v[1], z}
}
func rotateX(v [3]float64, ax float64) [3]float64 {
	// Rotate around X
	cosa, sina := math.Cos(ax), math.Sin(ax)
	y := v[1]*cosa - v[2]*sina
	z := v[1]*sina + v[2]*cosa

	return [3]float64{v[0], y, z}
}
func rotateZ(v [3]float64, az float64) [3]float64 {
	// Rotate around Z
	cosa, sina := math.Cos(az), math.Sin(az)
	y := v[1]*cosa - v[0]*sina
	// z := v[0]*math.Sin(0) + v[2]*math.Cos(0)
	x := v[0]*cosa + v[1]*sina
	return [3]float64{x, y, v[2]}
}

// Project 3D point to 2D screen coords
func project(v [3]float64, width, height int) (int, int) {
	// Simple perspective projection
	distance := 3.0
	scale := float64(width) / 5.0
	x := v[0] / (v[2] + distance) * scale
	y := v[1] / (v[2] + distance) * scale

	// Translate to center
	xc := int(x) + width/2
	yc := int(y) + height
	return xc, yc
}

func main() {
	ap := ansipixels.NewAnsiPixels(60)
	ap.HideCursor()
	if ap.Open() != nil {
		return
	}
	defer func() {
		ap.ShowCursor()
		ap.MouseTrackingOff()
		ap.MouseClickOff()
		ap.Restore()
	}()
	ap.MouseTrackingOn()
	ap.ClearScreen()
	xSpeed, ySpeed, zSpeed := .5, .5, .5
	var (
		width, height = ap.W, ap.H
		frames        = 60
	)

	i := 0
	img := image.NewNRGBA(image.Rect(0, 0, width, height*2))
	ap.FPSTicks(context.Background(), func(ctx context.Context) bool {
		clear(img.Pix)
		barWidth := img.Bounds().Dx() / 20
		angle := float64(i) * 2 * math.Pi / float64(frames)
		i = (i + 1) % 360
		pts := make([][2]int, len(vertices))
		for j, v := range vertices {
			rv := rotateX(v, angle*xSpeed)
			rv = rotateY(rv, angle*ySpeed)
			rv = rotateZ(rv, angle*zSpeed)
			pts[j][0], pts[j][1] = project(rv, width, height)
		}
		if ap.LeftClick() || ap.LeftDrag() {
			switch {
			case ap.Mx > 0 && ap.Mx < barWidth+1:
				if ap.My >= ap.H-10 {
					xSpeed = float64(ap.H-ap.My) / 10
				}
			case ap.Mx > barWidth+1 && ap.Mx < barWidth*2+1:
				if ap.My >= ap.H-10 {
					ySpeed = float64(ap.H-ap.My) / 10
				}
			case ap.Mx > barWidth*2+1 && ap.Mx < barWidth*3+1:
				if ap.My >= ap.H-10 {
					zSpeed = float64(ap.H-ap.My) / 10
				}
			}
		}
		for _, e := range edges {
			x0, y0 := pts[e[0]][0], pts[e[0]][1]
			x1, y1 := pts[e[1]][0], pts[e[1]][1]
			ansipixels.DrawLine(img, float64(x0), float64(y0), float64(x1), float64(y1), color.NRGBA{255, 255, 255, 255})
		}
		draw.Draw(img, image.Rect(1, img.Bounds().Dy()-(2*int(xSpeed*10))-2, barWidth, img.Bounds().Dy()-2), &image.Uniform{color.RGBA{255, 0, 255, 255}}, image.Point{}, draw.Over)
		newBarWidth := barWidth * 2
		draw.Draw(img, image.Rect(barWidth+2, img.Bounds().Dy()-(2*int(ySpeed*10))-2, newBarWidth+1, img.Bounds().Dy()-2), &image.Uniform{color.RGBA{255, 0, 255, 255}}, image.Point{}, draw.Over)
		draw.Draw(img, image.Rect(newBarWidth+3, img.Bounds().Dy()-(2*int(zSpeed*10))-2, newBarWidth+barWidth+2, img.Bounds().Dy()-2), &image.Uniform{color.RGBA{255, 0, 255, 255}}, image.Point{}, draw.Over)
		ap.StartSyncMode()

		if ap.ColorOutput.TrueColor {
			err := ap.DrawTrueColorImage(0, 0, &image.RGBA{Pix: img.Pix, Stride: img.Stride, Rect: img.Rect})
			if err != nil {
				return false
			}
		} else {
			err := ap.Draw216ColorImage(0, 0, &image.RGBA{Pix: img.Pix, Stride: img.Stride, Rect: img.Rect})
			if err != nil {
				return false
			}
		}
		ap.DrawRoundBox(0, ap.H-10, barWidth+1, 10)
		ap.DrawRoundBox(barWidth+1, ap.H-10, barWidth+1, 10)
		ap.DrawRoundBox(newBarWidth+2, ap.H-10, barWidth+1, 10)
		ap.EndSyncMode()
		if len(ap.Data) > 0 && ap.Data[0] == 'q' {
			// return
			return false
		}
		return true
	})

}
