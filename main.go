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
	// z := v[0]*math.Sin(0) + v[2]*math.Cos(0)
	x := v[0]*cosa + v[1]*sina
	return [3]float64{x, y, v[2]}
}

func project(v [3]float64, width, height int) (int, int) {
	distance := 3.0
	scale := float64(width) / 5.0
	x := v[0] / (v[2] + distance) * scale
	y := v[1] / (v[2] + distance) * scale

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

	i := 1
	img := image.NewNRGBA(image.Rect(0, 0, width, height*2))
	prevMousePosition := [2]int{}
	ap.FPSTicks(context.Background(), func(ctx context.Context) bool {
		clear(img.Pix)
		barWidth := img.Bounds().Dx() / 20
		barHeightImg := img.Bounds().Dy() / 3
		barHeightAp := ap.H / 3
		angle := float64(i) * 2 * math.Pi / float64(frames)
		// i = (i + 1) //% 360
		pts := make([][2]int, len(vertices))
		for j, v := range vertices {
			rv := rotateX(v, angle*xSpeed)
			rv = rotateY(rv, angle*ySpeed)
			rv = rotateZ(rv, angle*zSpeed)
			pts[j][0], pts[j][1] = project(rv, width, height)
			vertices[j] = rv
		}
		if ap.LeftClick() || ap.LeftDrag() {
			switch {
			case ap.Mx > 0 && ap.Mx < barWidth+1:
				if ap.My >= ap.H-barHeightAp {
					xSpeed = float64(ap.H-ap.My) / float64(barHeightAp)
				}
			case ap.Mx > barWidth+1 && ap.Mx < barWidth*2+1:
				if ap.My >= ap.H-barHeightAp {
					ySpeed = float64(ap.H-ap.My) / float64(barHeightAp)
				}
			case ap.Mx > barWidth*2+1 && ap.Mx < barWidth*3+1:
				if ap.My >= ap.H-barHeightAp {
					zSpeed = float64(ap.H-ap.My) / float64(barHeightAp)
				}
			default:
				xSpeed, ySpeed, zSpeed = 0, 0, 0
				if ap.LeftDrag() {
					if prevMousePosition[0] != ap.Mx {
						for j, v := range vertices {
							rv := rotateY(v, -angle*.7*(float64(ap.Mx-(prevMousePosition[0]))))
							pts[j][0], pts[j][1] = project(rv, width, height)
							vertices[j] = rv
						}
					}
					if prevMousePosition[0] != ap.My {
						for j, v := range vertices {
							rv := rotateX(v, angle*.7*(float64(ap.My-(prevMousePosition[1]))))
							pts[j][0], pts[j][1] = project(rv, width, height)
							vertices[j] = rv
						}
					}
				}
			}
			prevMousePosition = [2]int{ap.Mx, ap.My}
		}
		for _, e := range edges {
			x0, y0 := pts[e[0]][0], pts[e[0]][1]
			x1, y1 := pts[e[1]][0], pts[e[1]][1]
			ansipixels.DrawLine(img, float64(x0), float64(y0), float64(x1), float64(y1), color.NRGBA{255, 255, 255, 255})
		}
		draw.Draw(img, image.Rect(1, img.Bounds().Dy()-(int(xSpeed*float64(barHeightImg)))-2, barWidth, img.Bounds().Dy()-2), &image.Uniform{color.RGBA{255, 0, 255, 255}}, image.Point{}, draw.Over)
		newBarWidth := barWidth * 2
		draw.Draw(img, image.Rect(barWidth+2, img.Bounds().Dy()-(int(ySpeed*float64(barHeightImg)))-2, newBarWidth+1, img.Bounds().Dy()-2), &image.Uniform{color.RGBA{255, 0, 255, 255}}, image.Point{}, draw.Over)
		draw.Draw(img, image.Rect(newBarWidth+3, img.Bounds().Dy()-(int(zSpeed*float64(barHeightImg)))-2, newBarWidth+barWidth+2, img.Bounds().Dy()-2), &image.Uniform{color.RGBA{255, 0, 255, 255}}, image.Point{}, draw.Over)
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
		ap.DrawRoundBox(0, ap.H-barHeightAp, barWidth+1, barHeightAp)
		ap.DrawRoundBox(barWidth+1, ap.H-barHeightAp, barWidth+1, barHeightAp)
		ap.DrawRoundBox(newBarWidth+2, ap.H-barHeightAp, barWidth+1, barHeightAp)
		ap.EndSyncMode()
		if len(ap.Data) > 0 && ap.Data[0] == 'q' {
			// return
			return false
		}
		return true
	})

}
