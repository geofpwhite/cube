package main

import (
	"image/color"
	"math"
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

var Edges = [12][2]int{
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

var Faces = [6][4]int{
	{0, 1, 2, 3}, // back  (z = -1)
	{0, 1, 5, 4}, // left  (x = -1)
	{3, 2, 6, 7}, // right (x = +1)
	{1, 2, 6, 5}, // top   (y = +1)
	{0, 3, 7, 4}, // bottom(y = -1)
	{4, 5, 6, 7}, // front (z = +1)
}

var FaceColors = [6]color.NRGBA{
	{200, 50, 50, 255},  // back - red
	{50, 200, 50, 255},  // front - green
	{50, 50, 200, 255},  // left - blue
	{200, 200, 50, 255}, // right - yellow
	{200, 50, 200, 255}, // top - magenta
	{50, 200, 200, 255}, // bottom - cyan
}

var LightSource = [3]float64{4, -2, 0}

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
	for _, f := range Faces {
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
			for _, f := range Faces {
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
func distanceFromSource(x, y, z float64) float64 {
	x = max(x-LightSource[0], LightSource[0]-x)
	y = max(y-LightSource[1], LightSource[1]-y)
	z = max(z-LightSource[2], LightSource[2]-z)
	return math.Sqrt((x * x) + (y * y) + (z * z))
}
