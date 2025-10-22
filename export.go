package main

import (
	"image"
	"image/color"
	"image/gif"
	"os"
)

// func exportToGif(images []image.NRGBA, fps float64) error {

// 	delay := int(100 / fps) // Convert fps to centiseconds delay
// 	if delay < 1 {
// 		delay = 1
// 	}

// 	// Create a smaller palette focused on the face colors and their shades
// 	palette := color.Palette{color.Black} // Start with black

// 	// Add face colors and some variations
// 	for _, c := range FaceColors {
// 		palette = append(palette, c)
// 		// Add darker and lighter variations
// 		r, g, b, a := c.RGBA()
// 		r8, g8, b8 := uint8(r>>8), uint8(g>>8), uint8(b>>8)
// 		// Darker shade
// 		palette = append(palette, color.NRGBA{r8 / 2, g8 / 2, b8 / 2, uint8(a >> 8)})
// 		// Lighter shade
// 		palette = append(palette, color.NRGBA{
// 			uint8(min(255, int(r8)*3/2)),
// 			uint8(min(255, int(g8)*3/2)),
// 			uint8(min(255, int(b8)*3/2)),
// 			uint8(a >> 8),
// 		})
// 	}

// 	// Add some grayscale values for shading
// 	for i := 0; i < 32; i++ {
// 		gray := uint8(i * 8)
// 		palette = append(palette, color.NRGBA{gray, gray, gray, 255})
// 	}

// 	outGif := &gif.GIF{
// 		LoopCount: 0, // 0 means loop forever
// 	}

// 	// Convert each NRGBA frame to paletted
// 	bounds := images[0].Bounds()
// 	for _, img := range images {
// 		palettedImage := image.NewPaletted(bounds, palette)

// 		// Convert each pixel individually for better color matching
// 		for y := bounds.Min.Y; y < bounds.Max.Y; y++ {
// 			for x := bounds.Min.X; x < bounds.Max.X; x++ {
// 				palettedImage.Set(x, y, img.NRGBAAt(x, y))
// 			}
// 		}

// 		outGif.Image = append(outGif.Image, palettedImage)
// 		outGif.Delay = append(outGif.Delay, delay)
// 	}

// 	// Open a file to save the GIF
// 	f, err := os.Create("cube.gif")
// 	if err != nil {
// 		return err
// 	}
// 	defer f.Close()

//		// Encode the GIF animation
//		return gif.EncodeAll(f, outGif)
//	}
func exportToGif(images []image.NRGBA, fps float64) error {

	delay := int(100 / fps) // Convert fps to centiseconds delay
	if delay < 1 {
		delay = 1
	}

	// Create a smaller palette focused on the face colors and their shades
	palette := color.Palette{color.Black} // Start with black

	// Add face colors and some variations
	for _, c := range FaceColors {
		palette = append(palette, c)
		// Add darker and lighter variations
		r, g, b, a := c.RGBA()
		r8, g8, b8 := uint8(r>>8), uint8(g>>8), uint8(b>>8)
		// Darker shade
		palette = append(palette, color.NRGBA{r8 / 2, g8 / 2, b8 / 2, uint8(a >> 8)})
		// Lighter shade
		palette = append(palette, color.NRGBA{
			uint8(min(255, int(r8)*3/2)),
			uint8(min(255, int(g8)*3/2)),
			uint8(min(255, int(b8)*3/2)),
			uint8(a >> 8),
		})
	}

	// Add some grayscale values for shading
	for i := 0; i < 32; i++ {
		gray := uint8(i * 8)
		palette = append(palette, color.NRGBA{gray, gray, gray, 255})
	}

	outGif := &gif.GIF{
		LoopCount: 0, // 0 means loop forever
	}

	// Convert each NRGBA frame to paletted
	bounds := images[0].Bounds()
	for _, img := range images {
		palettedImage := image.NewPaletted(bounds, palette)

		// Convert each pixel individually for better color matching
		for y := bounds.Min.Y; y < bounds.Max.Y; y++ {
			for x := bounds.Min.X; x < bounds.Max.X; x++ {
				palettedImage.Set(x, y, img.NRGBAAt(x, y))
			}
		}

		outGif.Image = append(outGif.Image, palettedImage)
		outGif.Delay = append(outGif.Delay, delay)
	}

	// Open a file to save the GIF
	f, err := os.Create("cube.gif")
	if err != nil {
		return err
	}
	defer f.Close()

	// Encode the GIF animation
	return gif.EncodeAll(f, outGif)
}
