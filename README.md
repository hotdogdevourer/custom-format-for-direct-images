# Overview
- Can convert PNM (PPM/PGM) from/to CFDE container
- Handles bit-packing logic for 1-bit, 2-bit, 4-bit to 32-bit (even)
- Has built-in compression algorithms.
- I devourerd a hotdog

# Compilation
It works with any C89 compiler.

`cc -pedantic -std=c89 -o cfde cfde.c`

# Usaeg
`main create output.cfde --width=100 --height=50`

# Help text
```
CFDE Image Format Tool

Usage:
  main convert <input.pnm> <output.cfde> [--bpp=N] [--comp=N]
      Convert PPM/PGM to CFDE

  main decode <input.cfde> <output.pnm>
      Decode CFDE to PPM/PGM

  main raw <input.bin> <output.cfde> --width=N --height=N
          [--color-type=N] [--bpp=N] [--comp=N]
      Import raw binary as CFDE (--color-type: 0=gray,2=RGB,4=gray+alpha,6=RGBA)

  main create <output.cfde> --width=N --height=N
          [--type=gradient|single-colour] [--rgb=RRGGBB]
          [--color-type=N] [--bpp=N] [--comp=N]
      Create a synthetic CFDE image

  main view <input.cfde>
      View CFDE image in terminal (requires one rendering flag)
      Render Flags: --full-color  (24-bit ANSI true color)
                    --ansi-2      (2-color monochrome B/W)
                    --ansi-8      (8-color standard palette)
                    --ansi-16     (16-color VGA palette)
                    --ansi-256    (256-color xterm palette)
                    --grayscale   (Classic ASCII art ramp)
      View Flags:   --half-view  (1:1 pixels, skip every other row for terminal aspect)
                    --raw-view       (1 char per pixel, all rows, stretched due to terminal limits)
                    --full-view     (2 chars wide per pixel, all rows, correct aspect)
      Flush Flags:  --buffer      (Full RAM buffer, single fwrite)
                    --flush-line  (Line buffer, fwrite per row)
                    --flush       (printf per pixel)

  main info <input.cfde>
      Print CFDE header info

Options:
  --bpp=N         Bits per sample (1,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32)
  --comp=N        Compression level (0=none..5=hyper)
  --color-type=N  0=Gray, 2=RGB, 3=Indexed, 4=Gray+Alpha, 6=RGBA
  --width=N       Image width
  --height=N      Image height
  --rgb=RRGGBB    Hex color for single-colour create
  --type=T        gradient or single-colour
```

# License
MIT License. More licensing information in the LICENSE file in the repository's root directory
