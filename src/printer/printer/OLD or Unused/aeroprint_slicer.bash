#!/bin/bash

<<comment
Script adopted from u/martinkoistinen on Reddit "Batch processing with PrusaSlicer works like a dream"
https://www.reddit.com/r/prusa3d/comments/ji7v5f/batch_processing_with_prusaslicer_works_like_a/
comment

## Slicer File Path
SLICER='/snap/bin/prusa-slicer'

## Process .stl into .gcode
for file in ./*.stl; do
    echo "Processing file: $file"
    "$SLICER" \
        --load ./slicer_config.ini \
        --printer-technology FFF \
        # --scale-to-fit 76.2,76.2,76.2  #Scale to a 3"x3"x3" volume
        --center 105,105 \
        --slice \
        --export-gcode \
        "$file"
done
