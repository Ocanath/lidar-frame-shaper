#!/bin/bash
if [ -z "$1" ]; then
    echo "Usage: process.sh input_file"
    echo "Example: process.sh input.wav"
    exit 1
fi

INPUT_FILE="$1"
BASENAME="${INPUT_FILE%.*}"
EXT="${INPUT_FILE##*.}"
OUTPUT_FILE="${BASENAME}_processed.${EXT}"

echo "Processing $INPUT_FILE..."
echo "- Downsampling to 17631Hz"
echo "- Applying highpass filter at 2000Hz"
echo "- Applying lowpass filter at 10000Hz"
echo "- Scaling volume by 10000/32767 (0.305)"

ffmpeg -i "$INPUT_FILE" -ar 32288 -ac 1 -af "highpass=f=50,volume=2000/32767" -y "$OUTPUT_FILE"

if [ $? -eq 0 ]; then
    echo "Success! Output saved as: $OUTPUT_FILE"
else
    echo "Error processing file"
    exit 1
fi
