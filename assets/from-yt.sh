#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

if [ -z "$1" ]; then
    echo "Usage: from-yt.sh youtube_url"
    echo "Example: from-yt.sh \"https://www.youtube.com/watch?v=dQw4w9WgXcQ\""
    exit 1
fi

YOUTUBE_URL="$1"
TEMP_PREFIX="yt_dl_$$"
TEMP_WAV="${TEMP_PREFIX}.wav"

cleanup_temp() {
    rm -f ${TEMP_PREFIX}.*
}

echo "Starting YouTube to RS485 conversion..."
echo "URL: $YOUTUBE_URL"
echo

echo "Step 1/4: Getting video title..."
TITLE=$(yt-dlp --skip-download --print "%(title)s" "$YOUTUBE_URL" 2>/dev/null | head -1)
SAFE_TITLE=$(echo "$TITLE" | sed 's/[^[:alnum:]._-]/_/g')
if [ -z "$SAFE_TITLE" ]; then
    SAFE_TITLE="yt_audio"
fi
PROCESSED_WAV="${SAFE_TITLE}_processed.wav"
echo "Title: $TITLE"

echo
echo "Step 2/4: Downloading audio..."
yt-dlp --cookies-from-browser chrome -x "$YOUTUBE_URL" -o "${TEMP_PREFIX}.%(ext)s"
if [ $? -ne 0 ]; then
    echo "Error downloading from YouTube"
    cleanup_temp
    exit 1
fi

DOWNLOADED=$(ls ${TEMP_PREFIX}.* 2>/dev/null | head -1)
if [ -z "$DOWNLOADED" ]; then
    echo "Error: Could not find downloaded file"
    cleanup_temp
    exit 1
fi

echo
echo "Step 3/4: Converting and encoding for playback..."
if [ "${DOWNLOADED##*.}" != "wav" ]; then
    ffmpeg -i "$DOWNLOADED" -y "$TEMP_WAV"
    if [ $? -ne 0 ]; then
        echo "Error converting to WAV"
        cleanup_temp
        exit 1
    fi
    rm "$DOWNLOADED"
else
    mv "$DOWNLOADED" "$TEMP_WAV"
fi

"$SCRIPT_DIR/process.sh" "$TEMP_WAV"
if [ $? -ne 0 ]; then
    echo "Error processing WAV file"
    cleanup_temp
    exit 1
fi

mv "${TEMP_PREFIX}_processed.wav" "$PROCESSED_WAV"
# rm -f "$TEMP_WAV"
echo "Saved processed audio as: $PROCESSED_WAV"

echo
echo "Step 4/4: Running dartt_wav_player..."
./dartt_wav_player "$PROCESSED_WAV" 0 72 --baudrate 921600
if [ $? -ne 0 ]; then
    echo "Error running dartt_wav_player"
    exit 1
fi

echo
echo "Done."
