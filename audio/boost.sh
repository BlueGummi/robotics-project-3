#!/bin/bash

DIR="${1:-.}"
BOOST_DB=11

for FILE in "$DIR"/*.wav; do
    [ -e "$FILE" ] || continue

    TMP_FILE="$(dirname "$FILE")/.$(basename "$FILE").tmp.wav"

    ffmpeg -y -i "$FILE" -filter:a "volume=${BOOST_DB}dB" "$TMP_FILE"

    if [ $? -eq 0 ]; then
        mv "$TMP_FILE" "$FILE"
    else
        rm -f "$TMP_FILE"
    fi
done


