#!/bin/bash
for file in *.mp3; do
    ffmpeg -i "$file" "${file%.mp3}.wav" && rm "$file";
done

shopt -s nullglob

for f in *.wav; do
    tmp="tmp_$f"
    echo "Processing '$f'..."

    ffmpeg -y -i "$f" \
        -ac 1 \
        -ar 16000 \
        -sample_fmt s16 \
        -map 0:a \
        -map_metadata -1 \
        "$tmp"

    if [ $? -eq 0 ]; then
        mv "$tmp" "$f"
        echo "Converted '$f' successfully."
    else
        echo "Failed to convert '$f'. Skipping."
        rm -f "$tmp"
    fi
done

