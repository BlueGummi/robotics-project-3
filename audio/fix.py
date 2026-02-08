import os
import wave
from pathlib import Path

input_folder = Path(".")
output_folder = Path("clean_wavs")
output_folder.mkdir(exist_ok=True)

for wav_file in input_folder.glob("*.wav"):
    with wave.open(str(wav_file), "rb") as old_wav:
        params = old_wav.getparams()
        frames = old_wav.readframes(old_wav.getnframes())

    out_file = output_folder / wav_file.name
    with wave.open(str(out_file), "wb") as new_wav:
        new_wav.setnchannels(1)
        new_wav.setsampwidth(2)
        new_wav.setframerate(16000)
        new_wav.writeframes(frames)

