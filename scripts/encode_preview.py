#!/usr/bin/env python3
"""Encode the SFML demo's --capture-dir PNG frames as a 30 fps GIF."""
import argparse
from pathlib import Path
from PIL import Image
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('frames',type=Path)
parser.add_argument('output',type=Path)
args=parser.parse_args()
frames=[]
for path in sorted(args.frames.glob('frame-*.png')):
    with Image.open(path) as image:
        frames.append(image.resize((826,504)).convert('P',palette=Image.Palette.ADAPTIVE))
if not frames:
    parser.error('no frame PNGs found')
args.output.parent.mkdir(parents=True,exist_ok=True)
frames[0].save(args.output,save_all=True,append_images=frames[1:],duration=[(33,33,34)[i % 3] for i in range(len(frames))],loop=0,optimize=True)
