#!/usr/bin/env python3
"""Sample this project's serial benchmark on macOS; keep profiling separate from timings."""
import argparse
from pathlib import Path
import subprocess
import sys
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--build',type=Path,default=Path('build-release'))
parser.add_argument('--output',type=Path,default=Path('docs/performance/profile.txt'))
args=parser.parse_args()
if sys.platform!='darwin':
    parser.error('macOS sample required; use perf record on Linux')
args.output.parent.mkdir(parents=True,exist_ok=True)
command=[str((args.build/'world_step_bench').resolve()),'--bodies','4092','--threads','1',
         '--warmup','300','--samples','10000','--scene','columns']
with subprocess.Popen(command,stdout=subprocess.DEVNULL) as child:
    try:
        subprocess.run(['/usr/bin/sample',str(child.pid),'3','1','-file',str(args.output.resolve())],check=True)
    finally:
        if child.poll() is None:
            child.terminate()
        child.wait()
root=Path(__file__).resolve().parents[1]
text=args.output.read_text().replace(str(root),'<repo>').replace(str(Path.home()),'~')
args.output.write_text(text)
