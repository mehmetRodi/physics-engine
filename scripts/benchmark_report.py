#!/usr/bin/env python3
"""Collect sequential independent runs, retain raw samples, and plot run medians.

python scripts/benchmark_report.py --build build-release --output docs/performance --repeats 5
Requires requirements-report.txt only for plotting. No network or system tuning is performed.
"""
import argparse
import csv
import datetime
import hashlib
import io
import json
import os
from pathlib import Path
import platform
import statistics
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

def capture(args):
    return subprocess.check_output(args, text=True, stderr=subprocess.STDOUT).strip()

def optional(args):
    try:
        return capture(args)
    except (OSError, subprocess.CalledProcessError):
        return "unavailable"

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, default=Path('build-release'))
    parser.add_argument('--output', type=Path, default=Path('docs/performance'))
    parser.add_argument('--repeats', type=int, default=5)
    args = parser.parse_args()
    if args.repeats < 3:
        parser.error('use at least three independent process runs')
    build = args.build.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    # All cases get identical warmup/sample counts; alternating the outer case order
    # reduces systematic order effects. Measurements are strictly sequential.
    cases = [(f'warm_{i}', 352, 1, i, 1) for i in (1, 8, 16)]
    cases += [(f'cold_{i}', 352, 1, i, 0) for i in (1, 16)]
    cases += [(f'threads_{t}', 4092, t, 8, 1) for t in (1, 2, 4, 8)]
    commands = []
    rows = []
    for repeat in range(args.repeats):
        order = cases if repeat % 2 == 0 else list(reversed(cases))
        for name, bodies, threads, iterations, warm in order:
            cmd = [str(build/'world_step_bench'), '--bodies', str(bodies), '--threads', str(threads),
                   '--iterations', str(iterations), '--warm-start', str(warm),
                   '--warmup', '300', '--samples', '600', '--scene', 'columns', '--csv']
            print(f'run {repeat+1}/{args.repeats}: {name}', flush=True)
            commands.append(cmd)
            for row in csv.DictReader(io.StringIO(capture(cmd))):
                rows.append({'case': name, 'run': repeat, **row})
    with (output/'world_samples.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    sweep = []
    for repeat in range(args.repeats):
        print(f'sweep run {repeat+1}/{args.repeats}', flush=True)
        cmd = [str(build/'sweep_mode_bench'), '--csv']; commands.append(cmd)
        sweep.extend({'run': repeat, **row} for row in csv.DictReader(io.StringIO(capture(cmd))))
    with (output/'sweep_samples.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(sweep[0]))
        writer.writeheader(); writer.writerows(sweep)
    cmd = [str(build/'collision_pipeline_bench'), '--samples', '600', '--warmup', '300']
    commands.append(cmd)
    (output/'stage_samples.csv').write_text(capture(cmd)+'\n')
    # Hash source inputs so results remain identifiable before the first Git commit.
    source_files = sorted([p for folder in ('physics','collision','math','core','bench')
                           for p in (ROOT/folder).rglob('*') if p.suffix in ('.cpp','.hpp')]
                          + [ROOT/'CMakeLists.txt'])
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files}
    cpu = optional(['sysctl','-n','machdep.cpu.brand_string']) if sys.platform=='darwin' else optional(['lscpu'])
    metadata = {'recorded_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                'platform': platform.platform(), 'machine': platform.machine(), 'cpu': cpu,
                'logical_cpus': os.cpu_count(), 'compiler': optional(['c++','--version']),
                'cmake': optional(['cmake','--version']), 'repeats': args.repeats,
                'pinning': 'none', 'frequency_control': 'none',
                'method': 'sequential processes; world case order reversed on alternating repeats; sweep order fixed',
                'commands': [[str(Path(c[0]).name), *c[1:]] for c in commands], 'source_sha256': hashes}
    metadata['cmake_cache'] = [line for line in (build/'CMakeCache.txt').read_text().splitlines()
                               if line.startswith(('CMAKE_BUILD_TYPE:', 'CMAKE_CXX_FLAGS:', 'CMAKE_CXX_FLAGS_RELEASE:'))]
    (output/'environment.json').write_text(json.dumps(metadata, indent=2)+'\n')
    summaries = []
    for name, *_ in cases:
        selected = [r for r in rows if r['case']==name]
        medians = [statistics.median(int(r['ns']) for r in selected if int(r['run'])==i)
                   for i in range(args.repeats)]
        tails = []
        for i in range(args.repeats):
            values=sorted(int(r['ns']) for r in selected if int(r['run'])==i)
            tails.append(values[int(.99*(len(values)-1))])
        summaries.append({'case': name, 'median_ns': statistics.median(medians),
                          'min_run_median_ns': min(medians), 'max_run_median_ns': max(medians),
                          'median_run_p99_ns': statistics.median(tails),
                          'final_contacts': selected[-1]['contacts'],
                          'final_compression_m': selected[-1]['compression_m'],
                          'reserved_bytes': selected[-1]['reserved_bytes']})
    with (output/'summary.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(summaries[0]));writer.writeheader();writer.writerows(summaries)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes=plt.subplots(1,2,figsize=(11,4.2),layout='constrained')
    for ax, group, title in [(axes[0],summaries[:5],'352 bodies: solver settings'),
                             (axes[1],summaries[5:],'4,092 bodies: full-tick thread scaling')]:
        values=[r['median_ns']/1000 for r in group]
        lower=[(r['median_ns']-r['min_run_median_ns'])/1000 for r in group]
        upper=[(r['max_run_median_ns']-r['median_ns'])/1000 for r in group]
        ax.bar([r['case'].replace('_',' ') for r in group],values,yerr=[lower,upper],capsize=4,color='#236b9b')
        ax.set_title(title);ax.set_ylabel('microseconds / tick (lower is better)');ax.tick_params(axis='x',rotation=25)
    fig.suptitle(f'Median of {args.repeats} process medians; whiskers show min–max run medians')
    fig.savefig(output/'world-performance.png',dpi=160)
    plt.close(fig)
    distributions=['sparse','dense','all_overlapping']
    modes=['scalar_indirect','scalar_packed','simd']
    fig,ax=plt.subplots(figsize=(8,4.2),layout='constrained')
    for offset,mode in enumerate(modes):
        values=[]
        for distribution in distributions:
            per_run=[statistics.median(int(r['ns']) for r in sweep if r['mode']==mode and r['distribution']==distribution and int(r['run'])==i) for i in range(args.repeats)]
            values.append(statistics.median(per_run)/1000)
        ax.bar([i+(offset-1)*.25 for i in range(3)],values,width=.25,label=mode)
    ax.set_xticks(range(3),distributions);ax.set_yscale('log');ax.set_ylabel('microseconds / sweep (log scale)')
    ax.set_title('1,024 AABBs: layout and explicit SIMD');ax.legend()
    fig.savefig(output/'sweep-performance.png',dpi=160);plt.close(fig)
    print(f'Report artifacts: {output}')

if __name__=='__main__':
    main()
