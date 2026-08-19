"""Sample GPU/RAM while a command runs, so "the machine is too small" becomes a number.

Written for a purchase decision: this laptop has 8,151 MiB of VRAM (RTX 5070 Laptop) and every
candidate replacement has 16 GB. Doubling only helps if the pen run is actually near the ceiling,
so measure the ceiling rather than assume it. It doubles as the resource half of the acceptance
baseline the new machine has to reproduce, which is why it is Python and not PowerShell -- the
replacement is meant to run Ubuntu, and a measurement that cannot be repeated there is worthless
for comparison.

Wraps the command instead of running beside it. Two reasons: the logged interval then matches the
run exactly with nothing to start and stop by hand, and the child's pid is known, which is the
only way to ask nvidia-smi for per-process memory.

  python tools/log_resources.py --out run1.csv -- python main.py --use_cached_query

stdout and stderr are inherited, not captured, so pipe the whole thing to tee if you want the
run log as well.

DO NOT wrap the acceptance-baseline runs with this. Use it to profile resources; run
`python main.py --use_cached_query` bare when the point is to judge whether the port works. A
tool that exists to measure the machine must not be in the loop of the measurement it is
justifying -- when the pen task started misbehaving, three runs went into establishing that this
wrapper was not the cause, and the answer only became solid once it was removed entirely rather
than reconfigured. See chapters 16.4 and 17.6 of docs/behavior1k_rekep_setup.md.

On per-process memory: nvidia-smi reports "[N/A]" for it whenever the GPU is in WDDM mode, which
is every consumer Windows machine including this one. That is why the idle baseline below is not
decoration -- when the per-process number is missing, (total - baseline) is the only estimate of
what the run itself is holding, so the baseline is load-bearing and gets recorded either way.
"""
import argparse
import csv
import os
import subprocess
import sys
import threading
import time

try:
    import psutil
except ImportError:  # never let the logger be the reason a measurement run dies
    psutil = None

NVSMI = 'nvidia-smi'
FIELDS = ['t_rel_s', 'phase', 'gpu_mem_used_mib', 'gpu_mem_total_mib',
          'gpu_util_pct', 'proc_gpu_mem_mib', 'sys_ram_used_gb',
          'cpu_pct_total', 'cpu_pct_busiest_core', 'proc_cpu_pct']


def _run(args):
    try:
        out = subprocess.run([NVSMI] + args, capture_output=True, text=True, timeout=10)
        return out.stdout.strip() if out.returncode == 0 else ''
    except (OSError, subprocess.TimeoutExpired):
        return ''


def sample_gpu():
    """(mem_used_mib, mem_total_mib, util_pct) for GPU 0, or (None, None, None)."""
    line = _run(['--query-gpu=memory.used,memory.total,utilization.gpu',
                 '--format=csv,noheader,nounits', '-i', '0']).splitlines()
    if not line:
        return None, None, None
    try:
        used, total, util = (int(v.strip()) for v in line[0].split(','))
        return used, total, util
    except ValueError:
        return None, None, None


def sample_proc(pids):
    """MiB held by any of `pids`, or None when the driver will not say (WDDM)."""
    text = _run(['--query-compute-apps=pid,used_gpu_memory', '--format=csv,noheader,nounits'])
    if not text:
        return None
    total, seen = 0, False
    for row in text.splitlines():
        parts = [p.strip() for p in row.split(',')]
        if len(parts) != 2:
            continue
        try:
            pid, mib = int(parts[0]), int(parts[1])
        except ValueError:
            continue  # "[N/A]" -- WDDM
        if pid in pids:
            total += mib
            seen = True
    return total if seen else None


def descendants(pid):
    """The child and everything it spawned. Isaac Sim's CUDA context may not live in the
    process we launched, so a single pid is not enough."""
    if psutil is None:
        return {pid}
    try:
        p = psutil.Process(pid)
        return {pid} | {c.pid for c in p.children(recursive=True)}
    except Exception:
        return {pid}


def sys_ram_gb():
    return round(psutil.virtual_memory().used / 1e9, 3) if psutil else None


def sample_cpu():
    """(whole-machine %, busiest single core %).

    Both, because they answer different purchase questions and can disagree completely. A
    simulator that is single-thread bound pins one core at 100% while the machine-wide figure
    sits at 100/20 = 5%, and reading only the machine-wide number would say "CPU is idle" when
    the truth is "one core is the wall". The candidate machines range from 8 cores (Ryzen 7 7700)
    to 24 (i9-13900KF) against 20 here, so which of these two is saturated decides whether core
    count matters at all or only clock does.

    Percentages are measured since the previous call, so the first sample is meaningless -- the
    baseline window exists partly to absorb that.
    """
    if psutil is None:
        return None, None
    per_core = psutil.cpu_percent(percpu=True)
    if not per_core:
        return None, None
    return round(sum(per_core) / len(per_core), 1), round(max(per_core), 1)


class Sampler(threading.Thread):
    def __init__(self, writer, interval, t0, want_cpu=False):
        super().__init__(daemon=True)
        self.writer, self.interval, self.t0 = writer, interval, t0
        self.want_cpu = want_cpu
        self.phase, self.pid = 'baseline', None
        self.rows = []
        # not _stop: Thread._stop is a real method that Thread.join() calls, and shadowing it
        # makes join() raise "'Event' object is not callable" after the child exits.
        self._halt = threading.Event()
        # psutil.Process.cpu_percent() is a delta since the last call ON THE SAME OBJECT, so the
        # Process handles have to survive between samples or every reading comes back 0.0.
        self._proc_cache = {}

    def _cpu_of_tree(self):
        """CPU% summed over the child and its descendants. Scale is 0..100*ncores, so on this
        20-core machine ~100 means one core's worth of work, i.e. single-thread bound however
        the scheduler shuffles it between cores -- which per-core sampling cannot see."""
        if psutil is None or self.pid is None:
            return None
        live = descendants(self.pid)
        for pid in live - set(self._proc_cache):
            try:
                p = psutil.Process(pid)
                p.cpu_percent()  # prime; first reading is always 0.0
                self._proc_cache[pid] = p
            except Exception:
                pass
        for pid in set(self._proc_cache) - live:
            self._proc_cache.pop(pid, None)
        total, seen = 0.0, False
        for p in list(self._proc_cache.values()):
            try:
                total += p.cpu_percent()
                seen = True
            except Exception:
                self._proc_cache.pop(p.pid, None)
        return round(total, 1) if seen else None

    def run(self):
        while not self._halt.is_set():
            tick = time.time()
            used, total, util = sample_gpu()
            proc = sample_proc(descendants(self.pid)) if self.pid else None
            cpu_total, cpu_core = sample_cpu() if self.want_cpu else (None, None)
            row = {'t_rel_s': round(time.time() - self.t0, 2), 'phase': self.phase,
                   'gpu_mem_used_mib': used, 'gpu_mem_total_mib': total, 'gpu_util_pct': util,
                   'proc_gpu_mem_mib': proc, 'sys_ram_used_gb': sys_ram_gb(),
                   'cpu_pct_total': cpu_total, 'cpu_pct_busiest_core': cpu_core,
                   'proc_cpu_pct': self._cpu_of_tree() if self.want_cpu else None}
            self.writer.writerow(row)
            self.rows.append(row)
            # sleep the remainder, not the whole interval: the nvidia-smi calls above cost most
            # of a second on Windows, which silently halved the sample rate.
            self._halt.wait(max(0.0, self.interval - (time.time() - tick)))

    def halt(self):
        self._halt.set()


def peak(rows, key, phase=None):
    vals = [r[key] for r in rows if r[key] is not None and (phase is None or r['phase'] == phase)]
    return max(vals) if vals else None


def median(rows, key, phase):
    vals = sorted(r[key] for r in rows if r[key] is not None and r['phase'] == phase)
    return vals[len(vals) // 2] if vals else None


def main():
    ap = argparse.ArgumentParser(usage='%(prog)s --out CSV [options] -- COMMAND [ARGS...]')
    ap.add_argument('--out', required=True, help='CSV to write')
    ap.add_argument('--interval', type=float, default=1.0, help='seconds between samples')
    ap.add_argument('--baseline', type=float, default=10.0,
                    help='seconds of idle sampling before the command starts')
    ap.add_argument('--cpu', action='store_true',
                    help='also sample CPU. OFF by default because it is the most intrusive thing '
                         'this tool does: it walks the whole Isaac Sim process tree every second '
                         'and calls cpu_percent() on each member. The first run taken with it on '
                         '(run5) diverged after an otherwise byte-identical grasp, which may be '
                         'coincidence or may be the instrument disturbing the measurement -- '
                         'until that is separated, leave it off for baseline runs.')
    ap.add_argument('--timeout', type=float, default=0.0,
                    help='kill the command after this many seconds (0 = wait forever). A pen run '
                         'that works takes about 4 minutes; one that hits the PhysX CUDA-700 '
                         'abort has been measured hanging for 70 minutes after the abort, so '
                         'leave this set when collecting repetitions.')
    ap.add_argument('cmd', nargs=argparse.REMAINDER, help='-- then the command to run')
    args = ap.parse_args()

    cmd = args.cmd[1:] if args.cmd and args.cmd[0] == '--' else args.cmd
    if not cmd:
        ap.error('no command given; put it after --')
    if psutil is None:
        print('[log_resources] psutil missing: sys_ram_used_gb and descendant pids unavailable',
              file=sys.stderr, flush=True)

    used, total, _ = sample_gpu()
    if total is None:
        print('[log_resources] nvidia-smi unavailable; refusing to log a run with no GPU data',
              file=sys.stderr, flush=True)
        return 2

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or '.', exist_ok=True)
    with open(args.out, 'w', newline='', encoding='utf-8') as fh:
        writer = csv.DictWriter(fh, fieldnames=FIELDS)
        writer.writeheader()
        t0 = time.time()
        sampler = Sampler(writer, args.interval, t0, want_cpu=args.cpu)
        sampler.start()

        print('[log_resources] idle baseline for %.0fs (GPU currently %s/%s MiB)'
              % (args.baseline, used, total), flush=True)
        time.sleep(args.baseline)

        sampler.phase = 'run'
        print('[log_resources] launching: %s' % ' '.join(cmd), flush=True)
        proc = subprocess.Popen(cmd)
        sampler.pid = proc.pid
        killed = False
        try:
            rc = proc.wait(timeout=args.timeout or None)
        except subprocess.TimeoutExpired:
            # Isaac Sim does not die when its python does. Take the descendants first, while the
            # parent is still around to enumerate them, or they are orphaned holding the GPU.
            killed = True
            kids = descendants(proc.pid) - {proc.pid}
            for pid in kids:
                try:
                    psutil.Process(pid).kill()
                except Exception:
                    pass
            proc.kill()
            rc = proc.wait()
            print('[log_resources] TIMEOUT after %.0fs -- killed the command and %d descendants'
                  % (args.timeout, len(kids)), file=sys.stderr, flush=True)

        sampler.phase = 'settle'
        time.sleep(min(5.0, args.baseline))
        sampler.halt()
        sampler.join(timeout=5)
        rows = sampler.rows

    base_gpu = median(rows, 'gpu_mem_used_mib', 'baseline')
    base_ram = median(rows, 'sys_ram_used_gb', 'baseline')
    pk_gpu = peak(rows, 'gpu_mem_used_mib', 'run')
    pk_proc = peak(rows, 'proc_gpu_mem_mib', 'run')
    pk_ram = peak(rows, 'sys_ram_used_gb', 'run')
    pk_util = peak(rows, 'gpu_util_pct', 'run')

    def delta(pk, base):
        return '%.3f' % (pk - base) if pk is not None and base is not None else 'n/a'

    print('\n[log_resources] exit=%s%s  samples=%d  csv=%s'
          % (rc, ' (KILLED ON TIMEOUT)' if killed else '', len(rows), args.out), flush=True)
    print('[log_resources] VRAM total          : %s MiB' % total, flush=True)
    print('[log_resources] VRAM idle baseline  : %s MiB (median)' % base_gpu, flush=True)
    if pk_gpu is not None:
        print('[log_resources] VRAM peak during run: %s MiB (%.1f%% of card)'
              % (pk_gpu, 100.0 * pk_gpu / total), flush=True)
    else:
        print('[log_resources] VRAM peak during run: n/a', flush=True)
    print('[log_resources]   attributable to run: %s MiB (peak - idle baseline)'
          % delta(pk_gpu, base_gpu), flush=True)
    print('[log_resources]   per-process report : %s MiB'
          % (pk_proc if pk_proc is not None else '[N/A] -- WDDM, use the line above'), flush=True)
    print('[log_resources] GPU util peak       : %s%% (time with a kernel resident, NOT proof '
          'the GPU is compute-saturated)' % pk_util, flush=True)

    ncores = psutil.cpu_count(logical=True) if psutil else None
    if not args.cpu:
        print('[log_resources] CPU                 : not sampled (pass --cpu; see its help)',
              flush=True)
    else:
        pk_cpu_all = peak(rows, 'cpu_pct_total', 'run')
        pk_cpu_core = peak(rows, 'cpu_pct_busiest_core', 'run')
        pk_cpu_proc = peak(rows, 'proc_cpu_pct', 'run')
        print('[log_resources] CPU cores (logical) : %s' % ncores, flush=True)
        print('[log_resources] CPU peak machine    : %s%%' % pk_cpu_all, flush=True)
        print('[log_resources] CPU peak one core   : %s%%' % pk_cpu_core, flush=True)
        print('[log_resources] CPU peak by run     : %s%% of %s%% available'
              % (pk_cpu_proc, 100 * ncores if ncores else '?'), flush=True)
        if pk_cpu_proc is not None and ncores:
            cores_used = pk_cpu_proc / 100.0
            if cores_used < 1.5:
                verdict = ('SINGLE-THREAD BOUND (~%.1f cores). More cores will not help; clock '
                           'speed will. An 8-core part is then no worse than a 20-core one.'
                           % cores_used)
            elif cores_used > 0.7 * ncores:
                verdict = ('MULTI-THREAD BOUND (~%.1f of %d cores). Core count matters; dropping '
                           'to 8 cores would hurt.' % (cores_used, ncores))
            else:
                verdict = ('uses ~%.1f of %d cores -- neither saturated. CPU is unlikely to be '
                           'the binding constraint.' % (cores_used, ncores))
            print('[log_resources]   -> %s' % verdict, flush=True)
    print('[log_resources] RAM idle / peak     : %s / %s GB (rise %s GB)'
          % (base_ram, pk_ram, delta(pk_ram, base_ram)), flush=True)
    print('[log_resources] a large RAM rise alongside a pinned VRAM figure means the driver is '
          'spilling to host memory, i.e. 8 GB is genuinely short.', flush=True)
    return rc


if __name__ == '__main__':
    sys.exit(main())
