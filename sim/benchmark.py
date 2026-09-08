import os
import sys
import gc
import json

# Default: Lock thread pools to the 4 physical Cortex-A76 cores before importing heavy runtimes
# When running as a --worker subprocess, these are overridden before imports happen.
if "--worker" not in sys.argv:
    os.environ["OMP_NUM_THREADS"] = "4"
    os.environ["OPENBLAS_NUM_THREADS"] = "4"
    os.environ["MKL_NUM_THREADS"] = "4"
    os.environ["VECLIB_MAXIMUM_THREADS"] = "4"
    os.environ["NUMEXPR_NUM_THREADS"] = "4"

import argparse
import time
import subprocess
from pathlib import Path
import numpy as np
import cv2
from ultralytics import YOLO

try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False

cv2.setNumThreads(1)  # Prevent OpenCV from competing with inference threads


def pin_cpu_governor():
    """Attempt to set CPU governor to 'performance' for stable clock speeds.

    Requires sudo. Falls back gracefully if unavailable.
    """
    governor_path = "/sys/devices/system/cpu/cpu{}/cpufreq/scaling_governor"
    pinned = 0
    for cpu in range(4):
        path = governor_path.format(cpu)
        try:
            subprocess.run(
                ["sudo", "tee", path],
                input=b"performance",
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=5,
            )
            pinned += 1
        except Exception:
            pass

    if pinned > 0:
        print(f"  ⚡ CPU governor pinned to 'performance' on {pinned}/4 cores")
    else:
        print("  ⚠️  Could not pin CPU governor (run with sudo for stable clocks)")


def get_hw_status():
    """Retrieve Pi 5 temperature and throttle bitmask."""
    temp, throttled = "N/A", "N/A"
    try:
        t_out = subprocess.check_output(["vcgencmd", "measure_temp"]).decode()
        temp = t_out.strip().replace("temp=", "")
    except Exception:
        pass

    try:
        th_out = subprocess.check_output(["vcgencmd", "get_throttled"]).decode()
        throttled = th_out.strip().split("=")[1]
    except Exception:
        pass

    return temp, throttled


def get_memory_mb():
    """Return current process RSS in MB, or None if psutil is unavailable."""
    if HAS_PSUTIL:
        return psutil.Process().memory_info().rss / (1024 * 1024)
    return None


def load_video_frames(video_path, num_frames=100):
    """Preload frames into RAM so storage read speed doesn't skew benchmark."""
    print(f"\n⏳ Preloading {num_frames} frames into RAM from {video_path}...")
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"❌ Cannot open video file: {video_path}")
        sys.exit(1)

    frames = []
    while len(frames) < num_frames:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = cap.read()
            if not ret:
                break
        frames.append(frame)

    cap.release()
    mem = get_memory_mb()
    mem_str = f" (RSS: {mem:.0f} MB)" if mem else ""
    print(f"✅ Loaded {len(frames)} frames into memory.{mem_str}")
    return frames


def benchmark_engine(model_path, frames, imgsz=320, warmup=25):
    """Run pure inference timing on pre-cached frames."""
    print(f"\n{'─' * 55}")
    print(f"Testing: {model_path.name}")
    print(f"{'─' * 55}")

    t_start, th_start = get_hw_status()
    mem_before = get_memory_mb()
    print(f"  Initial State -> Temp: {t_start} | Throttled: {th_start}", end="")
    if mem_before:
        print(f" | RSS: {mem_before:.0f} MB")
    else:
        print()

    try:
        model = YOLO(str(model_path), task="detect")
    except Exception as e:
        print(f"❌ Failed to load {model_path}: {e}")
        return None

    # Warmup phase — 25 iterations to stabilize NCNN/OpenVINO JIT and CPU governor
    print(f"  Warming up ({warmup} iterations)...")
    for i in range(min(warmup, len(frames))):
        _ = model(frames[i], imgsz=imgsz, conf=0.25, verbose=False)

    latencies = []
    total_detections = 0
    print(f"  Benchmarking pure inference on {len(frames)} frames...")

    # Disable GC during timed loop to avoid collection-induced latency spikes
    gc.disable()
    try:
        for frame in frames:
            t0 = time.perf_counter()
            results = model(frame, imgsz=imgsz, conf=0.25, verbose=False)
            t1 = time.perf_counter()

            latencies.append((t1 - t0) * 1000.0)
            total_detections += len(results[0].boxes)
    finally:
        gc.collect()
        gc.enable()

    t_end, th_end = get_hw_status()
    mem_after = get_memory_mb()
    print(f"  Ending State  -> Temp: {t_end} | Throttled: {th_end}", end="")
    if mem_after:
        print(f" | RSS: {mem_after:.0f} MB")
    else:
        print()

    latencies = np.array(latencies)
    avg_latency = float(latencies.mean())
    avg_fps = 1000.0 / avg_latency

    return {
        "Model": model_path.name,
        "FPS": avg_fps,
        "Latency (ms)": avg_latency,
        "Median (ms)": float(np.median(latencies)),
        "P95 (ms)": float(np.percentile(latencies, 95)),
        "P99 (ms)": float(np.percentile(latencies, 99)),
        "Std (ms)": float(latencies.std()),
        "End Temp": t_end,
        "Throttled": "YES" if th_end not in ("0x0", "N/A") else "NO",
        "Det/Frame": total_detections / len(frames),
    }


# ─────────────────────────────────────────────────────────────────────────────
# Subprocess Worker for Multi-Core Benchmark
# ─────────────────────────────────────────────────────────────────────────────
# NCNN and OpenVINO read OMP_NUM_THREADS at library init time, so the only
# reliable way to benchmark different core counts is to spawn a fresh process
# with the env var set BEFORE imports.  The worker prints a JSON result line
# tagged with __BENCHMARK_RESULT__ so the parent can parse it cleanly.
# ─────────────────────────────────────────────────────────────────────────────

def run_worker_benchmark(model_path_str, video_path, imgsz, num_frames, num_threads):
    """Isolated single-model benchmark executed in a subprocess with pinned thread count."""
    # These env vars were already set by the parent before this process started,
    # but we reinforce them here for clarity.
    os.environ["OMP_NUM_THREADS"] = str(num_threads)
    os.environ["OPENBLAS_NUM_THREADS"] = str(num_threads)
    os.environ["MKL_NUM_THREADS"] = str(num_threads)
    os.environ["VECLIB_MAXIMUM_THREADS"] = str(num_threads)
    os.environ["NUMEXPR_NUM_THREADS"] = str(num_threads)

    cv2.setNumThreads(1)

    # Pin CPU affinity if psutil is available
    if HAS_PSUTIL:
        try:
            psutil.Process().cpu_affinity(list(range(num_threads)))
        except Exception:
            pass

    # Try to set torch thread count too
    try:
        import torch
        torch.set_num_threads(num_threads)
    except Exception:
        pass

    # Preload frames
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"❌ Cannot open video file: {video_path}", file=sys.stderr)
        sys.exit(1)

    frames = []
    while len(frames) < num_frames:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = cap.read()
            if not ret:
                break
        frames.append(frame)
    cap.release()

    model_path = Path(model_path_str)
    t_start, th_start = get_hw_status()

    try:
        model = YOLO(str(model_path), task="detect")
    except Exception as e:
        print(f"❌ Failed to load {model_path}: {e}", file=sys.stderr)
        sys.exit(1)

    # Warmup (25 iterations)
    for i in range(min(25, len(frames))):
        _ = model(frames[i], imgsz=imgsz, conf=0.25, verbose=False)

    latencies = []
    total_detections = 0

    gc.disable()
    try:
        for frame in frames:
            t0 = time.perf_counter()
            results = model(frame, imgsz=imgsz, conf=0.25, verbose=False)
            t1 = time.perf_counter()
            latencies.append((t1 - t0) * 1000.0)
            total_detections += len(results[0].boxes)
    finally:
        gc.collect()
        gc.enable()

    t_end, th_end = get_hw_status()

    latencies = np.array(latencies)
    avg_latency = float(latencies.mean())
    avg_fps = 1000.0 / avg_latency

    res = {
        "Model": model_path.name,
        "Cores": num_threads,
        "FPS": avg_fps,
        "Latency (ms)": avg_latency,
        "Median (ms)": float(np.median(latencies)),
        "P95 (ms)": float(np.percentile(latencies, 95)),
        "P99 (ms)": float(np.percentile(latencies, 99)),
        "Std (ms)": float(latencies.std()),
        "End Temp": t_end,
        "Throttled": "YES" if th_end not in ("0x0", "N/A") else "NO",
        "Det/Frame": total_detections / len(frames),
    }

    # Output JSON marker so parent process can cleanly extract results
    print("__BENCHMARK_RESULT__" + json.dumps(res))
    sys.exit(0)


def run_core_comparison(available_models, args, core_configs):
    """Spawn subprocess workers to benchmark each model at each core count, then compare."""
    print(f"\n\n{'#' * 90}")
    print(f"{'MULTI-CORE COMPARISON BENCHMARK':^90}")
    print(f"{'#' * 90}")
    print(f"  Testing core configurations: {core_configs}")
    print(f"  Purpose: determine if {min(core_configs)} cores sustains enough FPS to free")
    print(f"           {4 - min(core_configs)} core(s) for Pi Camera + Arduino serial")

    all_results = []

    for core_count in core_configs:
        print(f"\n{'─' * 90}")
        print(f"  BENCHMARKING WITH {core_count} CORES (OMP_NUM_THREADS={core_count})")
        if core_count < 4:
            print(f"  [Simulates reserving {4 - core_count} core(s) for Camera + Arduino serial]")
        print(f"{'─' * 90}")

        for model_path in available_models:
            print(f"\n  ⏳ {model_path.name} @ {core_count} cores ...", end=" ", flush=True)

            # Set thread env vars BEFORE spawning the child
            worker_env = os.environ.copy()
            worker_env["OMP_NUM_THREADS"] = str(core_count)
            worker_env["OPENBLAS_NUM_THREADS"] = str(core_count)
            worker_env["MKL_NUM_THREADS"] = str(core_count)
            worker_env["VECLIB_MAXIMUM_THREADS"] = str(core_count)
            worker_env["NUMEXPR_NUM_THREADS"] = str(core_count)

            cmd = [
                sys.executable, __file__,
                "--worker",
                "--worker-model", str(model_path),
                "--worker-threads", str(core_count),
                "--video", str(args.video),
                "--imgsz", str(args.imgsz),
                "--frames", str(args.frames),
            ]

            process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, env=worker_env,
            )
            stdout, stderr = process.communicate()

            res_found = False
            for line in stdout.splitlines():
                if line.startswith("__BENCHMARK_RESULT__"):
                    data = json.loads(line.replace("__BENCHMARK_RESULT__", ""))
                    all_results.append(data)
                    res_found = True
                    print(
                        f"✅ {data['FPS']:.1f} FPS | "
                        f"Avg: {data['Latency (ms)']:.1f}ms | "
                        f"P95: {data['P95 (ms)']:.1f}ms | "
                        f"Temp: {data['End Temp']}"
                    )
                    break

            if not res_found:
                print(f"❌ FAILED")
                if stderr.strip():
                    # Show last 3 lines of error for debugging
                    for err_line in stderr.strip().splitlines()[-3:]:
                        print(f"     {err_line}")

    # ── Per-Core Results Table ──
    if all_results:
        print(f"\n\n{'=' * 95}")
        print(f"{'MULTI-CORE BENCHMARK RESULTS (Pi 5)':^95}")
        print(f"{'=' * 95}")
        header = (
            f"{'Model':<28} | {'Cores':>5} | {'Avg FPS':>8} | {'Avg Lat':>9} | {'P95':>8} "
            f"| {'P99':>8} | {'Std':>7} | {'Temp':>7} | {'Throt':>5}"
        )
        print(header)
        print("-" * len(header))

        for r in all_results:
            print(
                f"{r['Model']:<28} | {r['Cores']:>5} | {r['FPS']:>8.1f} | {r['Latency (ms)']:>7.1f}ms "
                f"| {r['P95 (ms)']:>6.1f}ms | {r['P99 (ms)']:>6.1f}ms "
                f"| {r['Std (ms)']:>5.1f}ms | {r['End Temp']:>7} | {r['Throttled']:>5}"
            )

        # ── Head-to-Head Delta Table ──
        max_cores = max(core_configs)
        min_cores = min(core_configs)

        if max_cores != min_cores:
            print(f"\n{'=' * 90}")
            print(f"{'HEAD-TO-HEAD: ' + str(max_cores) + ' CORES vs ' + str(min_cores) + ' CORES':^90}")
            print(f"{'=' * 90}")
            delta_header = (
                f"{'Model':<28} | {str(max_cores) + '-Core FPS':>10} | "
                f"{str(min_cores) + '-Core FPS':>10} | {'FPS Loss':>10} | {'P95 Δ':>10}"
            )
            print(delta_header)
            print("-" * len(delta_header))

            unique_models = sorted(set(r["Model"] for r in all_results))
            for m in unique_models:
                res_max = next((r for r in all_results if r["Model"] == m and r["Cores"] == max_cores), None)
                res_min = next((r for r in all_results if r["Model"] == m and r["Cores"] == min_cores), None)

                if res_max and res_min:
                    fps_diff_pct = ((res_max["FPS"] - res_min["FPS"]) / res_max["FPS"]) * 100.0
                    p95_diff = res_min["P95 (ms)"] - res_max["P95 (ms)"]
                    print(
                        f"{m:<28} | {res_max['FPS']:>10.1f} | {res_min['FPS']:>10.1f} | "
                        f"-{fps_diff_pct:>8.1f}% | {p95_diff:>+9.1f}ms"
                    )

            print("-" * 90)
            print("💡 Takeaway for Model Car:")
            print(f"   If {min_cores} cores maintains ≥ 25 FPS with acceptable P95 latency, dedicating")
            print(f"   {max_cores - min_cores} core(s) to Pi Camera capture + Arduino serial prevents control jitter!")
            print("=" * 90)


def main():
    parser = argparse.ArgumentParser(description="Raspberry Pi 5 YOLO Inference Benchmark")
    parser.add_argument("--video", default="test_videos/dashcam_test.mp4", help="Target test video")
    parser.add_argument("--imgsz", type=int, default=320, help="Inference resolution")
    parser.add_argument("--frames", type=int, default=100, help="Number of frames to benchmark")
    parser.add_argument(
        "--cores", nargs="+", type=int, default=[4, 3],
        help="Core configurations for multi-core comparison (default: 4 3)"
    )
    parser.add_argument("--skip-core-comparison", action="store_true",
                        help="Skip the multi-core comparison benchmark")

    # Internal worker flags (hidden from --help)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--worker-model", type=str, default="", help=argparse.SUPPRESS)
    parser.add_argument("--worker-threads", type=int, default=4, help=argparse.SUPPRESS)
    args = parser.parse_args()

    # ── Worker mode: run isolated benchmark and exit ──
    if args.worker:
        run_worker_benchmark(
            model_path_str=args.worker_model,
            video_path=args.video,
            imgsz=args.imgsz,
            num_frames=args.frames,
            num_threads=args.worker_threads,
        )
        return

    # ── Normal mode: full benchmark ──

    # Detect models matching the directory contents
    candidate_targets = [
        Path("models/yolov10n_ncnn_model"),
        Path("models/yolov10n_openvino_model"),
        Path("models/yolov10n.pt"),
    ]

    available_models = [p for p in candidate_targets if p.exists()]

    if not available_models:
        print("❌ No exported models or weights found in the current folder.")
        sys.exit(1)

    print("=" * 60)
    print("  Raspberry Pi 5 — Pure Inference Benchmark")
    print("=" * 60)
    print(f"Resolution : {args.imgsz}x{args.imgsz}")
    print(f"Frames     : {args.frames}")
    print(f"Models     : {[m.name for m in available_models]}")

    # Pin CPU governor for stable clocks
    pin_cpu_governor()

    # ── PART 1: Standard benchmark (all 4 cores) ──
    frames = load_video_frames(args.video, num_frames=args.frames)

    results = []
    for model_path in available_models:
        res = benchmark_engine(model_path, frames, imgsz=args.imgsz)
        if res:
            results.append(res)

    # Print clean comparison summary
    if results:
        print("\n\n" + "=" * 85)
        print(f"{'FINAL BENCHMARK COMPARISON (Pi 5)':^85}")
        print("=" * 85)
        header = (
            f"{'Model':<28} | {'Avg FPS':>8} | {'Latency':>10} | {'P95':>8} | {'P99':>8} "
            f"| {'Std':>7} | {'Temp':>7} | {'Throt':>5}"
        )
        print(header)
        print("-" * len(header))

        for r in results:
            print(
                f"{r['Model']:<28} | {r['FPS']:>8.1f} | {r['Latency (ms)']:>8.1f}ms "
                f"| {r['P95 (ms)']:>6.1f}ms | {r['P99 (ms)']:>6.1f}ms "
                f"| {r['Std (ms)']:>5.1f}ms | {r['End Temp']:>7} | {r['Throttled']:>5}"
            )

        best = max(results, key=lambda x: x["FPS"])
        print("\n" + "-" * 85)
        print(f"🏆 Fastest Engine: {best['Model']} ({best['FPS']:.1f} FPS, {best['Latency (ms)']:.1f} ms)")
        print("-" * 85)

    # ── PART 2: Multi-core comparison (subprocess workers) ──
    if not args.skip_core_comparison:
        run_core_comparison(available_models, args, args.cores)
    else:
        print("\n  ⏭️  Skipping multi-core comparison (--skip-core-comparison)")

    print(f"\n\n{'=' * 60}")
    print("  ALL DONE!")
    print("=" * 60)


if __name__ == "__main__":
    main()
