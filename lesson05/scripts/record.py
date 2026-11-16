"""Record two real X11 demo windows; keep all raw frames, telemetry and impacts.

Run from anywhere: python3 lesson05/scripts/record.py /fresh/output/directory
"""
import argparse
import ctypes
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]
TITLE = "Lesson05 Ballistics + Live Hits"


def find_window():
    tree = subprocess.check_output(["xwininfo", "-root", "-tree"], text=True)
    for line in tree.splitlines():
        if f'"{TITLE}"' in line:
            return int(line.split()[0], 16)
    return None


def arrange(wid):
    # Xlib dimensions are physical pixels, avoiding GTK/Qt scale-factor mismatch.
    x = ctypes.CDLL("libX11.so.6")
    x.XOpenDisplay.restype = ctypes.c_void_p
    display = ctypes.c_void_p(x.XOpenDisplay(None))
    if not display.value:
        raise RuntimeError("Cannot open X11 display")
    x.XMoveResizeWindow(display, ctypes.c_ulong(wid), 60, 100, 1440, 1120)
    x.XRaiseWindow(display, ctypes.c_ulong(wid))
    x.XSetInputFocus(display, ctypes.c_ulong(wid), 1, 0)
    x.XSync(display, 0)
    x.XCloseDisplay(display)


def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)


def ffmpeg_path():
    ffmpeg = os.environ.get("FFMPEG") or shutil.which("ffmpeg")
    if not ffmpeg:
        choices = list(Path.home().glob("RM_Lesson03_Work/media-tools/imageio_ffmpeg/binaries/ffmpeg-linux-*"))
        if choices:
            ffmpeg = str(choices[0])
    if not ffmpeg:
        raise RuntimeError("Set FFMPEG to an existing ffmpeg executable")
    return ffmpeg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seconds", type=int, default=32)
    args = parser.parse_args()
    if args.seconds < 10 or args.seconds > 600:
        parser.error("seconds must be 10..600")
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    if os.environ.get("WAYLAND_DISPLAY") or not os.environ.get("DISPLAY"):
        raise RuntimeError("This window recorder requires an X11 session; use demo + export_video.py on Wayland")
    ffmpeg = ffmpeg_path()
    print("宝贝，开始自动录制静止靶和平移靶；窗口内会显示REC和倒计时。", flush=True)
    print("保存目录：", out, flush=True)
    for scenario in ("static", "translation"):
        folder = out / scenario
        folder.mkdir()
        simulator = demo = recorder = None
        env = os.environ.copy()
        env["TRAINING_HIT_LOG"] = str(folder / "projectiles.jsonl")
        try:
            with (folder / "simulator.log").open("w") as sl, (folder / "demo.log").open("w") as dl, (folder / "recorder.log").open("w") as rl:
                def start(argv, log, extra_env=None):
                    return subprocess.Popen(argv, cwd=ROOT, env=extra_env, stdout=log,
                                            stderr=subprocess.STDOUT, start_new_session=True)
                simulator = start(["bash", "lesson05/scripts/run.sh", "simulator", scenario], sl, env)
                time.sleep(6)
                if simulator.poll() is not None:
                    raise RuntimeError(f"{scenario}: simulator exited; see simulator.log")
                demo = start(["bash", "lesson05/scripts/run.sh", "demo", "--seconds", str(args.seconds),
                              "--scenario", scenario, "--output", str(folder / "capture")], dl)
                wid = None
                for _ in range(100):
                    wid = find_window()
                    if wid:
                        break
                    if demo.poll() is not None:
                        raise RuntimeError(f"{scenario}: demo exited; see demo.log")
                    time.sleep(.2)
                if not wid:
                    raise RuntimeError("Actual demo window not found")
                arrange(wid)
                time.sleep(1)
                print(f"● 正在录屏：{scenario}，约{args.seconds-2}秒", flush=True)
                recorder = start([ffmpeg, "-hide_banner", "-nostdin", "-n", "-f", "x11grab",
                                  "-framerate", "25", "-window_id", str(wid), "-i", os.environ["DISPLAY"],
                                  "-t", str(args.seconds-2), "-an", "-c:v", "libx264", "-preset", "veryfast",
                                  "-crf", "18", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                                  str(folder / "desktop.mp4")], rl)
                recorder.wait(timeout=args.seconds+10)
                demo.wait(timeout=15)
                if recorder.returncode or demo.returncode:
                    raise RuntimeError(f"{scenario}: recording or hit acceptance failed; see logs")
                print(f"{scenario}: actual window recording and armor-hit acceptance PASS", flush=True)
        finally:
            stop(recorder)
            stop(demo)
            stop(simulator)
    concat = out / "concat.txt"
    concat.write_text("file 'static/desktop.mp4'\nfile 'translation/desktop.mp4'\n")
    subprocess.run([ffmpeg, "-hide_banner", "-loglevel", "error", "-n", "-f", "concat", "-safe", "0",
                    "-i", str(concat), "-c", "copy", "-movflags", "+faststart", str(out / "lesson05-static-and-translation.mp4")], check=True)
    print("已保存：", out / "lesson05-static-and-translation.mp4", flush=True)


if __name__ == "__main__":
    main()
