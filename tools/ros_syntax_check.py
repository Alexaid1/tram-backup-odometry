"""Compile-check (syntax + semantics, no linking) of the ROS 2 node against real ROS 2 Humble
headers (robostack linux-64 packages) with zig c++ targeting x86_64-linux-gnu.

tram_vehicle_msgs headers are generated from std_msgs Float64/Int8 (same rosidl generator
output) by renaming + adding the std_msgs/Header member. Usage: python tools/ros_syntax_check.py
"""
import glob, os, re, subprocess, sys, shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKGS = Path(os.environ.get("ROBOSTACK_PKGS", r"C:\Users\Alexi\AppData\Local\Temp\claude\D-----------"
                           r"\657eaa0e-3682-40f6-affe-26f38c4bb40f\scratchpad\mm\root\pkgs"))
GEN = ROOT / "tools/_gen_msgs/include"


def pkg_dir(name):
    d = glob.glob(str(PKGS / "**" / f"ros-humble-{name}-[0-9]*"), recursive=True)
    d = [x for x in d if os.path.isdir(x)]
    return Path(d[0]) if d else None


def gen_msg(src_pkg, src_name, src_snake, dst_name, dst_snake, field, with_header=True):
    base = pkg_dir("std-msgs") / "include/std_msgs/std_msgs/msg"
    out = GEN / "tram_vehicle_msgs/msg"
    (out / "detail").mkdir(parents=True, exist_ok=True)
    files = [base / f"{src_snake}.hpp"] + [Path(p) for p in glob.glob(str(base / "detail" / f"{src_snake}__*.hpp"))]
    for f in files:
        if "__rosidl_typesupport" in f.name:
            continue
        t = f.read_text(encoding="utf-8")
        t = t.replace("std_msgs/msg/detail/header", "@@HDR@@")
        t = t.replace("std_msgs", "tram_vehicle_msgs").replace("STD_MSGS", "TRAM_VEHICLE_MSGS")
        t = t.replace("@@HDR@@", "std_msgs/msg/detail/header")
        t = t.replace(src_name, dst_name)
        # rename only file/guard/function-name uses, never the C types (int8_t)
        t = re.sub(rf"(?<![A-Za-z0-9]){src_snake}(?=__|\.hpp|\.h\b)", dst_snake, t)
        t = re.sub(rf"(?<=__){src_snake.upper()}(?=_|__)", dst_snake.upper(), t)
        t = re.sub(r"\bdata\b", field, t)
        t = re.sub(r"\b_data_type\b", f"_{field}_type", t)
        if with_header and f.name.endswith("__struct.hpp"):
            t = t.replace("#include <vector>", "#include <vector>\n#include \"std_msgs/msg/detail/header__struct.hpp\"", 1)
            t = t.replace(f"  // field types and members\n",
                          "  // field types and members\n  using _header_type = std_msgs::msg::Header_<ContainerAllocator>;\n  _header_type header;\n", 1)
        rel = f.relative_to(base)
        (out / rel.as_posix().replace(src_snake, dst_snake)).write_text(t, encoding="utf-8")


def includes():
    inc = [str(GEN)]
    for d in glob.glob(str(PKGS / "**" / "ros-humble-*"), recursive=True):
        p = Path(d) / "include"
        if not p.is_dir():
            continue
        for sub in p.iterdir():
            if sub.is_dir() and (sub / sub.name).is_dir():
                inc.append(str(sub))
        inc.append(str(p))
    return inc


if __name__ == "__main__":
    if GEN.exists():
        shutil.rmtree(GEN)
    gen_msg("std_msgs", "Float64", "float64", "VelocitySensor", "velocity_sensor", "velocity")
    if "--no-cmd" not in sys.argv:
        gen_msg("std_msgs", "Int8", "int8", "DriverControllerCommand", "driver_controller_command", "position")
    pkg = ROOT / "ros2_ws/src/tram_odometry"
    args = [sys.executable, "-m", "ziglang", "c++", "-target", "x86_64-linux-gnu", "-std=c++17", "-O1", "-c", "-o", str(ROOT / "tools/_bin/node_check.o"),
            "-Wall", "-Wextra", "-Wpedantic", "-Wno-unused-parameter", "-Wno-deprecated-declarations",
            "-Wno-nullability-completeness", "-Wno-unknown-warning-option", "-Wno-gnu-zero-variadic-macro-arguments",
            "-I", str(pkg / "include")]
    for i in includes():
        args += ["-isystem", i]
    extra = [a for a in sys.argv[1:] if a != "--no-cmd"]
    args += [str(pkg / "src/tram_odometry_node.cpp")] + extra
    r = subprocess.run(args, capture_output=True, text=True)
    lines = r.stderr.splitlines()
    errs = [i for i, l in enumerate(lines) if "error:" in l or "fatal error" in l]
    ours = [l for l in lines if "warning:" in l and "tram_odometry" in l]
    for i in errs[:15]:
        print("\n".join(lines[max(0, i - 2):i + 3]))
        print("---")
    print("\n".join(ours[:30]))
    print(f"errors: {len(errs)}, warnings in our code: {len(ours)}, EXIT {r.returncode}")
