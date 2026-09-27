"""Offline test of scripts/evaluate_run.py: builds recorder-format CSVs from a cached bag
and the replay output, runs evaluate_run.py, and checks its Python UTM against pyproj."""
import subprocess, sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from evaluate import load_csv

run = sys.argv[1] if len(sys.argv) > 1 else "30618_88aea4d9"
drop_ref = "--gnss-ref" in sys.argv
z = np.load(ROOT / f"tools/_cache/{run}.npz")
E = load_csv(ROOT / f"tools/_eval/{run}_est.csv")
d = ROOT / f"tools/_rec_{run}"
d.mkdir(exist_ok=True)
with open(d / "result_velocity.csv", "w") as f:
    f.write("stamp,velocity,recv_wall\n")
    for t, v, w in zip(E["t"], E["v"], E["t_recv"]):
        f.write(f"{t:.6f},{v:.5f},{w + 0.0005:.6f}\n")
with open(d / "result_position.csv", "w") as f:
    f.write("stamp,frame,x,y,z,yaw,vx,cov_xx,cov_yy,twist_cov_vv,recv_wall\n")
    for i in np.flatnonzero(E["valid"] > 0):
        f.write(f"{E['t'][i]:.6f},map,{E['x'][i]:.4f},{E['y'][i]:.4f},{E['z'][i]:.4f},{E['yaw'][i]:.5f},"
                f"{E['v'][i]:.5f},1,1,{E['var_v'][i]:.6f},{E['t_recv'][i] + 0.0005:.6f}\n")
with open(d / "inputs.csv", "w") as f:
    f.write("topic,stamp,value,recv_wall\n")
    for key, name in (("front", "front"), ("rear", "rear"), ("cmd", "cmd")):
        for h, v, t in zip(z[f"{key}_h"], z[f"{key}_v"][:, 0], z[f"{key}_t"]):
            f.write(f"{name},{h:.6f},{v},{t:.6f}\n")
with open(d / "reference.csv", "w") as f:
    f.write("stamp,x,y,z,v\n")
    if "ref_v" in z and not drop_ref:
        for h, r in zip(z["ref_h"], z["ref_v"]):
            f.write(f"{h:.6f},{r[0]:.4f},{r[1]:.4f},{r[2]:.4f},{r[7]:.5f}\n")
with open(d / "gnss.csv", "w") as f:
    f.write("topic,stamp,lat,lon,alt,status,vx,vy,recv_wall\n")
    for h, v in zip(z["mfix_h"], z["mfix_v"]):
        f.write(f"master_fix,{h:.6f},{v[0]:.9f},{v[1]:.9f},{v[2]:.3f},{int(v[3])},,,0\n")
    for h, v in zip(z["mvel_h"], z["mvel_v"]):
        f.write(f"master_vel,{h:.6f},,,,,{v[0]:.4f},{v[1]:.4f},0\n")
with open(d / "resources.csv", "w") as f:
    f.write("wall,cpu_percent,rss_mb\n")
    for k in range(20):
        f.write(f"{k},{1.5 + 0.1 * k % 3:.2f},{12.0 + 0.01 * k:.2f}\n")
r = subprocess.run([sys.executable, str(ROOT / "ros2_ws/src/tram_odometry/scripts/evaluate_run.py"), str(d),
                    str(ROOT / "ros2_ws/src/tram_odometry/maps/route_closed.json")], capture_output=True, text=True)
print(r.stdout[-3000:]); print(r.stderr[-2000:])
# UTM port check
spec = __import__("importlib.util").util.spec_from_file_location("er", ROOT / "ros2_ws/src/tram_odometry/scripts/evaluate_run.py")
er = __import__("importlib.util").util.module_from_spec(spec); spec.loader.exec_module(er)
from pyproj import Transformer
tr = Transformer.from_crs("EPSG:4326", "EPSG:32637", always_xy=True)
lat = z["mfix_v"][:200, 0]; lon = z["mfix_v"][:200, 1]
e1, n1 = er.utm(lat, lon); e2, n2 = tr.transform(lon, lat)
print("python UTM vs pyproj max diff (m):", float(np.max(np.hypot(e1 - e2, n1 - n2))))
