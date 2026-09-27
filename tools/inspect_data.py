"""Инвентаризация данных: pathgraph, проверочный bag, датасет."""
import json, sys, math
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

ROOT = Path(__file__).resolve().parents[1]
MSG_DIR = ROOT / "dataset" / "tram_vehicle_msgs" / "msg"


def typestore():
    ts = get_typestore(Stores.ROS2_HUMBLE)
    add = {}
    for f in MSG_DIR.glob("*.msg"):
        add.update(get_types_from_msg(f.read_text(encoding="utf-8"),
                                      f"tram_vehicle_msgs/msg/{f.stem}"))
    ts.register(add)
    return ts


def read_bag(path):
    """Returns dict topic -> dict(t_bag, t_hdr, fields...) as numpy arrays."""
    ts = typestore()
    out = {}
    with AnyReader([Path(path)], default_typestore=ts) as r:
        for c, t, raw in r.messages():
            m = r.deserialize(raw, c.msgtype)
            d = out.setdefault(c.topic, {"t": [], "h": [], "v": []})
            d["t"].append(t * 1e-9)
            st = m.header.stamp
            d["h"].append(st.sec + st.nanosec * 1e-9)
            tp = c.msgtype
            if tp.endswith("VelocitySensor"):
                d["v"].append((m.velocity,))
            elif tp.endswith("DriverControllerCommand"):
                d["v"].append((m.position,))
            elif tp.endswith("NavSatFix"):
                d["v"].append((m.latitude, m.longitude, m.altitude, m.status.status,
                               m.position_covariance[0]))
            elif tp.endswith("TwistStamped"):
                l = m.twist.linear
                d["v"].append((l.x, l.y, l.z))
            elif tp.endswith("Odometry"):
                p = m.pose.pose.position; q = m.pose.pose.orientation
                l = m.twist.twist.linear
                d["v"].append((p.x, p.y, p.z, q.x, q.y, q.z, q.w, l.x, l.y, l.z,
                               m.pose.covariance[0]))
                d.setdefault("frame", m.header.frame_id)
                d.setdefault("child", m.child_frame_id)
            if "frame" not in d:
                d["frame"] = m.header.frame_id
    for d in out.values():
        for k in ("t", "h", "v"):
            d[k] = np.asarray(d[k], dtype=float)
    return out


def describe(bag):
    t0 = min(d["t"][0] for d in bag.values())
    for topic, d in sorted(bag.items()):
        t = d["t"]; dt = np.diff(t)
        hd = d["t"] - d["h"]
        print(f"\n{topic}  n={len(t)} frame='{d['frame']}'"
              f" span=[{t[0]-t0:.1f},{t[-1]-t0:.1f}]s")
        if len(dt):
            print(f"  rate~{1/np.median(dt):.1f}Hz dt p50={np.median(dt)*1e3:.1f}ms"
                  f" p99={np.percentile(dt,99)*1e3:.1f}ms max={dt.max()*1e3:.0f}ms"
                  f" gaps>0.5s={int((dt>0.5).sum())}")
        print(f"  bag_t - header_t: median={np.median(hd)*1e3:.1f}ms"
              f" min={hd.min()*1e3:.1f} max={hd.max()*1e3:.1f}")
        v = d["v"]
        for i in range(v.shape[1]):
            col = v[:, i]
            print(f"  f{i}: min={col.min():.6g} max={col.max():.6g}"
                  f" mean={col.mean():.6g} nan={int(np.isnan(col).sum())}")


if __name__ == "__main__":
    describe(read_bag(sys.argv[1]))
