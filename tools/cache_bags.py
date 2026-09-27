"""Чтение всех bag в компактный кэш .npz (один файл на прогон).

Для каждого топика сохраняются массивы: <key>_t (bag time), <key>_h (header stamp), <key>_v.
GNSS fix дополнительно конвертируется в map (UTM37N - (300000, 6100000)).
"""
import sys
from pathlib import Path
from multiprocessing import Pool
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from inspect_data import read_bag

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "tools" / "_cache"
KEYS = {
    "/vehicle/front_bogie_velocity": "front",
    "/vehicle/rear_bogie_velocity": "rear",
    "/vehicle/driver_position_cmd": "cmd",
    "/sensing/gnss/master/fix": "mfix",
    "/sensing/gnss/rover/fix": "rfix",
    "/sensing/gnss/master/vel": "mvel",
    "/sensing/gnss/rover/vel": "rvel",
    "/localization/kinematic_state": "ref",
}


def to_map(lat, lon):
    from pyproj import Transformer
    tr = Transformer.from_crs("EPSG:4326", "EPSG:32637", always_xy=True)
    e, n = tr.transform(lon, lat)
    return np.asarray(e) - 300000.0, np.asarray(n) - 6100000.0


def one(path):
    path = Path(path)
    out = OUT / (path.name + ".npz")
    if out.exists():
        return path.name, "cached"
    try:
        bag = read_bag(path)
    except Exception as e:  # noqa
        return path.name, f"ERR {e}"
    arrs = {}
    for topic, d in bag.items():
        k = KEYS.get(topic)
        if k is None:
            continue
        arrs[k + "_t"] = d["t"]; arrs[k + "_h"] = d["h"]; arrs[k + "_v"] = d["v"]
        if k in ("mfix", "rfix") and len(d["v"]):
            x, y = to_map(d["v"][:, 0], d["v"][:, 1])
            arrs[k + "_xy"] = np.stack([x, y], -1)
    np.savez_compressed(out, **arrs)
    return path.name, "ok"


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    bags = sorted(p for p in (ROOT / "dataset" / "data").iterdir() if p.is_dir())
    bags.append(ROOT / "check-code-with-bag/check-code/bags/30618_88aea4d9")
    with Pool(6) as pool:
        for name, st in pool.imap_unordered(one, bags):
            print(name, st, flush=True)
