"""모든 정차 자리 쌍 + 대기 줄 로봇 → 픽업의 A* 길이 (새 지도). plan_tools.World를 쓴다."""
import json
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plan_tools import World  # noqa: E402

ws, astar, out = sys.argv[1], sys.argv[2], sys.argv[3]
w = World(ws, astar, out)
robots = {k: tuple(v[:2]) for k, v in yaml.safe_load(
    open(Path(ws) / 'bringup/amr_bringup/config/robots_restaurant.yaml'))['robots'].items()}
names = [k for k in w.stations if k.startswith(('stall_', 'buffet_', 't', 'L', 'b', 'c0'))]
pts = {**{k: w.stations[k] for k in names}, **robots}
res = {}
for a in pts:
    for b in names:
        if a != b:
            res[f'{a}|{b}'] = w.length(pts[a], pts[b])
Path(out, 'pairs.json').write_text(json.dumps(res))
print(len(res), sum(v is None for v in res.values()))
