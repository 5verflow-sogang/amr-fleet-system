"""tests에서 payload_nav를 import할 수 있게 experiments/research2를 경로에 넣는다."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
