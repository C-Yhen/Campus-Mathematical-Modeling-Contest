# -*- coding: utf-8 -*-
"""重建被误删的审计证据文件（UTF-8 编码）。

依次运行 certify_q1.py 的四个模式并捕获输出:
  1. --quick                        -> cycle_cover_bounds.txt（循环覆盖下界汇总）
  2. --aggregate Case1 N=3          -> aggregate_case1_n3.txt（Case1 N=3 严格排除证书）
  3. --mip --exact Case1 N=3 300s   -> exact_case1_n3.txt（完整模型限时记录）
  4. --mip --exact Case3 N=4 180s   -> exact_case3_n4_strengthened.txt（增强模型限时记录）
"""
import os
import subprocess
import sys

PY = sys.executable
CODE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.join(os.path.dirname(CODE), "docs")
CERT = os.path.join(CODE, "certify_q1.py")

JOBS = [
    (["--quick"], "cycle_cover_bounds.txt"),
    (["--aggregate", "--case", "Case1", "--n", "3"], "aggregate_case1_n3.txt"),
    (["--mip", "--exact", "--case", "Case1", "--n", "3", "--time-limit", "300"],
     "exact_case1_n3.txt"),
    (["--mip", "--exact", "--case", "Case3", "--n", "4", "--time-limit", "180"],
     "exact_case3_n4_strengthened.txt"),
]


def main() -> int:
    env = {**os.environ, "PYTHONUTF8": "1", "PYTHONIOENCODING": "utf-8"}
    only = set(sys.argv[1:])
    for args, out_name in JOBS:
        if only and out_name not in only:
            continue
        out_path = os.path.join(DOCS, out_name)
        print(f"[rebuild] {' '.join(args)}  ->  {out_name}", flush=True)
        with open(out_path, "w", encoding="utf-8") as f:
            proc = subprocess.run(
                [PY, CERT, *args],
                stdout=f,
                stderr=subprocess.STDOUT,
                encoding="utf-8",
                errors="replace",
                env=env,
            )
        print(f"[rebuild] exit={proc.returncode} ({out_name})", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
