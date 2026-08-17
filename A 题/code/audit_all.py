# -*- coding: utf-8 -*-
"""A 题三问结果的一键独立审计入口。

审计边界：
1. 对 result1.xlsx 检查工作表、采用机数、精确覆盖次数、同点不连续、
   欧氏飞行与服务时间，以及每架无人机不超过 9 h；
2. 调用 audit_q2_q3.run_audit()，逐活动审计 result2.xlsx、result3.xlsx；
3. 单独报告问题一“少一架不可行”的证据等级。结果工作簿通过可行性
   审计并不自动构成机数全局最优证明。

运行：python audit_all.py
输出：A 题/docs/audit_full_run.txt
"""

from __future__ import annotations

import os
from collections import Counter
from pathlib import Path

from audit_q2_q3 import run_audit as run_q2_q3_audit
from solve_q1 import BASE_DIR, LEVEL_TIMES, RESULT1, load_case
from solve_q2_q3 import load_routes, static_metric


CASES = ("Case1", "Case2", "Case3", "Case4")
ADOPTED_N = {"Case1": 4, "Case2": 2, "Case3": 5, "Case4": 4}
TIME_LIMIT_S = 9.0 * 3600.0
TOL_S = 2e-4
AUDIT_PATH = os.path.join(BASE_DIR, "docs", "audit_full_run.txt")


def _expected_signature(sheet: str) -> Counter[int]:
    points, _ = load_case(sheet)
    return Counter({int(pid): LEVEL_TIMES[str(level)] for pid, _, _, level in points})


def _read_evidence_text(path: Path) -> str:
    """兼容 Python UTF-8 与 Windows PowerShell UTF-16 重定向日志。"""
    raw = path.read_bytes()
    if raw.startswith((b"\xff\xfe", b"\xfe\xff")):
        return raw.decode("utf-16")
    text = raw.decode("utf-8", errors="replace")
    if "\x00" in text:
        return raw.decode("utf-16-le", errors="replace")
    return text


def _q1_audit() -> tuple[list[str], list[str]]:
    lines = [
        "[I] 问题1结果工作簿可行性审计",
        "判据：采用机数一致；覆盖次数精确；同点不连续；逐路线工时<=9h。",
    ]
    all_failures: list[str] = []

    for sheet in CASES:
        failures: list[str] = []
        points, _ = load_case(sheet)
        coords = {int(pid): (float(x), float(y)) for pid, x, y, _ in points}
        routes = load_routes(RESULT1, sheet)

        if len(routes) != ADOPTED_N[sheet]:
            failures.append(f"N mismatch: expected={ADOPTED_N[sheet]}, actual={len(routes)}")

        expected = _expected_signature(sheet)
        actual = Counter(pid for route in routes for pid in route)
        if actual != expected:
            failures.append(f"coverage mismatch: expected={dict(expected)}, actual={dict(actual)}")

        adjacent_repeats = []
        for uid, route in enumerate(routes, 1):
            for pos, (a, b) in enumerate(zip(route, route[1:]), 1):
                if a == b:
                    adjacent_repeats.append((uid, pos, a))
        if adjacent_repeats:
            failures.append(f"adjacent repeated points: {adjacent_repeats}")

        metrics = [static_metric(route, coords) for route in routes]
        times = [metric.total for metric in metrics]
        overtime = [(uid, value / 3600.0) for uid, value in enumerate(times, 1)
                    if value > TIME_LIMIT_S + TOL_S]
        if overtime:
            failures.append(f"routes over 9h: {overtime}")

        decomposition_error = [
            uid for uid, metric in enumerate(metrics, 1)
            if abs(metric.total - metric.flight - metric.service - metric.wait) > TOL_S
        ]
        if decomposition_error:
            failures.append(f"metric decomposition mismatch: UAV={decomposition_error}")

        lines.append(
            f"{sheet} Q1: N={len(routes)}, visits={sum(map(len, routes))}, "
            f"Tmax={max(times)/3600:.6f}h, Tmin={min(times)/3600:.6f}h, "
            f"delta={(max(times)-min(times))/3600:.6f}h, "
            f"coverage={'PASS' if actual == expected else 'FAIL'}, "
            f"under_9h={'PASS' if not overtime else 'FAIL'}, "
            f"status={'PASS' if not failures else 'FAIL'}"
        )
        for failure in failures:
            lines.append(f"  FAILURE: {failure}")
            all_failures.append(f"{sheet} Q1: {failure}")

    return lines, all_failures


def _evidence_audit() -> tuple[list[str], list[str]]:
    """核对问题一机数结论的证据文件，并如实标注结论强度。"""
    docs = Path(BASE_DIR) / "docs"
    required_tokens = {
        "cycle_cover_bounds.txt": (
            "严格循环覆盖下界汇总",
            (
                "Case2   1     139          164.518      14.5746    14.5746      True",
                "Case4   3     182          713.878      28.1463     9.3821      True",
            ),
        ),
        "aggregate_case1_n3.txt": (
            "Case1, N=3 聚合总工时全局下界证书",
            (
                "excluded_by_bound=True",
                "严格证书：总工时全局下界 27.064345 h > 27.000000 h",
            ),
        ),
        "exact_case1_n3.txt": (
            "Case1, N=3 完整模型限时记录",
            ("case='Case1', n_uav=3", "Time limit reached", "feasible_under_9h=None"),
        ),
        "exact_case3_n4_strengthened.txt": (
            "Case3, N=4 增强模型限时记录",
            ("case='Case3', n_uav=4", "Time limit reached", "feasible_under_9h=None"),
        ),
    }
    failures: list[str] = []
    for filename, (meaning, tokens) in required_tokens.items():
        path = docs / filename
        if not path.is_file() or path.stat().st_size == 0:
            failures.append(f"missing/empty evidence file: {filename} ({meaning})")
            continue
        content = _read_evidence_text(path)
        missing_tokens = [token for token in tokens if token not in content]
        if missing_tokens:
            failures.append(
                f"evidence content mismatch: {filename}, missing={missing_tokens}"
            )

    lines = [
        "[II] 问题1采用机数的证据等级（不计入路线可行性 PASS）",
        "Case1: 采用N=4；N=1、N=2由循环覆盖下界排除；N=3由聚合总工时全局下界27.0643h>27h严格排除，少一架严格不可行。",
        "Case2: 采用N=2；N=1循环覆盖平均负载下界14.5746h>9h，少一架严格不可行。",
        "Case3: 采用N=5；N=1~3由循环覆盖下界排除；N=4增强完整模型180s限时未形成结论，故当前为可行上界/计算采用值，未证明全局最小。",
        "Case4: 采用N=4；N=3循环覆盖平均负载下界9.3821h>9h，少一架严格不可行。",
    ]
    for failure in failures:
        lines.append(f"  FAILURE: {failure}")
    return lines, failures


def run_audit() -> str:
    q1_lines, q1_failures = _q1_audit()
    evidence_lines, evidence_failures = _evidence_audit()
    q23_report = run_q2_q3_audit().rstrip()
    q23_failed = "OVERALL: FAIL" in q23_report

    failures = q1_failures + evidence_failures
    if q23_failed:
        failures.append("问题2、3独立审计失败")

    lines = [
        "A题三问全量复现审计",
        "说明：PASS只表示所交结果满足本脚本列出的可行性/轨迹判据；全局最优性证据另行分级。",
        "",
        *q1_lines,
        "",
        *evidence_lines,
        "",
        "[III] 问题2、3独立审计",
        q23_report,
        "",
        f"FINAL OVERALL: {'PASS' if not failures else 'FAIL'}",
    ]
    if failures:
        lines.append(f"failure_count={len(failures)}")
    return "\n".join(lines) + "\n"


def main() -> None:
    report = run_audit()
    os.makedirs(os.path.dirname(AUDIT_PATH), exist_ok=True)
    with open(AUDIT_PATH, "w", encoding="utf-8") as file:
        file.write(report)
    print(report, end="")
    if "FINAL OVERALL: FAIL" in report:
        raise SystemExit(1)


if __name__ == "__main__":
    main()