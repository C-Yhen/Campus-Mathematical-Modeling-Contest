# -*- coding: utf-8 -*-
"""按题目表 2/3/4 要求重建 result1/2/3.xlsx。

- 每个文件新增“结果汇总”表：测试算例 | 无人机数量 N | Tmax | Tmin | δ（问题 1 无 δ 列）
- 各算例 sheet 修正表头为规范中文，并注明基地起返；路线数据保持不变
- Tmax/Tmin 与审计口径一致（问题 1/2 用静态度量，问题 3 用动态禁飞度量）
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import openpyxl
from solve_q2_q3 import (
    RESULT1, RESULT2, RESULT3,
    dynamic_metric, load_case, load_routes, load_zones, static_metric,
)

CASES = ("Case1", "Case2", "Case3", "Case4")
BASE = os.path.dirname(RESULT1)


def metric1(sheet):
    pts, _ = load_case(sheet)
    coords = {int(p[0]): (float(p[1]), float(p[2])) for p in pts}
    return lambda r: static_metric(r, coords)


def metric3(sheet):
    pts, _ = load_case(sheet)
    coords = {int(p[0]): (float(p[1]), float(p[2])) for p in pts}
    zones = load_zones(sheet)
    return lambda r: dynamic_metric(r, coords, zones)


def rebuild(path, metric_fn, with_delta):
    routes_all = {s: load_routes(path, s) for s in CASES}
    metrics = {s: metric_fn(s) for s in CASES}
    wb = openpyxl.Workbook()
    # ---- 汇总表（对应论文表 2/3/4）----
    ws = wb.active
    ws.title = "结果汇总"
    header = ["测试算例", "无人机数量 N", "单架无人机最长工作时间 Tmax（h）",
              "单架无人机最短工作时间 Tmin（h）"]
    if with_delta:
        header.append("δ = |Tmax − Tmin|（h）")
    ws.append(header)
    for sheet in CASES:
        routes = routes_all[sheet]
        times = [metrics[sheet](r).total / 3600.0 for r in routes]
        row = [sheet, len(routes), round(max(times), 4), round(min(times), 4)]
        if with_delta:
            row.append(round(max(times) - min(times), 4))
        ws.append(row)
    # ---- 各算例详细调度方案 ----
    for sheet in CASES:
        routes = routes_all[sheet]
        ws2 = wb.create_sheet(sheet)
        ws2.append(["算例", sheet])
        times = [metrics[sheet](r).total / 3600.0 for r in routes]
        ws2.append(["无人机数量 N", len(routes)])
        ws2.append(["Tmax（h）", round(max(times), 4)])
        ws2.append(["Tmin（h）", round(min(times), 4)])
        if with_delta:
            ws2.append(["δ（h）", round(max(times) - min(times), 4)])
        ws2.append([])
        width = max(len(r) for r in routes)
        ws2.append(["无人机编号"] + [f"第{i}个巡检点" for i in range(1, width + 1)])
        for uid, route in enumerate(routes, 1):
            ws2.append([uid] + route)
        ws2.append([])
        ws2.append(["注：所有无人机均从基地（坐标为 (0,0)）出发，完成全部巡检任务后返回基地。"])
    wb.save(path)
    print("已重建", path)


if __name__ == "__main__":
    rebuild(RESULT1, metric1, with_delta=False)
    rebuild(RESULT2, metric1, with_delta=True)
    rebuild(RESULT3, metric3, with_delta=True)
    print("完成")
