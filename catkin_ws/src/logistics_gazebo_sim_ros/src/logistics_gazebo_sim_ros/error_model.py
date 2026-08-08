"""Structured diagnostics shared by mission generation and the rqt UI."""
import json

ERRORS = {
    "E_SCENE": ("INPUT", "ERROR", False, "场景编号无效", ["select_valid_scene"]),
    "E_ALTITUDE": ("INPUT", "ERROR", False, "飞行高度超出允许范围", ["adjust_altitude"]),
    "E_DISTANCE": ("INPUT", "ERROR", False, "起点与终点距离必须至少为3 m", ["move_start_or_goal"]),
    "E_BOUNDARY": ("INPUT", "ERROR", False, "起终点或编队包络超出安全边界", ["move_start_or_goal"]),
    "E_BLOCKED": ("FEASIBILITY", "ERROR", False, "起点或终点位于障碍物安全区内", ["move_start_or_goal", "adjust_altitude"]),
    "E_FORMATION": ("INPUT", "ERROR", False, "队形、数量或间距参数无效", ["change_formation", "adjust_spacing"]),
    "E_START_CAPACITY": ("FEASIBILITY", "ERROR", False, "起点区域无法容纳初始或巡航队形", ["move_start", "use_compact_formation"]),
    "E_GOAL_CAPACITY": ("FEASIBILITY", "ERROR", False, "终点区域无法容纳巡航队形", ["move_goal", "use_compact_formation"]),
    "E_DELIVERY_CAPACITY": ("FEASIBILITY", "ERROR", False, "投递区域无法容纳一字队形", ["move_goal", "adjust_altitude"]),
    "E_LANDING_CAPACITY": ("FEASIBILITY", "ERROR", False, "返航降落区域无法容纳降落队形", ["move_start", "use_alternate_landing_site"]),
    "E_VERTICAL_CLEARANCE": ("FEASIBILITY", "ERROR", False, "上下净空不足，无法容纳当前三维队形", ["adjust_altitude", "use_flat_formation"]),
    "E_CORRIDOR_TOO_NARROW": ("FEASIBILITY", "ERROR", False, "路径局部净空不足，无法容纳完整编队", ["increase_altitude", "use_column", "use_vertical_formation", "replan_path"]),
    "E_NO_FEASIBLE_FORMATION": ("FEASIBILITY", "ERROR", False, "局部路段没有可安全通过的候选队形", ["increase_altitude", "replan_path", "reduce_fleet_size"]),
    "E_FORMATION_TRANSITION": ("FEASIBILITY", "ERROR", False, "找不到满足机间距和障碍物约束的队形变换窗口", ["replan_path", "increase_altitude", "move_transition_area"]),
    "E_OMPL_BINARY": ("ENVIRONMENT", "ERROR", True, "OMPL规划程序不可用", ["rebuild_workspace", "check_installation"]),
    "E_OMPL_TIMEOUT": ("PLANNING", "WARNING", True, "规划器运行超时，当前结果不能证明任务无解", ["retry", "increase_planning_time", "adjust_route"]),
    "E_OMPL_NO_PATH": ("PLANNING", "WARNING", True, "当前求解时间内没有找到路径，不代表场景一定无解", ["retry", "adjust_altitude", "change_formation"]),
    "E_OMPL_OUTPUT": ("INTERNAL", "ERROR", True, "OMPL输出格式异常", ["retry", "inspect_planner_log"]),
    "E_TOPPRA": ("DYNAMICS", "ERROR", True, "路径无法满足轨迹连续性或动力学约束", ["retry", "reduce_speed", "replan_path"]),
    "E_REPORT": ("INTERNAL", "ERROR", True, "规划分析报告缺失或损坏", ["retry", "inspect_log"]),
    "E_ENVIRONMENT": ("ENVIRONMENT", "ERROR", True, "运行环境发生间歇性异常", ["retry", "inspect_environment"]),
    "E_INTERNAL": ("INTERNAL", "ERROR", True, "规划系统内部异常", ["retry", "inspect_log"]),
}

ENVIRONMENT_MARKERS = (
    "object is not callable", "object is not iterable",
    "keywords must be strings", "Parameter' object",
    "unsupported operand type", "SafeDumper", "SafeLoader",
    "has no attribute 'nodeType'", "not supported between instances of",
)


def diagnostic(code, detail="", context=None):
    category, severity, retryable, message, suggestions = ERRORS.get(
        code, ERRORS["E_INTERNAL"])
    return {"code": code if code in ERRORS else "E_INTERNAL",
            "category": category, "severity": severity,
            "retryable": retryable, "message": message,
            "detail": str(detail), "context": context or {},
            "suggestions": list(suggestions)}


def classify_exception(exc):
    text = str(exc)
    if any(marker in text for marker in ENVIRONMENT_MARKERS):
        return diagnostic("E_ENVIRONMENT", text)
    code = text.split(":", 1)[0]
    detail = text.split(":", 1)[1] if ":" in text else text
    context={}
    if detail.lstrip().startswith("{"):
        try:
            context=json.loads(detail);detail=context.get("message",detail)
        except (TypeError,ValueError):pass
    return diagnostic(code, detail, context)
