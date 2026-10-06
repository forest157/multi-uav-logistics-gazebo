"""Bounded, transition-based operator events; no ROS or Qt dependency."""

import math


def _text(value, limit):
    if not isinstance(value, str):
        return ''
    return ' '.join(value.split())[:limit]


class OperatorEventJournal:
    def __init__(self, limit=50):
        if type(limit) is not int or not 1 <= limit <= 200:
            raise ValueError('event limit must be 1..200')
        self.limit = limit
        self.rows = []
        self.transitions = {}
        self.last_plan = None

    def _append(self, source, level, title, guidance, now):
        if type(now) not in (int, float) or not math.isfinite(now) or now < 0:
            raise ValueError('invalid event timestamp')
        self.rows.insert(0, {
            'time': float(now), 'source': source, 'level': level,
            'title': _text(title, 160), 'guidance': _text(guidance, 320),
        })
        del self.rows[self.limit:]
        return True

    def _transition(self, source, key, level, title, guidance, now):
        old = self.transitions.get(source)
        if old == key:
            return False
        self.transitions[source] = key
        if key == 'NORMAL' and old is None:
            return False
        return self._append(source, level, title, guidance, now)

    def planning_failure(self, category, title, guidance, now):
        category = category if category in (
            'INPUT', 'FEASIBILITY', 'PLANNING', 'DYNAMICS',
            'ENVIRONMENT', 'INTERNAL', 'TIMEOUT') else 'UNKNOWN'
        title = _text(title, 160) or '规划失败'
        guidance = _text(guidance, 320) or '检查规划日志并调整任务参数。'
        identity = (category, title, guidance)
        if self.last_plan is not None and self.last_plan[0] == identity and now - self.last_plan[1] < 2.0:
            return False
        self.last_plan = (identity, now)
        return self._append('规划/'+category, 'ERROR', title, guidance, now)

    def mission(self, payload, now):
        if not isinstance(payload, dict):
            return False
        changed = False
        interlock = payload.get('safety_interlock')
        if isinstance(interlock, dict) and type(interlock.get('hold')) is bool:
            if interlock['hold']:
                reason = _text(interlock.get('reason'), 120)
                changed |= self._transition('安全联锁', 'HOLD', 'ERROR',
                    '安全联锁保持', reason or '检查机间距、静态净空与诊断；勿强制继续。', now)
            else:
                changed |= self._transition('安全联锁', 'NORMAL', 'INFO',
                    '安全联锁解除', '确认状态与净空稳定后再继续任务。', now)
        action = payload.get('dynamic_action')
        if action == 'HOLD':
            changed |= self._transition('动态避障', 'HOLD', 'WARN',
                '动态避障保持', '查看冲突预测和感知新鲜度；等待安全候选或人工处理。', now)
        elif action in ('NORMAL', 'SLOW', 'AVOID', 'ORCA'):
            changed |= self._transition('动态避障', 'NORMAL', 'INFO',
                '动态避障保持解除', '观察航迹收敛，再继续任务。', now)
        state = payload.get('state')
        if state == 'EMERGENCY_LAND':
            changed |= self._transition('任务', 'EMERGENCY_LAND', 'ERROR',
                '紧急降落', '确认所有无人机落地并检查任务日志。', now)
        elif state == 'COMPLETE':
            changed |= self._transition('任务', 'COMPLETE', 'INFO',
                '任务完成', '核对解除武装与任务报告。', now)
        elif state in ('READY', 'RUNNING', 'PAUSED', 'INITIALIZING'):
            self.transitions['任务'] = 'NORMAL'
        return changed

    def diagnostic(self, level, message, now):
        if type(level) is not int or level not in (0, 1, 2, 3):
            return False
        if level == 0:
            return self._transition('静态安全', 'NORMAL', 'INFO',
                '静态安全诊断恢复', '继续监视机间距、障碍净空和跟踪误差。', now)
        names = {1: ('WARN', '跟踪或净空警告'),
                 2: ('ERROR', '静态安全违规'),
                 3: ('WARN', '静态安全数据未就绪')}
        severity, title = names[level]
        guidance = _text(message, 160) or '查看安全诊断；状态未恢复前勿继续。'
        return self._transition('静态安全', level, severity, title, guidance, now)

    def risk(self, level, now):
        if level == 'SAFE':
            return self._transition('动态风险', 'NORMAL', 'INFO',
                '动态风险解除', '持续观察障碍轨迹与安全净空。', now)
        names = {'WARNING': ('WARN', '动态冲突预警', '检查冲突对象和预测净空。'),
                 'CRITICAL': ('ERROR', '动态冲突严重', '检查任务保持和安全联锁状态。'),
                 'STALE': ('WARN', '动态风险数据过期', '检查感知和风险节点；勿依赖旧预测。')}
        if level not in names:
            return False
        severity, title, guidance = names[level]
        return self._transition('动态风险', level, severity, title, guidance, now)
