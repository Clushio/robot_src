"""ANAV GUI 的独立状态与异常中心。"""

import csv
from collections import deque
import json
import os
import queue
import threading
import time

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QAbstractItemView, QCheckBox, QDialog, QFileDialog, QHeaderView,
    QHBoxLayout, QLabel, QMessageBox, QPlainTextEdit, QPushButton,
    QTableWidget, QTableWidgetItem, QVBoxLayout,
)


LEVEL_OK = 0
LEVEL_WARN = 1
LEVEL_ERROR = 2
LEVEL_STALE = 3

LEVEL_NAMES = {
    LEVEL_OK: '恢复',
    LEVEL_WARN: 'WARN',
    LEVEL_ERROR: 'ERROR',
    LEVEL_STALE: 'STALE',
}

LEVEL_COLORS = {
    LEVEL_OK: QColor('#067647'),
    LEVEL_WARN: QColor('#B54708'),
    LEVEL_ERROR: QColor('#B42318'),
    LEVEL_STALE: QColor('#7A271A'),
}


def _truthy(value, default=False):
    if value is None:
        return default
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def diagnostic_events_from_array(message):
    """Convert a ROS DiagnosticArray into thread-safe plain dictionaries."""
    stamp = time.time()
    try:
        if message.header.stamp.to_sec() > 0:
            stamp = message.header.stamp.to_sec()
    except (AttributeError, TypeError):
        pass

    events = []
    for status in message.status:
        values = {item.key: item.value for item in status.values}
        code = values.get('code', 'ANAV-SYS-999')
        # level = int(status.level)
        if isinstance(status.level, (bytes, bytearray)):
            level = status.level[0]
        else:
            level = int(status.level)
        events.append({
            'module': status.name or 'unknown',
            'code': code,
            'level': level,
            'message': status.message or values.get('state', code),
            'active': _truthy(values.get('active'), level != LEVEL_OK),
            'detail': values.get('detail', values.get('reason', '')),
            'action': values.get('action', ''),
            'kind': values.get('kind', 'FAULT'),
            'values': values,
            'timestamp': stamp,
        })
    return events


class DiagnosticInbox:
    """Thread-safe, bounded transition inbox used by ROS callback threads."""

    def __init__(self, max_events=2048, repeat_interval=1.0):
        self.max_events = max(32, int(max_events))
        self.repeat_interval = max(0.1, float(repeat_interval))
        self._events = deque()
        self._last_state_by_module = {}
        self._last_enqueue_by_module = {}
        self._dropped = 0
        self._lock = threading.Lock()

    def submit_array(self, message):
        now = time.monotonic()
        for event in diagnostic_events_from_array(message):
            self.submit_event(event, now=now)

    def submit_event(self, event, now=None):
        now = time.monotonic() if now is None else now
        module = event['module']
        state = (
            event['code'], int(event['level']), bool(event['active'])
        )
        with self._lock:
            transition = self._last_state_by_module.get(module) != state
            last_enqueue = self._last_enqueue_by_module.get(module, 0.0)
            if not transition and now - last_enqueue < self.repeat_interval:
                return False

            queued_event = dict(event)
            queued_event['values'] = dict(event.get('values') or {})
            queued_event['_transition'] = transition
            self._make_room_locked()
            self._events.append(queued_event)
            self._last_state_by_module[module] = state
            self._last_enqueue_by_module[module] = now
            return True

    def _make_room_locked(self):
        if len(self._events) < self.max_events:
            return

        # Prefer dropping an old heartbeat. State transitions are retained
        # unless the queue consists entirely of transitions.
        for index, event in enumerate(self._events):
            if not event.get('_transition', False):
                del self._events[index]
                self._dropped += 1
                return
        self._events.popleft()
        self._dropped += 1

    def drain(self, max_items=256):
        drained = []
        with self._lock:
            for _ in range(min(max_items, len(self._events))):
                event = self._events.popleft()
                event.pop('_transition', None)
                drained.append(event)
            dropped = self._dropped
            self._dropped = 0
        return drained, dropped


class DiagnosticLogWriter:
    """Write JSONL records away from the Qt GUI thread."""

    def __init__(self, log_directory, max_pending=2048):
        self.log_directory = log_directory
        self._queue = queue.Queue(maxsize=max(32, int(max_pending)))
        self._stop_event = threading.Event()
        self._last_warning_at = 0.0
        self._thread = threading.Thread(
            target=self._run, name='anav-diagnostic-log', daemon=True
        )
        self._thread.start()

    def submit(self, event):
        if self._stop_event.is_set():
            return
        serializable = dict(event)
        serializable['values'] = dict(event.get('values') or {})
        try:
            self._queue.put_nowait(serializable)
        except queue.Full:
            self._warn_throttled('Diagnostic log queue is full; record dropped')

    def _warn_throttled(self, message):
        now = time.monotonic()
        if now - self._last_warning_at >= 5.0:
            print(message)
            self._last_warning_at = now

    def _run(self):
        while not self._stop_event.is_set() or not self._queue.empty():
            try:
                event = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                os.makedirs(self.log_directory, exist_ok=True)
                date = time.strftime(
                    '%Y%m%d', time.localtime(event['timestamp'])
                )
                path = os.path.join(
                    self.log_directory, f'diagnostics_{date}.jsonl'
                )
                with open(path, 'a', encoding='utf-8') as handle:
                    handle.write(json.dumps(
                        event, ensure_ascii=False, sort_keys=True
                    ) + '\n')
            except OSError as error:
                self._warn_throttled(
                    f'Unable to persist diagnostic event: {error}'
                )
            finally:
                self._queue.task_done()

    def shutdown(self, timeout=1.0):
        self._stop_event.set()
        self._thread.join(timeout=max(0.0, timeout))


class FaultCenterDialog(QDialog):
    """Non-modal diagnostic history for both ROS and GUI watchdog events."""

    counts_changed = pyqtSignal(int, int, int)
    attention_requested = pyqtSignal(int)

    def __init__(self, parent=None, log_directory=None, max_history=2000):
        super().__init__(parent, Qt.Window)
        self.setWindowTitle('系统状态与异常中心')
        self.resize(1080, 620)
        self.setMinimumSize(820, 480)
        self.max_history = max(100, int(max_history))
        self.events = []
        self.active_by_key = {}
        self.last_level_by_module = {}
        self._table_dirty = False
        self.log_directory = log_directory or os.path.join(
            os.path.expanduser('~'), 'anav_logs'
        )
        self.log_writer = DiagnosticLogWriter(self.log_directory)
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 14)
        layout.setSpacing(10)

        title = QLabel('系统状态与异常中心')
        title.setStyleSheet('font-size: 20px; font-weight: 700;')
        layout.addWidget(title)

        filters = QHBoxLayout()
        filters.addWidget(QLabel('显示级别：'))
        self.ok_filter = QCheckBox('恢复 / OK')
        self.warn_filter = QCheckBox('WARN')
        self.error_filter = QCheckBox('ERROR')
        self.stale_filter = QCheckBox('STALE / 失联')
        self.ok_filter.setChecked(False)
        self.warn_filter.setChecked(True)
        self.error_filter.setChecked(True)
        self.stale_filter.setChecked(True)
        for checkbox in (
            self.ok_filter, self.warn_filter, self.error_filter,
            self.stale_filter,
        ):
            checkbox.toggled.connect(self.refresh_table)
            filters.addWidget(checkbox)
        filters.addStretch()
        self.summary_label = QLabel('当前没有未恢复异常')
        filters.addWidget(self.summary_label)
        layout.addLayout(filters)

        self.table = QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(
            ('时间', '级别', '错误码', '模块', '描述', '状态', '次数')
        )
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setVisible(False)
        header = self.table.horizontalHeader()
        header.setStretchLastSection(False)
        for column in (0, 1, 2, 3, 5, 6):
            header.setSectionResizeMode(column, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(4, QHeaderView.Stretch)
        self.table.itemSelectionChanged.connect(self.show_selected_detail)
        layout.addWidget(self.table, 3)

        self.detail = QPlainTextEdit()
        self.detail.setReadOnly(True)
        self.detail.setPlaceholderText('选择一条记录查看详细原因与处理建议。')
        self.detail.setMaximumBlockCount(500)
        layout.addWidget(self.detail, 2)

        actions = QHBoxLayout()
        self.clear_resolved_button = QPushButton('清除已恢复记录')
        self.clear_resolved_button.clicked.connect(self.clear_resolved)
        self.export_button = QPushButton('导出记录')
        self.export_button.clicked.connect(self.export_events)
        close_button = QPushButton('关闭')
        close_button.clicked.connect(self.hide)
        actions.addWidget(self.clear_resolved_button)
        actions.addWidget(self.export_button)
        actions.addStretch()
        actions.addWidget(close_button)
        layout.addLayout(actions)

    def ingest_array(self, message):
        self.ingest_events(diagnostic_events_from_array(message))

    def ingest_events(self, events):
        changed = False
        attention_level = None
        for event in events:
            event_changed, event_attention = self._apply_condition(**event)
            changed = changed or event_changed
            if event_attention is not None:
                attention_level = event_attention
        if changed:
            self._finish_model_change()
        if attention_level is not None:
            self.attention_requested.emit(attention_level)

    def report_condition(self, module, code, level, message, active=True,
                         detail='', action='', kind='FAULT', values=None,
                         timestamp=None):
        changed, attention_level = self._apply_condition(
            module=module, code=code, level=level, message=message,
            active=active, detail=detail, action=action, kind=kind,
            values=values, timestamp=timestamp,
        )
        if changed:
            self._finish_model_change()
        if attention_level is not None:
            self.attention_requested.emit(attention_level)
        return changed

    def _apply_condition(self, module, code, level, message, active=True,
                         detail='', action='', kind='FAULT', values=None,
                         timestamp=None):
        timestamp = time.time() if timestamp is None else timestamp
        values = dict(values or {})
        key = (module, code)

        if not active or level == LEVEL_OK:
            recovered = []
            for active_key, event_index in list(self.active_by_key.items()):
                if active_key[0] != module:
                    continue
                event = self.events[event_index]
                event['active'] = False
                event['resolved_at'] = timestamp
                recovered.append(event)
                del self.active_by_key[active_key]
            if recovered:
                self._append_event({
                    'timestamp': timestamp,
                    'last_seen': timestamp,
                    'level': LEVEL_OK,
                    'code': code,
                    'module': module,
                    'message': message,
                    'detail': detail,
                    'action': action,
                    'kind': 'RECOVERY',
                    'active': False,
                    'occurrences': 1,
                    'values': values,
                    'resolved_at': timestamp,
                }, persist=True)
            self.last_level_by_module[module] = LEVEL_OK
            return bool(recovered), None

        existing_index = self.active_by_key.get(key)
        if existing_index is not None:
            event = self.events[existing_index]
            event['last_seen'] = timestamp
            event['detail'] = detail or event['detail']
            event['values'].update(values)
            return False, None

        # A DiagnosticStatus name represents one current state. When its code
        # changes, close the old state before opening the new one.
        for active_key, event_index in list(self.active_by_key.items()):
            if active_key[0] == module:
                self.events[event_index]['active'] = False
                self.events[event_index]['resolved_at'] = timestamp
                del self.active_by_key[active_key]

        previous_occurrences = max(
            (event['occurrences'] for event in self.events
             if event['module'] == module and event['code'] == code),
            default=0,
        )
        event = {
            'timestamp': timestamp,
            'last_seen': timestamp,
            'level': level,
            'code': code,
            'module': module,
            'message': message,
            'detail': detail,
            'action': action,
            'kind': kind,
            'active': True,
            'occurrences': previous_occurrences + 1,
            'values': values,
            'resolved_at': None,
        }
        self.events.append(event)
        self.active_by_key[key] = len(self.events) - 1
        self.last_level_by_module[module] = level
        self._persist_event(event)
        attention = level if level in (LEVEL_ERROR, LEVEL_STALE) else None
        return True, attention

    def _append_event(self, event, persist=False):
        self.events.append(event)
        if persist:
            self._persist_event(event)

    def _persist_event(self, event):
        self.log_writer.submit(event)

    def _finish_model_change(self):
        self._trim_history()
        self._emit_counts()
        if self.isVisible():
            self.refresh_table()
        else:
            self._table_dirty = True

    def _trim_history(self):
        if len(self.events) <= self.max_history:
            return

        active_indices = set(self.active_by_key.values())
        if len(active_indices) >= self.max_history:
            keep_indices = set(
                sorted(active_indices)[-self.max_history:]
            )
        else:
            resolved_indices = [
                index for index in range(len(self.events))
                if index not in active_indices
            ]
            resolved_budget = self.max_history - len(active_indices)
            keep_indices = active_indices.union(
                resolved_indices[-resolved_budget:]
            )

        self.events = [
            event for index, event in enumerate(self.events)
            if index in keep_indices
        ]
        self._rebuild_active_index()

    def _rebuild_active_index(self):
        self.active_by_key = {
            (event['module'], event['code']): index
            for index, event in enumerate(self.events)
            if event['active']
        }

    def showEvent(self, event):
        super().showEvent(event)
        if self._table_dirty:
            self.refresh_table()

    def shutdown(self):
        self.log_writer.shutdown()

    def selected_levels(self):
        levels = set()
        if self.ok_filter.isChecked():
            levels.add(LEVEL_OK)
        if self.warn_filter.isChecked():
            levels.add(LEVEL_WARN)
        if self.error_filter.isChecked():
            levels.add(LEVEL_ERROR)
        if self.stale_filter.isChecked():
            levels.add(LEVEL_STALE)
        return levels

    def refresh_table(self):
        if not hasattr(self, 'table'):
            return
        self._table_dirty = False
        selected_levels = self.selected_levels()
        visible = [
            (index, event) for index, event in enumerate(self.events)
            if event['level'] in selected_levels
        ]
        visible.reverse()
        self.table.setRowCount(len(visible))
        for row, (event_index, event) in enumerate(visible):
            values = (
                time.strftime('%Y-%m-%d %H:%M:%S',
                              time.localtime(event['timestamp'])),
                LEVEL_NAMES.get(event['level'], str(event['level'])),
                event['code'], event['module'], event['message'],
                '未恢复' if event['active'] else '已恢复',
                str(event['occurrences']),
            )
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setData(Qt.UserRole, event_index)
                if column == 1:
                    item.setForeground(
                        LEVEL_COLORS.get(event['level'], QColor('#344054'))
                    )
                self.table.setItem(row, column, item)
        self.table.resizeRowsToContents()

    def show_selected_detail(self):
        rows = self.table.selectionModel().selectedRows()
        if not rows:
            self.detail.clear()
            return
        item = self.table.item(rows[0].row(), 0)
        if item is None:
            return
        event = self.events[item.data(Qt.UserRole)]
        lines = [
            f"时间：{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(event['timestamp']))}",
            f"最后更新：{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(event['last_seen']))}",
            f"级别：{LEVEL_NAMES.get(event['level'], event['level'])}",
            f"错误码：{event['code']}",
            f"模块：{event['module']}",
            f"状态：{'未恢复' if event['active'] else '已恢复'}",
            f"说明：{event['message']}",
        ]
        if event['detail']:
            lines.append(f"详细原因：{event['detail']}")
        if event['action']:
            lines.append(f"处理建议：{event['action']}")
        if event['values']:
            lines.append('')
            lines.append('诊断数据：')
            for key in sorted(event['values']):
                lines.append(f"  {key}: {event['values'][key]}")
        self.detail.setPlainText('\n'.join(lines))

    def _emit_counts(self):
        active_events = (
            self.events[index]
            for index in self.active_by_key.values()
            if 0 <= index < len(self.events)
        )
        counts = {LEVEL_WARN: 0, LEVEL_ERROR: 0, LEVEL_STALE: 0}
        for event in active_events:
            if event['level'] in counts:
                counts[event['level']] += 1
        warn_count = counts[LEVEL_WARN]
        error_count = counts[LEVEL_ERROR]
        stale_count = counts[LEVEL_STALE]
        total = warn_count + error_count + stale_count
        self.summary_label.setText(
            f'未恢复：{total}（WARN {warn_count} / ERROR {error_count} / '
            f'STALE {stale_count}）'
            if total else '当前没有未恢复异常'
        )
        self.counts_changed.emit(warn_count, error_count, stale_count)

    def clear_resolved(self):
        active_events = [event for event in self.events if event['active']]
        self.events = active_events
        self._rebuild_active_index()
        self.detail.clear()
        self._emit_counts()
        if self.isVisible():
            self.refresh_table()
        else:
            self._table_dirty = True

    def export_events(self):
        destination, _selected_filter = QFileDialog.getSaveFileName(
            self, '导出诊断记录',
            os.path.join(os.path.expanduser('~'), 'anav_diagnostics.csv'),
            'CSV 文件 (*.csv);;所有文件 (*)',
        )
        if not destination:
            return
        try:
            with open(destination, 'w', newline='', encoding='utf-8-sig') as handle:
                writer = csv.writer(handle)
                writer.writerow(
                    ('时间', '最后更新', '级别', '错误码', '模块', '描述',
                     '详细原因', '处理建议', '状态', '发生次数')
                )
                for event in self.events:
                    writer.writerow((
                        time.strftime('%Y-%m-%d %H:%M:%S',
                                      time.localtime(event['timestamp'])),
                        time.strftime('%Y-%m-%d %H:%M:%S',
                                      time.localtime(event['last_seen'])),
                        LEVEL_NAMES.get(event['level'], event['level']),
                        event['code'], event['module'], event['message'],
                        event['detail'], event['action'],
                        '未恢复' if event['active'] else '已恢复',
                        event['occurrences'],
                    ))
        except OSError as error:
            QMessageBox.warning(self, '导出失败', str(error))
