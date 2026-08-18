"""ANAV GUI 的独立状态与异常中心。"""

import csv
import json
import os
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


class FaultCenterDialog(QDialog):
    """Non-modal diagnostic history for both ROS and GUI watchdog events."""

    counts_changed = pyqtSignal(int, int, int)
    attention_requested = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent, Qt.Window)
        self.setWindowTitle('系统状态与异常中心')
        self.resize(1080, 620)
        self.setMinimumSize(820, 480)
        self.events = []
        self.active_by_key = {}
        self.last_level_by_module = {}
        self._build_ui()

        self.log_directory = os.path.join(
            os.path.expanduser('~'), 'anav_logs'
        )

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
        stamp = time.time()
        try:
            if message.header.stamp.to_sec() > 0:
                stamp = message.header.stamp.to_sec()
        except (AttributeError, TypeError):
            pass
        for status in message.status:
            values = {item.key: item.value for item in status.values}
            code = values.get('code', 'ANAV-SYS-999')
            active = _truthy(
                values.get('active'), status.level != LEVEL_OK
            )
            self.report_condition(
                module=status.name or 'unknown',
                code=code,
                level=int(status.level),
                message=status.message or values.get('state', code),
                active=active,
                detail=values.get('detail', values.get('reason', '')),
                action=values.get('action', ''),
                kind=values.get('kind', 'FAULT'),
                values=values,
                timestamp=stamp,
            )

    def report_condition(self, module, code, level, message, active=True,
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
            self.refresh_table()
            self._emit_counts()
            return

        existing_index = self.active_by_key.get(key)
        if existing_index is not None:
            event = self.events[existing_index]
            event['last_seen'] = timestamp
            event['detail'] = detail or event['detail']
            event['values'].update(values)
            return

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
        self.refresh_table()
        self._emit_counts()
        if level in (LEVEL_ERROR, LEVEL_STALE):
            self.attention_requested.emit(level)

    def _append_event(self, event, persist=False):
        self.events.append(event)
        if persist:
            self._persist_event(event)

    def _persist_event(self, event):
        try:
            os.makedirs(self.log_directory, exist_ok=True)
            date = time.strftime('%Y%m%d', time.localtime(event['timestamp']))
            path = os.path.join(
                self.log_directory, f'diagnostics_{date}.jsonl'
            )
            serializable = dict(event)
            with open(path, 'a', encoding='utf-8') as handle:
                handle.write(json.dumps(
                    serializable, ensure_ascii=False, sort_keys=True
                ) + '\n')
        except OSError as error:
            print(f'Unable to persist diagnostic event: {error}')

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
        warn_count = sum(
            event['active'] and event['level'] == LEVEL_WARN
            for event in self.events
        )
        error_count = sum(
            event['active'] and event['level'] == LEVEL_ERROR
            for event in self.events
        )
        stale_count = sum(
            event['active'] and event['level'] == LEVEL_STALE
            for event in self.events
        )
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
        self.active_by_key = {
            (event['module'], event['code']): index
            for index, event in enumerate(self.events)
        }
        self.detail.clear()
        self.refresh_table()
        self._emit_counts()

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
