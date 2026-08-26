#!/usr/bin/env python3

import csv
import json
import math
import os
import tempfile
import threading
import time


def _json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def atomic_write_json(path, value):
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix='.{}.'.format(os.path.basename(path)), suffix='.tmp', dir=directory,
        text=True,
    )
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as handle:
            json.dump(
                _json_safe(value), handle, ensure_ascii=False, indent=2,
                sort_keys=True, allow_nan=False,
            )
            handle.write('\n')
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        try:
            directory_fd = os.open(directory, os.O_DIRECTORY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        except (AttributeError, OSError):
            pass
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


class DurableCsv:
    """Append-only CSV with frequent flush and bounded-loss fsync."""

    def __init__(self, path, fieldnames, sync_interval=1.0):
        self.path = path
        self.fieldnames = list(fieldnames)
        self.sync_interval = max(0.1, float(sync_interval))
        self._lock = threading.Lock()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        existing = os.path.isfile(path) and os.path.getsize(path) > 0
        self._handle = open(path, 'a', newline='', encoding='utf-8')
        self._writer = csv.DictWriter(
            self._handle, fieldnames=self.fieldnames, extrasaction='ignore'
        )
        if not existing:
            self._writer.writeheader()
            self._handle.flush()
            os.fsync(self._handle.fileno())
        self._last_sync = time.monotonic()

    def append(self, row, force_sync=False):
        with self._lock:
            self._writer.writerow({key: row.get(key, '') for key in self.fieldnames})
            self._handle.flush()
            now = time.monotonic()
            if force_sync or now - self._last_sync >= self.sync_interval:
                os.fsync(self._handle.fileno())
                self._last_sync = now

    def sync(self):
        with self._lock:
            self._handle.flush()
            os.fsync(self._handle.fileno())
            self._last_sync = time.monotonic()

    def close(self):
        with self._lock:
            if self._handle.closed:
                return
            self._handle.flush()
            os.fsync(self._handle.fileno())
            self._handle.close()
