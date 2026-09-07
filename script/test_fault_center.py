import json

import pytest

from fault_center import _json_default


def test_json_default_decodes_ros_byte_fields():
    encoded = json.dumps(
        {'level': b'warn', 'invalid_utf8': bytearray(b'\xff')},
        default=_json_default,
    )
    decoded = json.loads(encoded)
    assert decoded['level'] == 'warn'
    assert decoded['invalid_utf8'] == '\ufffd'


def test_json_default_rejects_unknown_objects():
    with pytest.raises(TypeError):
        _json_default(object())
