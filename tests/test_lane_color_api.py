"""
test_lane_color_api.py — Verification for Web UI & /set_lane_color HTTP API (Requirement 3)
==============================================================================================
Validates:
  1. DiagnosticHandler endpoint /set_lane_color for satin ribbon colors (orange, brown, green, pink)
     and legacy colors (dark, yellow, blue, white).
  2. DiagnosticHandler HTTP 400 rejection on invalid lane color modes.
  3. Default color fallback (?color omitted) defaults to 'orange'.
  4. SystemState.lane_color_mode and ChartLaneDetector.color_mode synchronization under lock.
  5. UI button default attribute and label in PAGE.
  6. Frontend JavaScript cycleLaneColor() sequence and DOM update contract.
  7. Companion live_detect.py parity.
"""

import io
import json
import re
import pytest

from car.perception.lane_detector import ChartLaneDetector
from car.main import SystemState, DiagnosticHandler, PAGE


class MockHttpSocket:
    """In-memory mock socket simulating HTTP client connections."""

    def __init__(self, request_bytes: bytes):
        self.rfile = io.BytesIO(request_bytes)
        self.wfile = io.BytesIO()

    def makefile(self, mode: str, *args, **kwargs):
        return self.rfile if "r" in mode else self.wfile

    def sendall(self, data: bytes):
        self.wfile.write(data)


class MockServer:
    """Mock HTTP server holding sys_state and lane_detector."""

    def __init__(self, state: SystemState, detector: ChartLaneDetector):
        self.sys_state = state
        self.lane_detector = detector


def _send_request(server: MockServer, path: str):
    """Dispatches simulated HTTP GET request through DiagnosticHandler."""
    raw_req = f"GET {path} HTTP/1.0\r\nHost: localhost:8080\r\n\r\n".encode("utf-8")
    sock = MockHttpSocket(raw_req)
    DiagnosticHandler(sock, ("127.0.0.1", 8080), server)

    resp_data = sock.wfile.getvalue()
    parts = resp_data.split(b"\r\n\r\n", 1)
    header_block = parts[0].decode("utf-8", errors="replace")
    body = parts[1] if len(parts) > 1 else b""
    status_code = int(header_block.splitlines()[0].split()[1])
    try:
        data = json.loads(body.decode("utf-8"))
    except Exception:
        data = None
    return status_code, header_block, data


def test_set_lane_color_satin_ribbon_colors_200():
    """Verify /set_lane_color sets orange, brown, green, pink on sys_state and detector."""
    sys_state = SystemState()
    detector = ChartLaneDetector(color_mode="dark")
    server = MockServer(sys_state, detector)

    ribbon_colors = ["orange", "brown", "green", "pink"]
    for color in ribbon_colors:
        status, headers, data = _send_request(server, f"/set_lane_color?color={color}")
        assert status == 200, f"Expected 200 for color {color}, got {status}"
        assert data is not None
        assert data["status"] == "ok"
        assert data["color"] == color
        assert sys_state.lane_color_mode == color
        assert detector.color_mode == color


def test_set_lane_color_legacy_colors_200():
    """Verify backward compatibility: legacy colors (dark, yellow, blue, white) are accepted."""
    sys_state = SystemState()
    detector = ChartLaneDetector(color_mode="orange")
    server = MockServer(sys_state, detector)

    legacy_colors = ["dark", "yellow", "blue", "white"]
    for color in legacy_colors:
        status, headers, data = _send_request(server, f"/set_lane_color?color={color}")
        assert status == 200
        assert data["status"] == "ok"
        assert data["color"] == color
        assert sys_state.lane_color_mode == color
        assert detector.color_mode == color


def test_set_lane_color_invalid_returns_400():
    """Verify /set_lane_color returns HTTP 400 and does NOT alter state when an invalid color is sent."""
    sys_state = SystemState()
    detector = ChartLaneDetector(color_mode="orange")
    server = MockServer(sys_state, detector)

    status, headers, data = _send_request(server, "/set_lane_color?color=invalid")
    assert status == 400
    assert data is not None
    assert data["status"] == "error"
    assert "Invalid lane color mode 'invalid'" in data["message"]
    # State remains unchanged
    assert sys_state.lane_color_mode == "orange"
    assert detector.color_mode == "orange"


def test_set_lane_color_default_fallback_orange():
    """Verify /set_lane_color with no color query param defaults to orange."""
    sys_state = SystemState()
    detector = ChartLaneDetector(color_mode="dark")
    server = MockServer(sys_state, detector)

    status, headers, data = _send_request(server, "/set_lane_color")
    assert status == 200
    assert data["status"] == "ok"
    assert data["color"] == "orange"
    assert sys_state.lane_color_mode == "orange"
    assert detector.color_mode == "orange"


def test_set_lane_color_case_and_whitespace_insensitivity():
    """Verify /set_lane_color normalizes uppercase and whitespace."""
    sys_state = SystemState()
    detector = ChartLaneDetector(color_mode="dark")
    server = MockServer(sys_state, detector)

    status, headers, data = _send_request(server, "/set_lane_color?color=+GREEN+")
    assert status == 200
    assert data["status"] == "ok"
    assert data["color"] == "green"
    assert sys_state.lane_color_mode == "green"
    assert detector.color_mode == "green"


def test_ui_html_button_defaults_in_page():
    """Verify #btn-lane-color in main.py PAGE has data-color='orange' and 'Lane: ORANGE' label."""
    assert 'id="btn-lane-color"' in PAGE
    assert 'data-color="orange"' in PAGE
    assert '<span>Lane: ORANGE</span>' in PAGE


def test_ui_js_cycle_lane_color_in_page():
    """Verify cycleLaneColor() in main.py PAGE defines orange -> brown -> green -> pink sequence."""
    pattern = r"const\s+colors\s*=\s*\[\s*['\"]orange['\"]\s*,\s*['\"]brown['\"]\s*,\s*['\"]green['\"]\s*,\s*['\"]pink['\"]\s*\]"
    assert re.search(pattern, PAGE) is not None, "cycleLaneColor must define ['orange', 'brown', 'green', 'pink']"
    assert "curr = (btn ? btn.getAttribute('data-color') : '') || 'orange'" in PAGE
    assert "btn.setAttribute('data-color', d.color)" in PAGE
    assert "span.innerText = 'Lane: ' + d.color.toUpperCase()" in PAGE


def test_live_detect_parity():
    """Verify live_detect.py consistency with ribbon colors."""
    from car.live_detect import PAGE as LIVE_PAGE, Handler as LiveHandler
    import car.live_detect as ld

    assert 'id="btn-lane-color"' in LIVE_PAGE
    assert 'data-color="orange"' in LIVE_PAGE
    assert '🎨 Lane: Orange' in LIVE_PAGE
    assert "const colors = ['orange', 'brown', 'green', 'pink'];" in LIVE_PAGE

    # Test LiveHandler endpoint
    class MockLiveServer:
        pass

    raw_req = b"GET /set_lane_color?color=pink HTTP/1.0\r\nHost: localhost:8080\r\n\r\n"
    sock = MockHttpSocket(raw_req)
    LiveHandler(sock, ("127.0.0.1", 8080), MockLiveServer())
    resp_data = sock.wfile.getvalue()
    assert b"200 OK" in resp_data
    assert b'"color": "pink"' in resp_data
    assert ld.lane_color_mode == "pink"

    # Test invalid color on LiveHandler returns 400
    raw_req = b"GET /set_lane_color?color=badcolor HTTP/1.0\r\nHost: localhost:8080\r\n\r\n"
    sock = MockHttpSocket(raw_req)
    LiveHandler(sock, ("127.0.0.1", 8080), MockLiveServer())
    resp_data = sock.wfile.getvalue()
    assert b"400 Bad Request" in resp_data
