"""
test_challenger_satin_2_stress.py — Adversarial Integration & Tooling Stress Test Suite
========================================================================================
Author: Challenger 2 (Integration & Tooling Stress Verifier)
Target:
  - car/tools/hsv_calibration_helper.py
  - car/main.py (/set_lane_color, sys_state lock, PAGE frontend JS contract, /telemetry)
  - car/perception/config.py (LANE_COLOR_MODE default)
"""

import io
import json
import os
import re
import sys
import tempfile
import threading
from typing import Dict, List, Optional, Tuple, Any
from unittest.mock import patch
import pytest
import cv2
import numpy as np

from car.perception.lane_detector import ChartLaneDetector
from car.main import SystemState, DiagnosticHandler, PAGE
from car.perception.config import LANE_COLOR_MODE
import car.tools.hsv_calibration_helper as helper


class MockHttpSocket:
    """Simulates an HTTP socket stream for BaseHTTPRequestHandler."""

    def __init__(self, request_bytes: bytes):
        self.rfile = io.BytesIO(request_bytes)
        self.wfile = io.BytesIO()

    def makefile(self, mode: str, *args, **kwargs):
        return self.rfile if "r" in mode else self.wfile

    def sendall(self, data: bytes):
        self.wfile.write(data)


class MockServer:
    """Mock HTTP server holding sys_state and lane_detector."""

    def __init__(self, state: SystemState, detector: Optional[ChartLaneDetector] = None):
        self.sys_state = state
        self.lane_detector = detector


def dispatch_request(server: MockServer, path: str) -> Tuple[int, Dict[str, str], Optional[Dict[str, Any]], str]:
    """Sends simulated HTTP GET request through DiagnosticHandler."""
    raw_req = f"GET {path} HTTP/1.0\r\nHost: localhost:8080\r\n\r\n".encode("utf-8")
    sock = MockHttpSocket(raw_req)
    DiagnosticHandler(sock, ("127.0.0.1", 8080), server)

    resp_data = sock.wfile.getvalue()
    parts = resp_data.split(b"\r\n\r\n", 1)
    header_block = parts[0].decode("utf-8", errors="replace")
    body = parts[1].decode("utf-8", errors="replace") if len(parts) > 1 else ""

    lines = header_block.splitlines()
    status_code = int(lines[0].split()[1])

    headers = {}
    for line in lines[1:]:
        if ":" in line:
            k, v = line.split(":", 1)
            headers[k.strip().lower()] = v.strip()

    try:
        data = json.loads(body)
    except Exception:
        data = None

    return status_code, headers, data, body


# ==============================================================================
# SECTION 1: Tooling Verification (car/tools/hsv_calibration_helper.py)
# ==============================================================================

def test_hsv_helper_all_4_satin_modes_headless(capsys):
    """Verify headless mode for orange, brown, green, and pink satin ribbon modes."""
    ribbon_modes = ["orange", "brown", "green", "pink"]

    for mode in ribbon_modes:
        test_args = ["hsv_calibration_helper.py", "--headless", "--mode", mode]
        with patch.object(sys, "argv", test_args):
            exit_code = helper.main()
            assert exit_code == 0, f"Helper failed with code {exit_code} on mode {mode}"

        captured = capsys.readouterr().out
        assert "HSV CALIBRATION QUANTITATIVE SUMMARY" in captured
        assert f"Mode:               {mode}" in captured
        assert "MASK EXTRACTION STATISTICS:" in captured
        assert "Raw inRange Mask:" in captured
        assert "After 5x5 Ellipse Dilate (2x):" in captured
        assert "HSV CHANNEL DISTRIBUTION (Active Ribbon Pixels):" in captured
        assert "Hue (0-179)" in captured
        assert "Saturation (0-255)" in captured
        assert "Value (0-255)" in captured


def test_hsv_helper_custom_bounds_parsing_and_analysis(capsys):
    """Verify custom bounds with various format styles."""
    # Format 1: simple comma-separated
    test_args = ["hsv_calibration_helper.py", "--headless", "--mode", "custom", "--lower", "12,65,75", "--upper", "28,215,225"]
    with patch.object(sys, "argv", test_args):
        assert helper.main() == 0

    out1 = capsys.readouterr().out
    assert "Lower HSV Bound:    [12, 65, 75]" in out1
    assert "Upper HSV Bound:    [28, 215, 225]" in out1

    # Format 2: bracketed string format
    test_args2 = ["hsv_calibration_helper.py", "--headless", "--mode", "orange", "--lower", "[8, 125, 85]", "--upper", "[20, 245, 245]"]
    with patch.object(sys, "argv", test_args2):
        assert helper.main() == 0

    out2 = capsys.readouterr().out
    assert "Lower HSV Bound:    [8, 125, 85]" in out2
    assert "Upper HSV Bound:    [20, 245, 245]" in out2


def test_hsv_helper_parse_hsv_triplet_unit_adversarial():
    """Unit test parse_hsv_triplet with adversarial variations."""
    # Valid formats
    assert helper.parse_hsv_triplet("5,120,80") == [5, 120, 80]
    assert helper.parse_hsv_triplet("[5, 120, 80]") == [5, 120, 80]
    assert helper.parse_hsv_triplet("(5, 120, 80)") == [5, 120, 80]
    assert helper.parse_hsv_triplet(" 5 , 120 , 80 ") == [5, 120, 80]
    assert helper.parse_hsv_triplet([5, 120, 80]) == [5, 120, 80]
    assert helper.parse_hsv_triplet((5, 120, 80)) == [5, 120, 80]
    assert helper.parse_hsv_triplet(None) is None

    # Invalid formats should raise ValueError
    with pytest.raises(ValueError):
        helper.parse_hsv_triplet("5,120")
    with pytest.raises(ValueError):
        helper.parse_hsv_triplet("5,120,80,200")
    with pytest.raises(ValueError):
        helper.parse_hsv_triplet("invalid,text,here")
    with pytest.raises(ValueError):
        helper.parse_hsv_triplet("")


def test_hsv_helper_inverted_bounds_produces_zero_match_without_crash(capsys):
    """Stress test inverted bounds (lower > upper) — verify 0 matching pixels and no crash."""
    test_args = ["hsv_calibration_helper.py", "--headless", "--lower", "150,200,200", "--upper", "10,50,50"]
    with patch.object(sys, "argv", test_args):
        assert helper.main() == 0

    out = capsys.readouterr().out
    assert "Raw inRange Mask:                     0 px" in out
    assert "[!] No pixels matched the specified HSV boundaries." in out


def test_hsv_helper_output_saving(capsys):
    """Verify --output writes a valid 4-panel composite image."""
    with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as tf:
        out_path = tf.name

    try:
        test_args = ["hsv_calibration_helper.py", "--headless", "--mode", "pink", "--output", out_path]
        with patch.object(sys, "argv", test_args):
            assert helper.main() == 0

        assert os.path.exists(out_path)
        img = cv2.imread(out_path)
        assert img is not None
        # Height is ROI height 264 (480 * (1 - 0.45)), width is 4 panels * 640 = 2560
        assert img.shape == (264, 2560, 3)
    finally:
        if os.path.exists(out_path):
            os.remove(out_path)


def test_hsv_helper_fallback_without_display_server(capsys):
    """Verify running without --headless when GUI / display server is unavailable falls back gracefully."""
    test_args = ["hsv_calibration_helper.py", "--mode", "green"]
    with patch.object(sys, "argv", test_args), \
         patch.object(cv2, "namedWindow", side_effect=cv2.error("Headless OpenCV error simulated")):
        exit_code = helper.main()
        assert exit_code == 0

    out = capsys.readouterr().out
    assert "GUI initialization failed" in out or "Falling back to headless summary" in out
    assert "HSV CALIBRATION QUANTITATIVE SUMMARY" in out
    assert "Mode:               green" in out


def test_hsv_helper_full_frame_option(capsys):
    """Verify --full-frame analyzes entire 480x640 frame instead of ROI."""
    test_args = ["hsv_calibration_helper.py", "--headless", "--mode", "orange", "--full-frame"]
    with patch.object(sys, "argv", test_args):
        assert helper.main() == 0

    out = capsys.readouterr().out
    assert "Analyzed Area:      ROI shape (480x640), Total Pixels: 307,200" in out


# ==============================================================================
# SECTION 2: Web API Verification (car/main.py /set_lane_color & sys_state lock)
# ==============================================================================

def test_api_rapid_successive_calls():
    """Stress test: 1,000 rapid successive calls switching across all ribbon and legacy colors."""
    sys_state = SystemState()
    detector = ChartLaneDetector(color_mode="orange")
    server = MockServer(sys_state, detector)

    all_colors = ["orange", "brown", "green", "pink", "dark", "yellow", "blue", "white"]

    for i in range(1000):
        color = all_colors[i % len(all_colors)]
        status, headers, data, _ = dispatch_request(server, f"/set_lane_color?color={color}")
        assert status == 200
        assert data["status"] == "ok"
        assert data["color"] == color
        assert sys_state.lane_color_mode == color
        assert detector.color_mode == color


def test_api_query_parameter_variations():
    """Stress test query parsing variations: missing, casing, whitespaces, encodings."""
    sys_state = SystemState()
    detector = ChartLaneDetector(color_mode="dark")
    server = MockServer(sys_state, detector)

    # 1. Missing 'color' query param -> defaults to orange
    status, _, data, _ = dispatch_request(server, "/set_lane_color")
    assert status == 200 and data["color"] == "orange"
    assert sys_state.lane_color_mode == "orange"

    # 2. Blank 'color=' query param (parse_qs produces empty dict without keep_blank_values) -> defaults to orange
    status, _, data, _ = dispatch_request(server, "/set_lane_color?color=")
    assert status == 200 and data["color"] == "orange"
    assert sys_state.lane_color_mode == "orange"

    # 3. Irrelevant query param name -> defaults to orange
    status, _, data, _ = dispatch_request(server, "/set_lane_color?mode=green&foo=bar")
    assert status == 200 and data["color"] == "orange"

    # 4. Uppercase
    status, _, data, _ = dispatch_request(server, "/set_lane_color?color=BROWN")
    assert status == 200 and data["color"] == "brown"
    assert sys_state.lane_color_mode == "brown"

    # 5. Mixed case
    status, _, data, _ = dispatch_request(server, "/set_lane_color?color=pInK")
    assert status == 200 and data["color"] == "pink"
    assert sys_state.lane_color_mode == "pink"

    # 6. URL-encoded whitespace & tabs
    status, _, data, _ = dispatch_request(server, "/set_lane_color?color=%20%20green%20%20")
    assert status == 200 and data["color"] == "green"
    assert sys_state.lane_color_mode == "green"

    # 7. Plus-sign encoded spaces
    status, _, data, _ = dispatch_request(server, "/set_lane_color?color=+orange+")
    assert status == 200 and data["color"] == "orange"
    assert sys_state.lane_color_mode == "orange"

    # 8. Multiple color values -> query.get('color')[0]
    status, _, data, _ = dispatch_request(server, "/set_lane_color?color=pink&color=brown")
    assert status == 200 and data["color"] == "pink"
    assert sys_state.lane_color_mode == "pink"


def test_api_invalid_color_rejection_and_state_preservation():
    """Adversarial stress test: invalid color payloads must return 400 and preserve state."""
    sys_state = SystemState()
    detector = ChartLaneDetector(color_mode="pink")
    server = MockServer(sys_state, detector)

    adversarial_inputs = [
        "purple",
        "red",
        "cyan",
        "123",
        "null",
        "undefined",
        "orange_brown",
        "orange%20brown",
        "<script>alert(1)</script>",
        "ORANGE;DROP",
        "../../etc/passwd",
    ]

    for bad_color in adversarial_inputs:
        # Pre-condition: mode is pink
        sys_state.lane_color_mode = "pink"
        detector.color_mode = "pink"

        status, _, data, body = dispatch_request(server, f"/set_lane_color?color={bad_color}")
        assert status == 400, f"Expected 400 for bad color '{bad_color}', got {status}"
        assert data is not None, f"Failed to parse JSON response for '{bad_color}': body={body}"
        assert data["status"] == "error"
        assert "Invalid lane color mode" in data["message"]

        # Post-condition: state MUST NOT be corrupted
        assert sys_state.lane_color_mode == "pink", f"sys_state corrupted by '{bad_color}'"
        assert detector.color_mode == "pink", f"detector corrupted by '{bad_color}'"


def test_api_concurrency_and_thread_safety():
    """Concurrent stress test: 20 threads simultaneously issuing /set_lane_color requests."""
    sys_state = SystemState()
    detector = ChartLaneDetector(color_mode="orange")
    server = MockServer(sys_state, detector)

    num_threads = 20
    requests_per_thread = 50
    colors = ["orange", "brown", "green", "pink", "yellow", "dark"]

    errors = []

    def worker(thread_idx: int):
        try:
            for i in range(requests_per_thread):
                col = colors[(thread_idx + i) % len(colors)]
                status, _, data, _ = dispatch_request(server, f"/set_lane_color?color={col}")
                if status != 200 or data["status"] != "ok":
                    errors.append(f"Thread {thread_idx} req {i} failed: status {status}")
        except Exception as e:
            errors.append(f"Thread {thread_idx} exception: {e}")

    threads = [threading.Thread(target=worker, args=(t,)) for t in range(num_threads)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=10.0)

    assert not errors, f"Concurrency errors encountered: {errors[:5]}"

    # State must be synchronized and valid
    with sys_state.lock:
        final_state_color = sys_state.lane_color_mode
        final_det_color = detector.color_mode

    assert final_state_color in colors
    assert final_det_color in colors
    assert final_state_color == final_det_color, "Lock contention caused state desynchronization!"


def test_api_telemetry_reports_current_lane_color():
    """Verify /telemetry endpoint reports the active lane_color dynamically."""
    sys_state = SystemState()
    detector = ChartLaneDetector(color_mode="orange")
    server = MockServer(sys_state, detector)

    # Initially orange
    status, _, data, _ = dispatch_request(server, "/telemetry")
    assert status == 200
    assert data["lane_color"] == "orange"

    # Switch to pink
    dispatch_request(server, "/set_lane_color?color=pink")
    status, _, data, _ = dispatch_request(server, "/telemetry")
    assert status == 200
    assert data["lane_color"] == "pink"

    # Switch to brown
    dispatch_request(server, "/set_lane_color?color=brown")
    status, _, data, _ = dispatch_request(server, "/telemetry")
    assert status == 200
    assert data["lane_color"] == "brown"


# ==============================================================================
# SECTION 3: Frontend JavaScript & DOM Contract in PAGE
# ==============================================================================

def test_frontend_js_cycle_lane_color_contract():
    """Verify JavaScript cycleLaneColor function and DOM attribute bindings in PAGE."""
    # 1. HTML button initial structure
    assert 'id="btn-lane-color"' in PAGE
    assert 'onclick="cycleLaneColor()"' in PAGE
    assert 'data-color="orange"' in PAGE
    assert '<span>Lane: ORANGE</span>' in PAGE

    # 2. Extract cycleLaneColor definition
    fn_match = re.search(r"function\s+cycleLaneColor\s*\(\)\s*\{([^}]+(?:\{[^}]*\}[^}]*)*)\}", PAGE)
    assert fn_match is not None, "cycleLaneColor function not found in PAGE"
    fn_body = fn_match.group(1)

    # 3. Extract colors array
    array_match = re.search(r"const\s+colors\s*=\s*\[([^\]]+)\]", fn_body)
    assert array_match is not None, "colors array not defined in cycleLaneColor"
    raw_elements = [x.strip().strip("'\"") for x in array_match.group(1).split(",")]
    assert raw_elements == ["orange", "brown", "green", "pink"], f"Colors sequence mismatch: {raw_elements}"

    # 4. Simulate cycle sequence mathematically:
    # orange -> brown -> green -> pink -> orange
    current = "orange"
    simulated_sequence = []
    for _ in range(8):
        idx = (raw_elements.index(current) + 1) % len(raw_elements)
        next_color = raw_elements[idx]
        simulated_sequence.append((current, next_color))
        current = next_color

    expected_sequence = [
        ("orange", "brown"),
        ("brown", "green"),
        ("green", "pink"),
        ("pink", "orange"),
        ("orange", "brown"),
        ("brown", "green"),
        ("green", "pink"),
        ("pink", "orange"),
    ]
    assert simulated_sequence == expected_sequence

    # 5. Verify status polling sync updates the button
    assert "d.lane_color" in PAGE
    assert "btn.setAttribute('data-color', d.lane_color)" in PAGE or "btn.setAttribute('data-color', d.color)" in PAGE


def test_default_config_consistency():
    """Verify LANE_COLOR_MODE in config.py is 'orange'."""
    assert LANE_COLOR_MODE == "orange"
