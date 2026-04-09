"""Flask web server for Mobot telemetry and control.

Can be started two ways:

1. Daemon thread from main.py:
       ws = WebServer(shared_state, config_path, route_path, static_dir)
       ws.set_servo(servo_control)
       ws.start()

2. Standalone (no running robot):
       python3 web_server.py --standalone \\
           --config ../config/params.yaml \\
           --route  ../config/route.yaml

Endpoints:
  GET  /                      — index.html
  GET  /diagnostics           — diagnostics.html
  GET  /route                 — route_editor.html
  GET  /debug                 — debug.html
  GET  /servo                 — servo_tuner.html
  GET  /api/state             — SSE stream at 20 Hz
  GET  /api/state/snapshot    — single JSON snapshot
  GET  /api/history           — last N rows from most recent CSV in logs/
  GET  /api/params            — current params.yaml as JSON
  POST /api/params            — write new params.yaml, set reload flag
  GET  /api/route             — current route.yaml as JSON
  POST /api/route             — write new route.yaml atomically
  GET  /api/map               — logs/map_*.json contents or null
  POST /api/servo_nudge       — {"delta_us": int} — live servo trim
  POST /api/restart           — SIGTERM self → systemd restarts
  POST /api/reset_odometry    — set reset_odometry flag in SharedState
  GET  /api/debug/state_log   — last 500 state transitions
  GET  /api/logs              — list of log files in logs/
"""

import argparse
import csv
import glob
import json
import logging
import os
import signal
import sys
import tempfile
import threading
import time
from collections import deque

import yaml

log = logging.getLogger(__name__)

# ── Optional Flask import (not available at import time on bare Pi) ───────────
try:
    from flask import Flask, Response, jsonify, request, send_from_directory
    from flask_cors import CORS
    _FLASK_AVAILABLE = True
except ImportError:
    log.warning("flask / flask-cors not installed — web server unavailable")
    _FLASK_AVAILABLE = False

# ── Paths ─────────────────────────────────────────────────────────────────────
_THIS_DIR   = os.path.dirname(os.path.abspath(__file__))
_STATIC_DIR = os.path.join(_THIS_DIR, "..", "static")
_LOGS_DIR   = os.path.join(_THIS_DIR, "..", "logs")
_CONFIG_DIR = os.path.join(_THIS_DIR, "..", "config")

HISTORY_ROWS   = 500
TRANSITION_CAP = 500
SSE_HZ         = 20
SSE_INTERVAL   = 1.0 / SSE_HZ


# ── Stub SharedState for standalone mode ──────────────────────────────────────

class _StubSharedState:
    """Minimal stand-in when running without the robot stack."""

    def __init__(self):
        self.line_position      = 0.0
        self.sensor_confidence  = 0
        self.sensor_flags       = 0
        self.sensor_raw         = [0] * 16
        self.heading_rad        = 0.0
        self.wheel_velocity_ms  = 0.0
        self.motor_rpm          = 0.0
        self.input_voltage      = 0.0
        self.x                  = 0.0
        self.y                  = 0.0
        self.steering           = 0.0
        self.throttle           = 0.0
        self.state              = "IDLE"
        self.running            = True
        self.reset_odometry     = False
        self._lock              = threading.Lock()

    def get(self, *fields):
        with self._lock:
            return tuple(getattr(self, f, None) for f in fields)

    def set(self, **kwargs):
        with self._lock:
            for k, v in kwargs.items():
                setattr(self, k, v)


def _safe(val, default=0):
    """Return val if not None, else default."""
    return val if val is not None else default


# ── WebServer ─────────────────────────────────────────────────────────────────

class WebServer:
    """Flask web server that exposes telemetry + control APIs."""

    def __init__(self, shared_state, config_path: str, route_path: str,
                 static_dir: str | None = None):
        self._state       = shared_state
        self._config_path = os.path.abspath(config_path)
        self._route_path  = os.path.abspath(route_path)
        self._static_dir  = os.path.abspath(static_dir or _STATIC_DIR)
        self._servo       = None                        # set via set_servo()
        self._reload_flag = False
        self._reload_lock = threading.Lock()
        self._transitions: deque = deque(maxlen=TRANSITION_CAP)
        self._app         = None

        # Ensure the reset_odometry attribute exists on the state object
        if self._state is not None and not hasattr(self._state, "reset_odometry"):
            self._state.reset_odometry = False

        if _FLASK_AVAILABLE:
            self._build_app()

    # ── Public interface ──────────────────────────────────────────────────────

    def start(self):
        """Start Flask in a background daemon thread."""
        if not _FLASK_AVAILABLE:
            log.error("Flask not available — web server not started")
            return
        t = threading.Thread(target=self._serve, daemon=True, name="web_server")
        t.start()
        log.info("Web server starting on http://0.0.0.0:5000")

    def set_servo(self, servo_control):
        """Provide a live ServoControl reference for real-time nudge."""
        self._servo = servo_control

    def get_reload_flag(self) -> bool:
        """Return True (and clear flag) if params were updated via POST."""
        with self._reload_lock:
            if self._reload_flag:
                self._reload_flag = False
                return True
            return False

    def log_transition(self, from_state: str, to_state: str, trigger: str = ""):
        """Record a state-machine transition for the debug endpoint."""
        self._transitions.append({
            "t":       time.time(),
            "from":    from_state,
            "to":      to_state,
            "trigger": trigger,
        })

    # ── Internal ──────────────────────────────────────────────────────────────

    def _serve(self):
        # Suppress Flask/Werkzeug startup banner and request logs
        logging.getLogger("werkzeug").setLevel(logging.WARNING)
        self._app.run(host="0.0.0.0", port=5000, threaded=True,
                      use_reloader=False, debug=True)

    def _snapshot(self) -> dict:
        """Build a JSON-serialisable state dict. All fields default to 0 if None."""
        s = self._state
        try:
            (line_pos, conf, heading, x, y, sm_state,
             steering, throttle, vel, voltage, flags,
             raw, rpm) = s.get(
                "line_position", "sensor_confidence", "heading_rad",
                "x", "y", "state", "steering", "throttle",
                "wheel_velocity_ms", "input_voltage", "sensor_flags",
                "sensor_raw", "motor_rpm",
            )
        except Exception:
            return {}

        return {
            "line_pos":     _safe(line_pos),
            "confidence":   _safe(conf),
            "heading_rad":  _safe(heading),
            "x":            _safe(x),
            "y":            _safe(y),
            "state":        _safe(sm_state, "UNKNOWN"),
            "steering":     _safe(steering),
            "throttle":     _safe(throttle),
            "wheel_vel_ms": _safe(vel),
            "input_voltage":_safe(voltage),
            "sensor_raw":   raw if raw is not None else [0] * 16,
            "sensor_flags": _safe(flags),
            "motor_rpm":    _safe(rpm),
        }

    def _read_yaml_as_dict(self, path: str) -> dict:
        try:
            with open(path) as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            log.error("Failed to read %s: %s", path, e)
            return {}

    def _write_yaml_atomic(self, path: str, data: dict) -> bool:
        """Write YAML to a temp file then atomically rename into place."""
        dir_ = os.path.dirname(path)
        try:
            with tempfile.NamedTemporaryFile("w", dir=dir_, delete=False,
                                             suffix=".tmp") as tf:
                yaml.safe_dump(data, tf, default_flow_style=False,
                               sort_keys=False)
                tmp_path = tf.name
            os.replace(tmp_path, path)
            return True
        except Exception as e:
            log.error("Atomic write to %s failed: %s", path, e)
            return False

    def _latest_csv(self):
        """Return path to most recent run_*.csv in logs/, or None."""
        pattern = os.path.join(_LOGS_DIR, "run_*.csv")
        files   = sorted(glob.glob(pattern))
        return files[-1] if files else None

    # ── Flask app factory ─────────────────────────────────────────────────────

    def _build_app(self):
        app = Flask(__name__, static_folder=None)
        CORS(app)

        static_dir  = self._static_dir
        config_path = self._config_path
        route_path  = self._route_path

        # ── Page routes ───────────────────────────────────────────────────────

        @app.route("/")
        def index():
            return send_from_directory(static_dir, "index.html")

        @app.route("/diagnostics")
        def diagnostics():
            return send_from_directory(static_dir, "diagnostics.html")

        @app.route("/route")
        def route_editor():
            return send_from_directory(static_dir, "route_editor.html")

        @app.route("/debug")
        def debug():
            return send_from_directory(static_dir, "debug.html")

        @app.route("/servo")
        def servo_tuner():
            return send_from_directory(static_dir, "servo_tuner.html")

        # Serve any other static asset (JS, CSS, images…)
        @app.route("/static/<path:filename>")
        def static_file(filename):
            return send_from_directory(static_dir, filename)

        # ── SSE state stream ──────────────────────────────────────────────────

        @app.route("/api/state")
        def state_stream():
            def generate():
                while True:
                    try:
                        data = self._snapshot()
                        yield f"data: {json.dumps(data)}\n\n"
                    except Exception as e:
                        log.debug("SSE error: %s", e)
                        yield "data: {}\n\n"
                    time.sleep(SSE_INTERVAL)

            return Response(generate(),
                            mimetype="text/event-stream",
                            headers={
                                "Cache-Control":               "no-cache",
                                "X-Accel-Buffering":           "no",
                                "Access-Control-Allow-Origin": "*",
                            })

        # ── Snapshot ──────────────────────────────────────────────────────────

        @app.route("/api/state/snapshot")
        def state_snapshot():
            return jsonify(self._snapshot())

        # ── History ───────────────────────────────────────────────────────────

        @app.route("/api/history")
        def history():
            n    = request.args.get("n", HISTORY_ROWS, type=int)
            path = self._latest_csv()
            if path is None:
                return jsonify([])
            try:
                with open(path, newline="") as f:
                    reader  = csv.DictReader(f)
                    rows    = list(reader)
                    records = rows[-n:] if len(rows) > n else rows
                return jsonify(records)
            except Exception as e:
                log.error("history read error: %s", e)
                return jsonify([])

        # ── Params ────────────────────────────────────────────────────────────

        @app.route("/api/params", methods=["GET"])
        def get_params():
            return jsonify(self._read_yaml_as_dict(config_path))

        @app.route("/api/params", methods=["POST"])
        def post_params():
            try:
                # Accept either JSON body or raw YAML body
                if request.is_json:
                    data = request.get_json(force=True)
                else:
                    data = yaml.safe_load(request.data.decode())
                if not isinstance(data, dict):
                    return jsonify({"ok": False, "error": "body must be a mapping"}), 400
            except Exception as e:
                return jsonify({"ok": False, "error": str(e)}), 400

            ok = self._write_yaml_atomic(config_path, data)
            if ok:
                with self._reload_lock:
                    self._reload_flag = True
                log.info("params.yaml updated via web API")
            return jsonify({"ok": ok})

        # ── Route ─────────────────────────────────────────────────────────────

        @app.route("/api/route", methods=["GET"])
        def get_route():
            return jsonify(self._read_yaml_as_dict(route_path))

        @app.route("/api/route", methods=["POST"])
        def post_route():
            try:
                if request.is_json:
                    data = request.get_json(force=True)
                else:
                    data = yaml.safe_load(request.data.decode())
                if not isinstance(data, dict):
                    return jsonify({"ok": False, "error": "body must be a mapping"}), 400
            except Exception as e:
                return jsonify({"ok": False, "error": str(e)}), 400

            ok = self._write_yaml_atomic(route_path, data)
            if ok:
                log.info("route.yaml updated via web API")
            return jsonify({"ok": ok})

        # ── Map ───────────────────────────────────────────────────────────────

        @app.route("/api/map")
        def get_map():
            pattern = os.path.join(_LOGS_DIR, "map_*.json")
            files   = sorted(glob.glob(pattern))
            if not files:
                return jsonify(None)
            try:
                with open(files[-1]) as f:
                    return jsonify(json.load(f))
            except Exception as e:
                log.error("map read error: %s", e)
                return jsonify(None)

        # ── Servo nudge ───────────────────────────────────────────────────────

        @app.route("/api/servo_nudge", methods=["POST"])
        def servo_nudge():
            body = request.get_json(force=True, silent=True) or {}
            delta_us = int(body.get("delta_us", 0))
            if delta_us == 0:
                return jsonify({"ok": False, "error": "delta_us is 0 or missing"}), 400

            # Update center_pw in params (in-memory, does NOT write to file)
            cfg  = self._read_yaml_as_dict(config_path)
            servo_cfg = cfg.setdefault("servo", {})
            old_center = int(servo_cfg.get("center_pw", 1500))
            new_center = old_center + delta_us
            servo_cfg["center_pw"] = new_center

            # Push live to servo if available
            if self._servo is not None:
                try:
                    self._servo._center = new_center
                    self._servo.set_steering(0.0)   # re-apply center
                    log.info("Servo nudge: center_pw %d → %d", old_center, new_center)
                except Exception as e:
                    log.error("Servo nudge hardware error: %s", e)

            return jsonify({"ok": True, "center_pw": new_center})

        # ── Restart ───────────────────────────────────────────────────────────

        @app.route("/api/restart", methods=["POST"])
        def restart():
            log.info("Restart requested via web API — sending SIGTERM")
            # Respond first, then signal after a brief delay
            def _signal():
                time.sleep(0.2)
                os.kill(os.getpid(), signal.SIGTERM)
            threading.Thread(target=_signal, daemon=True).start()
            return jsonify({"ok": True})

        # ── Reset odometry ────────────────────────────────────────────────────

        @app.route("/api/reset_odometry", methods=["POST"])
        def reset_odometry():
            if not hasattr(self._state, "reset_odometry"):
                self._state.reset_odometry = False
            self._state.reset_odometry = True
            log.info("Odometry reset requested via web API")
            return jsonify({"ok": True})

        # ── State transition log ──────────────────────────────────────────────

        @app.route("/api/debug/state_log")
        def state_log():
            return jsonify(list(self._transitions))

        # ── Log file listing ──────────────────────────────────────────────────

        @app.route("/api/logs")
        def list_logs():
            try:
                pattern = os.path.join(_LOGS_DIR, "*.csv")
                files   = sorted(glob.glob(pattern), reverse=True)
                result  = [os.path.basename(f) for f in files]
                return jsonify(result)
            except Exception as e:
                log.error("logs listing error: %s", e)
                return jsonify([])

        self._app = app


# ── Standalone entry point ────────────────────────────────────────────────────

def _standalone():
    parser = argparse.ArgumentParser(description="Mobot web server (standalone)")
    parser.add_argument("--standalone", action="store_true", required=True)
    parser.add_argument("--config", default=os.path.join(_CONFIG_DIR, "params.yaml"))
    parser.add_argument("--route",  default=os.path.join(_CONFIG_DIR, "route.yaml"))
    parser.add_argument("--static", default=_STATIC_DIR)
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    state = _StubSharedState()
    ws    = WebServer(state, args.config, args.route, args.static)

    log.info("Starting standalone web server on http://0.0.0.0:5000")
    log.info("Config : %s", args.config)
    log.info("Route  : %s", args.route)
    log.info("Static : %s", args.static)

    # Run in foreground (blocking)
    logging.getLogger("werkzeug").setLevel(logging.WARNING)
    ws._app.run(host="0.0.0.0", port=5000, threaded=True, use_reloader=False, debug=True)


if __name__ == "__main__":
    _standalone()
