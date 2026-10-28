"""console_web: serves the operator console's page (share/omni_console/web) over plain HTTP, port 8080.

The page is served by the robot itself, so its ws:// connection to rosbridge on the same host is allowed in every
browser (an https page, like Foxglove's web app, may not open ws:// to the robot, iPad Safari included), and the page
always matches the robot's software. /config.json tells the page the ports. No caching: a rebuilt page shows on reload.
Parameters port (8080), rosbridge_port (9090), foxglove_port (8765), web_dir (default: the package's share).
"""

from __future__ import annotations

import functools
import json
import os
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class Handler(SimpleHTTPRequestHandler):
    config = b"{}"
    logs_dir = None
    extensions_map = {**SimpleHTTPRequestHandler.extensions_map, ".js": "text/javascript",
                      ".mjs": "text/javascript", ".css": "text/css", ".svg": "image/svg+xml"}

    def do_GET(self):
        from urllib.parse import urlparse
        from omni_console.supervisor import serve_logs
        if serve_logs(self, urlparse(self.path), self.logs_dir):          # the Logbook's session logs
            return
        if self.path.split("?")[0] == "/config.json":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(self.config)))
            self.end_headers()
            self.wfile.write(self.config)
            return
        super().do_GET()

    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def log_message(self, fmt, *args):
        pass


class WebNode(Node):
    def __init__(self):
        super().__init__("console_web")
        P = self.declare_parameter
        port = P("port", 8080).value
        web = P("web_dir", "").value or os.path.join(get_package_share_directory("omni_console"), "web")
        cfg = dict(rosbridge_port=P("rosbridge_port", 9090).value, foxglove_port=P("foxglove_port", 8765).value)
        logs = Path(os.path.expanduser(P("state_dir", "~/.omni").value)) / "logs" / "sessions"
        handler = type("ConsoleHandler", (Handler,), {"config": json.dumps(cfg).encode(), "logs_dir": logs})
        try:
            self.httpd = ThreadingHTTPServer(("0.0.0.0", port), functools.partial(handler, directory=web))
        except OSError as e:                         # the supervisor (or another step) already serves the page
            self.httpd = None
            self.get_logger().info(f"port {port} is taken ({e.strerror}): the page is served from there")
            return
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info(f"operator console: http://localhost:{port} (from {web})")


def main():
    rclpy.init()
    node = WebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.httpd is not None:
            node.httpd.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
