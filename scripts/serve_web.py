#!/usr/bin/env python3

import argparse
import functools
import http.server
import json
import pathlib
import subprocess


SERVER_ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_RUNTIME_JSON = SERVER_ROOT / "output" / "web_runtime_pointcloud.json"


def repo_relative_url(path: pathlib.Path, root: pathlib.Path) -> str:
    return "/" + str(path.resolve().relative_to(root.resolve())).replace("\\", "/")


class ViewerRequestHandler(http.server.SimpleHTTPRequestHandler):
    """把根路径重定向到 web 页面，并关闭静态文件缓存。"""

    def do_GET(self):
        if self.path in {"/", ""}:
            self.send_response(302)
            self.send_header("Location", "/web/")
            self.end_headers()
            return
        super().do_GET()

    def do_POST(self):
        if self.path != "/api/convert":
            self.send_error(404, "Unknown API endpoint")
            return

        try:
            content_length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self.send_error(400, "Invalid Content-Length")
            return

        try:
            payload = json.loads(self.rfile.read(content_length).decode("utf-8"))
        except json.JSONDecodeError as exc:
            self.send_error(400, f"Invalid JSON: {exc}")
            return

        pointcloud_path = str(payload.get("pointcloud_path", "")).strip()
        if not pointcloud_path:
            self.send_error(400, "pointcloud_path is required")
            return

        output_path = str(payload.get("output_path", "")).strip()
        if output_path:
            output_file = (SERVER_ROOT / output_path.lstrip("/")).resolve()
        else:
            output_file = DEFAULT_RUNTIME_JSON.resolve()

        voxel = str(payload.get("voxel_leaf_size", "")).strip()
        x_range = str(payload.get("x_range", "")).strip()
        y_range = str(payload.get("y_range", "")).strip()
        z_range = str(payload.get("z_range", "")).strip()
        command = [
            str(SERVER_ROOT / "build" / "pcd2web"),
            "--input",
            pointcloud_path,
            "--output",
            str(output_file),
        ]
        if voxel:
            command.extend(["--voxel", voxel])
        if x_range:
            command.extend(["--x-range", x_range])
        if y_range:
            command.extend(["--y-range", y_range])
        if z_range:
            command.extend(["--z-range", z_range])

        try:
            result = subprocess.run(
                command,
                cwd=str(SERVER_ROOT),
                capture_output=True,
                text=True,
                check=False,
            )
        except OSError as exc:
            self.send_error(500, f"Failed to launch converter: {exc}")
            return

        if result.returncode != 0:
            self.send_response(500)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.end_headers()
            self.wfile.write(
                json.dumps(
                    {
                        "ok": False,
                        "command": command,
                        "stdout": result.stdout,
                        "stderr": result.stderr,
                    }
                ).encode("utf-8")
            )
            return

        response = {
            "ok": True,
            "json_path": repo_relative_url(output_file, SERVER_ROOT),
            "stdout": result.stdout,
            "stderr": result.stderr,
        }
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.end_headers()
        self.wfile.write(json.dumps(response).encode("utf-8"))

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


def main():
    parser = argparse.ArgumentParser(
        description="Serve the repository root for the point cloud web viewer."
    )
    parser.add_argument("--host", default="127.0.0.1", help="Bind host, default: 127.0.0.1")
    parser.add_argument("--port", type=int, default=8080, help="Bind port, default: 8080")
    parser.add_argument(
        "--root",
        default=str(pathlib.Path(__file__).resolve().parents[1]),
        help="Document root, default: repository root",
    )
    args = parser.parse_args()

    root = pathlib.Path(args.root).resolve()
    handler = functools.partial(ViewerRequestHandler, directory=str(root))

    with http.server.ThreadingHTTPServer((args.host, args.port), handler) as server:
        print(f"Serving {root} at http://{args.host}:{args.port}/web/")
        server.serve_forever()


if __name__ == "__main__":
    main()
