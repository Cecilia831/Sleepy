#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mac 端 环境与生命体征 接收服务器  (server.py)
============================================
功能：
  接收 ESP32-S3 通过 HTTP GET 上报的数据：
      temp(温度)  hum(湿度)  resp(呼吸频率,次/分钟)  move(身体移动 1/0)
  追加写入 temp.csv（带本地时间戳）。

端口：
  - 使用 8088（与 ESP32 固件 main.cpp 中 PORT=8088 一致）。
  - 若提示 Address already in use，先执行：  lsof -i :8088  处理占用。

启动方式（Mac 终端）：
  python3 server.py
  保持此终端不关闭，ESP32 每 5 秒上报一次数据。

生成文件 temp.csv，形如：
  时间,温度(C),湿度(%),呼吸(次/分),身体移动
  2025-01-01 10:30:00,24.8,73.6,16.0,0
"""

import http.server
import urllib.parse
import datetime
import csv
import os
import socket

HOST = "0.0.0.0"            # 监听所有网卡，允许局域网内 ESP32 接入
PORT = 8088                 # 与 ESP32 端一致
CSV_FILE = "temp.csv"       # 数据输出文件


def get_local_ip():
    """获取本机局域网 IP（用于提示核对）。"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("172.20.10.6", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "未检测到(可执行 ifconfig en0 查看)"


def get_param(q, *names, default=""):
    """从查询字典中按多个候选字段名取值。"""
    for n in names:
        if n in q and q[n]:
            return q[n][0]
    return default


class DataHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        """处理 ESP32 发来的：
        GET /log?temp=24.8&hum=73.6&resp=16.0&move=0
        """
        try:
            parsed = urllib.parse.urlparse(self.path)

            if parsed.path == "/log":
                q = urllib.parse.parse_qs(parsed.query)

                temp = get_param(q, "temp", "temperature")
                hum  = get_param(q, "hum", "humidity")
                resp = get_param(q, "resp", "respiration")
                move = get_param(q, "move", "movement", default="0")

                if temp == "" or hum == "":
                    self._reply(400, "missing param")
                    return

                # 身体移动：显示为 是/否
                move_str = "是" if move == "1" else "否"

                ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                # 追加写入 CSV（首次自动建表头）
                file_exists = os.path.isfile(CSV_FILE)
                with open(CSV_FILE, "a", newline="", encoding="utf-8") as f:
                    w = csv.writer(f)
                    if not file_exists:
                        w.writerow(["时间", "温度(C)", "湿度(%)", "呼吸(次/分)", "身体移动"])
                    w.writerow([ts, temp, hum, resp, move_str])

                print(f"[收到] {ts}  温度={temp}C 湿度={hum}% 呼吸={resp}/分 移动={move_str}")
                self._reply(200, "OK")
            else:
                self._reply(404, "not found")
        except Exception as e:
            print("[错误]", repr(e))
            self._reply(500, "error")

    def _reply(self, code, text):
        self.send_response(code)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(text.encode("utf-8"))

    def log_message(self, fmt, *args):
        # 关闭默认访问日志，保持终端清爽
        pass


if __name__ == "__main__":
    local_ip = get_local_ip()
    print("=" * 56)
    print("  环境与生命体征接收服务器  (server.py)")
    print("-" * 56)
    print(f"  本机局域网 IP : {local_ip}")
    print(f"  监听地址     : {HOST}:{PORT}")
    print(f"  输出文件     : {CSV_FILE}")
    print("-" * 56)
    print("  请确认：本机 IP 是否为 172.20.10.6（ESP32 的目标地址）")
    print("  等待 ESP32 上报数据...  (Ctrl+C 停止)")
    print("=" * 56)

    try:
        server = http.server.ThreadingHTTPServer((HOST, PORT), DataHandler)
    except OSError as e:
        print(f"[错误] 端口 {PORT} 被占用: {e}")
        print(f"  请执行:  lsof -i :{PORT}  找到占用进程并 kill，然后重试。")
        raise SystemExit(1)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n服务器已停止")
        server.server_close()
