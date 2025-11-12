#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, time, math, json, os
import requests
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# ======= 工具函数 =======
def planar_speed_from_odom(msg):
    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    return math.sqrt(vx * vx + vy * vy)

def planar_speed_from_twist(msg):
    vx = msg.linear.x
    vy = msg.linear.y
    return math.sqrt(vx * vx + vy * vy)

# ======= OctoPrint 客户端（带重试与超时） =======
class OctoClient:
    def __init__(self, base_url, api_key, connect_timeout=3.0, read_timeout=12.0, retries=3, backoff=0.5):
        self.base = base_url.rstrip("/")
        self.timeout = (connect_timeout, read_timeout)  # (connect, read)
        self.s = requests.Session()
        # only API key; let requests set Content-Type by json/files automatically
        self.s.headers.update({"X-Api-Key": api_key})

        # retry for transient network errors / timeouts / 50x
        retry = Retry(
            total=retries,
            connect=retries,
            read=retries,
            backoff_factor=backoff,
            status_forcelist=[502, 503, 504],
            allowed_methods=["GET", "POST"]
        )
        adapter = HTTPAdapter(max_retries=retry, pool_maxsize=10)
        self.s.mount("http://", adapter)
        self.s.mount("https://", adapter)

        # 节流：不要每拍都打 /api/job
        self._last_state_ts = 0.0
        self._state_cache = ("unknown", {})
        self.state_min_period = 0.5  # 秒，最短查询周期

    # 安全 GET：失败返回 None
    def _safe_get(self, url):
        try:
            return self.s.get(url, timeout=self.timeout)
        except requests.RequestException as e:
            rospy.logwarn(f"[SpeedGuard] GET {url} failed: {e}")
            return None

    # 安全 POST（JSON）
    def _safe_post_json(self, url, payload):
        try:
            return self.s.post(url, json=payload, timeout=self.timeout)
        except requests.RequestException as e:
            rospy.logwarn(f"[SpeedGuard] POST {url} failed: {e}")
            return None

    # 安全 POST（上传）
    def _safe_post_upload(self, url, files, data):
        try:
            return self.s.post(url, files=files, data=data, timeout=self.timeout)
        except requests.RequestException as e:
            rospy.logwarn(f"[SpeedGuard] UPLOAD {url} failed: {e}")
            return None

    def job_state(self, force=False):
        now = time.time()
        if not force and (now - self._last_state_ts) < self.state_min_period:
            return self._state_cache
        r = self._safe_get(f"{self.base}/api/job")
        self._last_state_ts = now
        if r is None:
            # 保持上次缓存，避免上层崩溃
            return self._state_cache
        if r.status_code != 200:
            rospy.logwarn(f"[SpeedGuard] /api/job http {r.status_code}: {r.text}")
            return self._state_cache
        try:
            data = r.json()
            state = data.get("state", "")
        except Exception as e:
            rospy.logwarn(f"[SpeedGuard] parse /api/job failed: {e}")
            return self._state_cache
        self._state_cache = (state, data)
        return self._state_cache

    def pause(self):
        r = self._safe_post_json(f"{self.base}/api/job", {"command": "pause", "action": "pause"})
        return (r is not None) and (r.status_code in (204, 200))

    def resume(self):
        r = self._safe_post_json(f"{self.base}/api/job", {"command": "pause", "action": "resume"})
        return (r is not None) and (r.status_code in (204, 200))

    def start(self):
        r = self._safe_post_json(f"{self.base}/api/job", {"command": "start"})
        return (r is not None) and (r.status_code in (204, 200))

    def select_local_and_print(self, local_path_under_local):
        url = f"{self.base}/api/files/local/{local_path_under_local}"
        r = self._safe_post_json(url, {"command": "select", "print": True})
        if r is None:
            return False
        if r.status_code not in (204, 200):
            rospy.logwarn(f"[SpeedGuard] select http {r.status_code}: {r.text}")
            return False
        return True

    def upload_local_then_print(self, host_filepath, select_and_print=True):
        if not os.path.isfile(host_filepath):
            rospy.logwarn(f"[SpeedGuard] upload file missing: {host_filepath}")
            return False, None
        with open(host_filepath, "rb") as f:
            files = {"file": (os.path.basename(host_filepath), f)}
            data = {"select": "true", "print": "true"} if select_and_print else {}
            r = self._safe_post_upload(f"{self.base}/api/files/local", files=files, data=data)
        if r is None:
            return False, None
        if r.status_code not in (201, 204, 200):
            rospy.logwarn(f"[SpeedGuard] upload http {r.status_code}: {r.text}")
            return False, None
        rel = None
        try:
            up = r.json()
            rel = up.get("path") or up.get("filename")
        except Exception:
            pass
        if not rel:
            rel = os.path.basename(host_filepath)
        return True, rel

# ======= 主节点 =======
class SpeedGuardAutoStart:
    def __init__(self):
        # === ROS 参数 ===
        self.rate_hz = float(rospy.get_param("~rate_hz", 10.0))
        self.threshold = float(rospy.get_param("~threshold", 0.04))  # m/s
        self.debounce_s = float(rospy.get_param("~debounce", 1.0))

        # 兼容 stable_needed/stable_need
        sn = rospy.get_param("~stable_needed", None)
        if sn is None:
            sn = rospy.get_param("~stable_need", 3)
        self.stable_needed = int(sn)

        self.source = rospy.get_param("~speed_source", "odom")  # 'odom' or 'cmd_vel'
        self.odom_topic = rospy.get_param("~odom_topic", "/kmriiwa/base/state/odom")
        self.cmd_topic = rospy.get_param("~cmd_vel_topic", "/kmriiwa/base/command/cmd_vel")

        # OctoPrint 参数（可调超时）
        base_url = rospy.get_param("~octoprint_url", "http://octopi.local")
        api_key = rospy.get_param("~api_key", "YOUR_API_KEY_HERE")
        ct = float(rospy.get_param("~http_connect_timeout", 3.0))
        rt = float(rospy.get_param("~http_read_timeout", 12.0))
        rtry = int(rospy.get_param("~http_retries", 3))
        backoff = float(rospy.get_param("~http_backoff", 0.5))

        self.auto_start = bool(rospy.get_param("~auto_start", True))
        self.gcode_local = rospy.get_param("~gcode_local_name", "")
        self.gcode_upload = rospy.get_param("~gcode_upload_path", "")

        self.cli = OctoClient(base_url, api_key, connect_timeout=ct, read_timeout=rt,
                              retries=rtry, backoff=backoff)

        # === 状态 ===
        self.speed = 0.0
        self.last_action_t = 0.0
        self.low_stable_cnt = 0
        self.is_paused = False

        if self.source == "odom":
            rospy.Subscriber(self.odom_topic, Odometry, self._cb_odom, queue_size=10)
            rospy.loginfo(f"[SpeedGuard] source=odom topic={self.odom_topic}")
        else:
            rospy.Subscriber(self.cmd_topic, Twist, self._cb_twist, queue_size=10)
            rospy.loginfo(f"[SpeedGuard] source=cmd_vel topic={self.cmd_topic}")

    def _cb_odom(self, msg): self.speed = planar_speed_from_odom(msg)
    def _cb_twist(self, msg): self.speed = planar_speed_from_twist(msg)

    def _too_soon(self):
        return (time.time() - self.last_action_t) < self.debounce_s

    def _mark_action(self):
        self.last_action_t = time.time()

    # === 打印控制逻辑 ===
    def _ensure_started_if_ready(self):
        # 注意：job_state 内含节流，不会每拍都打 API
        state, _ = self.cli.job_state()
        s = state.lower()

        # 若暂停，则恢复
        if "paused" in s:
            if self.cli.resume():
                rospy.loginfo("[SpeedGuard] resume() issued.")
                self.is_paused = False
                self._mark_action()
            return

        # 若空闲，且允许自动启动
        if not self.auto_start:
            return

        if ("operational" in s) or ("ready" in s) or ("standby" in s) or ("idle" in s):
            if self.gcode_upload:
                ok, rel = self.cli.upload_local_then_print(self.gcode_upload, select_and_print=True)
                if ok:
                    rospy.loginfo(f"[SpeedGuard] uploaded+started printing: {rel}")
                    self._mark_action()
                else:
                    rospy.logwarn("[SpeedGuard] upload failed; cannot start.")
            elif self.gcode_local:
                ok = self.cli.select_local_and_print(self.gcode_local)
                if ok:
                    rospy.loginfo(f"[SpeedGuard] selected+started local: {self.gcode_local}")
                    self._mark_action()
                else:
                    rospy.logwarn("[SpeedGuard] select+print failed; check file name.")
            else:
                rospy.logwarn("[SpeedGuard] auto_start enabled but no gcode provided.")

    def _pause_if_printing(self):
        state, _ = self.cli.job_state()
        s = state.lower()
        if any(k in s for k in ["printing", "sd printing", "paused and printing"]):
            if self.cli.pause():
                rospy.loginfo(f"[SpeedGuard] pause() issued; speed={self.speed:.3f} > {self.threshold:.3f}")
                self.is_paused = True
                self._mark_action()

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("[SpeedGuard] started.")
        while not rospy.is_shutdown():
            if self._too_soon():
                rate.sleep()
                continue

            if self.speed > self.threshold:
                self.low_stable_cnt = 0
                self._pause_if_printing()
            else:
                self.low_stable_cnt += 1
                if self.low_stable_cnt >= self.stable_needed:
                    self._ensure_started_if_ready()
                    self.low_stable_cnt = 0

            rate.sleep()

# ======= 主入口 =======
if __name__ == "__main__":
    rospy.init_node("speed_guard_autostart")
    node = SpeedGuardAutoStart()
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass