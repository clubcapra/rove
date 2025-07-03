#!/usr/bin/env python3
import os, time, rclpy
from rclpy.node import Node

class WifiRoamerNode(Node):
    def __init__(self):
        super().__init__("wifi_roamer_node")

        # ── ROS 2 parameters ────────────────────────────────────────────────────
        self.declare_parameter("interface",       "wlan0")
        self.declare_parameter("target_ssid",     "MyWiFi")
        self.declare_parameter("wifi_password",   "")
        self.declare_parameter("signal_threshold", -70)   # dBm
        self.declare_parameter("check_period",     5.0)   # seconds
        self.declare_parameter("host_2_ping", "8.8.8.8")
        # ----------------------------------------------------------------------

        self.interface        = self.get_parameter("interface").value
        self.target_ssid      = self.get_parameter("target_ssid").value
        self.wifi_password    = self.get_parameter("wifi_password").value
        self.signal_threshold = self.get_parameter("signal_threshold").value
        self.check_period     = self.get_parameter("check_period").value
        self.host_2_ping      = self.get_parameter("host_2_ping").value

        self.get_logger().info(
            f"Started wifi_roamer_node on {self.interface}, targeting "
            f"'{self.target_ssid}', threshold {self.signal_threshold} dBm, "
            f"checking every {self.check_period}s"
        )
        self.timer = self.create_timer(self.check_period, self.check_wifi)

    # ───────────────────────── helpers ────────────────────────────────────────
    def run_cmd(self, cmd: str):
        proc = os.popen(cmd)
        out  = proc.read().strip()
        code = proc.close() or 0          # os.popen returns None on success
        return out, code

    def current_ssid(self):
        out, _ = self.run_cmd(
            "nmcli -t -f IN-USE,SSID dev wifi | grep '^*' | cut -d: -f2"
        )
        self.get_logger().debug(f"SSID query -> '{out}'")
        return out or None

    def current_signal_dbm(self):
        # Get SIGNAL% → convert to dBm (approx: quality/2 - 100)
        out, _ = self.run_cmd(
            "nmcli -t -f IN-USE,SIGNAL dev wifi | grep '^*' | cut -d: -f2"
        )
        if not out:
            return None
        try:
            quality = int(out)           # 0‑100
            return int(quality / 2 - 100)
        except ValueError:
            return None
    # -------------------------------------------------------------------------

    def reconnect(self):
        self.get_logger().warn("Re‑associating to target SSID …")

        # ① Disconnect (ignore errors)
        self.run_cmd(f"sudo nmcli dev disconnect {self.interface}")
        time.sleep(1)

        # ② Connect
        pw_arg = f" password '{self.wifi_password}'" if self.wifi_password else ""
        cmd = (f"sudo nmcli -s dev wifi connect '{self.target_ssid}' "
               f"ifname {self.interface}{pw_arg}")
        out, code = self.run_cmd(cmd)
        if code != 0:
            self.get_logger().error(
                f"nmcli connect failed (code {code}): {out or '<no output>'}"
            )
            return

        self.get_logger().info(f"nmcli connected: {out or '<no output>'}")

        # ③ Wait a little for link and DHCP; abort after 6 s
        for _ in range(6):
            if self.current_ssid() == self.target_ssid:
                break
            time.sleep(1)

    # ───────────────────────── main loop ──────────────────────────────────────
    def check_wifi(self):
        ssid = self.current_ssid()
        if ssid != self.target_ssid:
            self.get_logger().warn(f"Not connected or wrong SSID: {ssid}")
            self.reconnect()
            return

        sig = self.current_signal_dbm()
        if sig is None:
            self.get_logger().warn("Cannot read signal strength")
            self.reconnect()
            return

        self.get_logger().info(f"Connected to {ssid} at {sig} dBm")

        # Added: one ping to verify connectivity
        ping_out, ping_code = self.run_cmd(f"ping -c 1 -W 1 {self.host_2_ping}")
        if ping_code == 0:
            self.get_logger().info("Ping successful: network connectivity confirmed")
        else:
            self.get_logger().warn("Ping failed: possible connectivity issue, reconnecting")
            self.reconnect()
            return

        if sig < self.signal_threshold:
            self.get_logger().warn(f"Weak signal ({sig} dBm) → reconnect")
            self.reconnect()

def main(args=None):
    rclpy.init(args=args)
    node = WifiRoamerNode()
    rclpy.spin(node)
    node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
