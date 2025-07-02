#!/usr/bin/env python3
"""
wifi_roamer_node.py  –  automatic Wi‑Fi roaming helper for ROS 2

Author: 2025‑07‑02
Tested on: Ubuntu 22.04, ROS 2 Humble, NetworkManager 1.42

ROS parameters
--------------
interface_name         (string, default "wlan0")
scan_period_sec        (double,  default 10.0)   – how often to rescan
rssi_threshold_dbm     (int,     default -70)    – if current AP weaker than this, consider roaming
min_delta_dbm          (int,     default 5)      – required improvement over current RSSI
preferred_bssid        (string,  default "")     – optional hard preference (comma‑separated list)

Published topics
----------------
/wifi_roamer/events    (std_msgs/String)
"""
import subprocess
import shlex
import re
import time
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Helpers ──────────────────────────────────────────────────────────────────────
def run(cmd: str) -> str:
    """Run a shell command and return stdout; raise on error."""
    completed = subprocess.run(
        shlex.split(cmd), check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    return completed.stdout.strip()

def parse_nmcli_wifi_list(raw: str) -> List[Tuple[str, str, int]]:
    """
    Parse `nmcli -f BSSID,SSID,SIGNAL dev wifi list` output.
    Returns list of (bssid, ssid, signal_percent) tuples.
    """
    rows = []
    for line in raw.splitlines():
        if not line or line.startswith("BSSID"):  # header
            continue
        parts = re.split(r"\s{2,}", line.strip())
        if len(parts) >= 3:
            bssid, ssid, signal = parts[0], parts[1], int(parts[2])
            rows.append((bssid, ssid, signal))
    return rows

def wifi_signal_to_dbm(signal_percent: int) -> int:
    """Convert NM signal % to rough dBm estimate (linear interpolation)."""
    # 100 % ≈ –30 dBm, ‑100 % ≈ ‑90 dBm
    return int((signal_percent / 100.0) * 60 - 90)

# Node ─────────────────────────────────────────────────────────────────────────
class WifiRoamer(Node):
    def __init__(self):
        super().__init__("wifi_roamer")
        # Parameters
        self.declare_parameter("interface_name", "wlan0")
        self.declare_parameter("scan_period_sec", 10.0)
        self.declare_parameter("rssi_threshold_dbm", -70)
        self.declare_parameter("min_delta_dbm", 10)
        self.declare_parameter("preferred_bssid", "")

        self.iface = self.get_parameter("interface_name").get_parameter_value().string_value
        self.scan_period = self.get_parameter("scan_period_sec").get_parameter_value().double_value
        self.rssi_thr = self.get_parameter("rssi_threshold_dbm").get_parameter_value().integer_value
        self.min_delta = self.get_parameter("min_delta_dbm").get_parameter_value().integer_value
        pref = self.get_parameter("preferred_bssid").get_parameter_value().string_value
        self.preferred_bssids = {b.strip().lower() for b in pref.split(",") if b.strip()}

        # Publisher
        self.pub = self.create_publisher(String, "/wifi_roamer/events", 10)

        # Timer
        self.timer = self.create_timer(self.scan_period, self.tick)
        self.get_logger().info("WifiRoamer node started; scanning every %.1fs" % self.scan_period)

    # ---------------------------------------------------------------------
    def tick(self):
        try:
            current_bssid, current_ssid, current_dbm = self.current_connection()
        except RuntimeError as e:
            self.warn(str(e))
            self.reconnect_if_possible()
            return

        self.debug(f"Current: {current_ssid} {current_bssid} {current_dbm} dBm")

        if current_dbm > self.rssi_thr:
            # Good enough, nothing to do
            return

        best = self.find_better_ap(current_ssid, current_dbm)
        if best:
            best_bssid, best_dbm = best
            self.info(
                f"Roaming: {current_dbm} dBm too weak (<{self.rssi_thr}); "
                f"switching to {best_bssid} at {best_dbm} dBm"
            )
            self.roam_to(best_bssid)

    # ---------------------------------------------------------------------
    # NetworkManager interactions
    def current_connection(self) -> Tuple[str, str, int]:
        """
        Return (bssid, ssid, rssi_dbm) of current connection.
        Raises RuntimeError if not connected or malformed data.
        """
        # Check connection state
        state_line = run(f"nmcli -t -f GENERAL.STATE dev show {self.iface}")

        if not state_line.startswith("GENERAL.STATE:100"):
            raise RuntimeError("Interface disconnected")

        # Get WiFi connection info
        raw = run("nmcli --escape no -t -f active,bssid,ssid,signal dev wifi")
        for line in raw.splitlines():
            if not line.strip().startswith("yes"):
                continue
            parts = line.strip().split(":")
            if len(parts) < 9:
                self.get_logger().warn(f"Malformed active connection line: {line}")
                continue
            try:
                active = parts[0]
                bssid = ":".join(parts[1:7])
                ssid = parts[7]
                signal = int(parts[8])
                dbm = wifi_signal_to_dbm(signal)
                return bssid.lower(), ssid, dbm
            except Exception as e:
                self.get_logger().warn(f"Failed to parse fields: {parts} — {e}")



    def find_better_ap(self, ssid: str, current_dbm: int) -> Optional[Tuple[str, int]]:
        """Return (bssid, dbm) of a suitable roaming target, else None."""
        run(f"nmcli dev wifi rescan ifname {self.iface}")
        time.sleep(1.0)  # brief pause for fresh scan
        scan_raw = run("nmcli -f BSSID,SSID,SIGNAL dev wifi list")
        candidates = [
            (b.lower(), wifi_signal_to_dbm(sig))
            for b, s, sig in parse_nmcli_wifi_list(scan_raw)
            if s == ssid
        ]
        if not candidates:
            return None

        # Prefer manually preferred BSSIDs first
        candidates.sort(key=lambda x: (x[0] not in self.preferred_bssids, -x[1]))
        best_bssid, best_dbm = candidates[0]
        if best_dbm - current_dbm >= self.min_delta and best_dbm > self.rssi_thr:
            return best_bssid, best_dbm
        return None

    def roam_to(self, bssid: str):
        """Ask NetworkManager to roam to the given BSSID."""
        try:
            run(f"nmcli dev wifi connect {bssid} ifname {self.iface}")
        except subprocess.CalledProcessError as e:
            self.warn(f"Roam failed: {e.stderr.strip()}")

    def reconnect_if_possible(self):
        """Try to reconnect when interface is completely disconnected."""
        self.info("Interface disconnected; scanning for known APs…")
        try:
            run(f"nmcli dev wifi rescan ifname {self.iface}")
            scan_raw = run("nmcli -f BSSID,SSID,SIGNAL dev wifi list")
        except subprocess.CalledProcessError as e:
            self.warn(f"Scan failed: {e.stderr.strip()}")
            return

        # Use last known SSID from connection profiles
        try:
            ssid = run("nmcli -t -f name connection show --active")
        except subprocess.CalledProcessError:
            self.warn("No active connection profile found")
            return

        candidates = [
            (b, wifi_signal_to_dbm(sig))
            for b, s, sig in parse_nmcli_wifi_list(scan_raw)
            if s == ssid
        ]
        if not candidates:
            self.warn(f"No APs with SSID '{ssid}' found")
            return

        best_bssid, best_dbm = max(candidates, key=lambda x: x[1])
        self.info(f"Reconnecting to {best_bssid} at {best_dbm} dBm")
        self.roam_to(best_bssid)

    # Convenience logging wrappers
    def info(self, msg: str):
        self.get_logger().info(msg)
        self.pub.publish(String(data=msg))

    def warn(self, msg: str):
        self.get_logger().warn(msg)
        self.pub.publish(String(data="WARN: " + msg))

    def debug(self, msg: str):
        self.get_logger().debug(msg)
        # No spam on /events for debug lines

# Main ─────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = WifiRoamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
