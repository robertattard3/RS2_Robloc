#!/usr/bin/env python3
"""
Interactive test node for UR3e RQT UI.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Pose
import threading
import time
import math

RED    = '\033[91m'
GREEN  = '\033[92m'
YELLOW = '\033[93m'
CYAN   = '\033[96m'
BOLD   = '\033[1m'
RESET  = '\033[0m'

LATENCY_THRESHOLD_MS = 100.0


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert roll/pitch/yaw (radians) to quaternion (x, y, z, w)."""
    cr = math.cos(roll  / 2.0)
    sr = math.sin(roll  / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw   / 2.0)
    sy = math.sin(yaw   / 2.0)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class InteractiveTestNode(Node):

    def __init__(self):
        super().__init__('ur3e_interactive_test')

        # --- Publishers ---
        self.pose_publisher  = self.create_publisher(Pose,    '/tool_pose',       10)
        self.estop_publisher = self.create_publisher(Bool,    '/emergency_stop',  10)
        self.ts_publisher    = self.create_publisher(Float64, '/estop_timestamp', 10)

        # --- Subscribers ---
        self.estop_sub = self.create_subscription(
            Bool, '/emergency_stop', self._estop_callback, 10
        )
        self.timestamp_sub = self.create_subscription(
            Float64, '/estop_timestamp', self._timestamp_callback, 10
        )

        # --- Position state ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        # --- Orientation state (radians) ---
        self.current_roll  = 0.0
        self.current_pitch = 0.0
        self.current_yaw   = 0.0

        # --- General state ---
        self.estop_count   = 0
        self.running       = True
        self.publishing    = True
        self.publish_count = 0

        # Latency tracking
        self._pending_click_time = None
        self._latency_history    = []

        print(f"\n{BOLD}{CYAN}=== UR3e Interactive Test Node ==={RESET}")
        print(f"{GREEN}✔ Publishing to  : /tool_pose{RESET}")
        print(f"{GREEN}✔ Listening on   : /emergency_stop{RESET}")
        print(f"{GREEN}✔ Listening on   : /estop_timestamp{RESET}")
        print(f"\n{YELLOW}--- Position commands ---{RESET}")
        print("  x <value>        — set X  (e.g. x 0.5)")
        print("  y <value>        — set Y  (e.g. y -1.2)")
        print("  z <value>        — set Z  (e.g. z 0.75)")
        print("  xyz <x> <y> <z>  — set all position at once")
        print("  sweepx [start] [end] [steps] [delay_ms]  — sweep X axis")
        print("  sweepy [start] [end] [steps] [delay_ms]  — sweep Y axis")
        print("  sweepz [start] [end] [steps] [delay_ms]  — sweep Z axis")
        print("  sweepall [start] [end] [steps] [delay_ms]")
        print("               — sweep X Y Z simultaneously")
        print(f"\n{YELLOW}--- Orientation commands ---{RESET}")
        print("  roll  <value>    — set roll  in radians (e.g. roll 0.5)")
        print("  pitch <value>    — set pitch in radians (e.g. pitch -0.3)")
        print("  yaw   <value>    — set yaw   in radians (e.g. yaw 1.57)")
        print("  rpy <r> <p> <y>  — set all orientation at once")
        print("  sweeproll  [start] [end] [steps] [delay_ms]  — sweep roll axis")
        print("  sweeppitch [start] [end] [steps] [delay_ms]  — sweep pitch axis")
        print("  sweepyaw   [start] [end] [steps] [delay_ms]  — sweep yaw axis")
        print("  sweeporientation [start] [end] [steps] [delay_ms]")
        print("               — sweep roll pitch yaw simultaneously")
        print(f"\n{YELLOW}--- Other commands ---{RESET}")
        print("  reset            — reset all position and orientation to 0.0")
        print("  estop            — simulate an e-stop publish")
        print("  connect          — resume publishing to /tool_pose")
        print("  disconnect       — stop publishing (triggers NO SIGNAL in UI)")
        print("  latency          — show latency history from UI button presses")
        print("  status           — show current values")
        print("  debug            — show publisher/subscriber counts")
        print("  quit             — exit\n")

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _timestamp_callback(self, msg):
        self._pending_click_time = msg.data

    def _estop_callback(self, msg):
        receive_time = time.time()
        self.estop_count += 1

        if msg.data:
            if self._pending_click_time is not None:
                latency_ms = (receive_time - self._pending_click_time) * 1000.0
                self._latency_history.append(latency_ms)
                self._pending_click_time = None

                status = (
                    f"{GREEN}✔ {latency_ms:.2f}ms"
                    f" (under {LATENCY_THRESHOLD_MS:.0f}ms){RESET}"
                    if latency_ms < LATENCY_THRESHOLD_MS
                    else f"{RED}✘ {latency_ms:.2f}ms"
                    f" (EXCEEDS {LATENCY_THRESHOLD_MS:.0f}ms){RESET}"
                )
                print(
                    f"\n{RED}{BOLD}🚨 E-STOP RECEIVED!"
                    f"  Count: {self.estop_count}"
                    f"  |  Latency: {status}{RESET}"
                )
            else:
                print(
                    f"\n{RED}{BOLD}🚨 E-STOP RECEIVED!"
                    f"  Count: {self.estop_count}"
                    f"  |  Latency: N/A (terminal command){RESET}"
                )
        else:
            print(f"\n{YELLOW}⚠ E-stop cleared (False) received{RESET}")

        self._print_prompt()

    def _publish_loop(self):
        """Runs in its own thread. Publishes pose at 10Hz."""
        time.sleep(0.5)
        print(f"{GREEN}✔ Publish loop started{RESET}")

        while self.running:
            if self.publishing:
                try:
                    qx, qy, qz, qw = euler_to_quaternion(
                        self.current_roll,
                        self.current_pitch,
                        self.current_yaw
                    )
                    msg = Pose()
                    msg.position.x    = float(self.current_x)
                    msg.position.y    = float(self.current_y)
                    msg.position.z    = float(self.current_z)
                    msg.orientation.x = qx
                    msg.orientation.y = qy
                    msg.orientation.z = qz
                    msg.orientation.w = qw
                    self.pose_publisher.publish(msg)
                    self.publish_count += 1
                except Exception as e:
                    print(f"{RED}✘ Publish error: {e}{RESET}")
            time.sleep(0.1)

    # ------------------------------------------------------------------
    # Sweep
    # ------------------------------------------------------------------

    def _run_sweep(self, axis: str, start: float, end: float,
                   steps: int, delay: float):
        """Incrementally moves a single axis from start to end."""
        step_size = (end - start) / steps

        print(
            f"\n{CYAN}Starting sweep on {axis.upper()} axis:"
            f"  {start:.6f} → {end:.6f}"
            f"  over {steps} steps"
            f"  ({delay*1000:.0f}ms per step)"
            f"  total time ~{steps * delay:.1f}s{RESET}"
        )

        for i in range(steps + 1):
            if not self.running:
                break

            value = round(start + (step_size * i), 6)

            if axis == 'x':
                self.current_x = value
            elif axis == 'y':
                self.current_y = value
            elif axis == 'z':
                self.current_z = value
            elif axis == 'roll':
                self.current_roll = value
            elif axis == 'pitch':
                self.current_pitch = value
            elif axis == 'yaw':
                self.current_yaw = value

            progress = int((i / steps) * 20)
            bar = f"[{'█' * progress}{'░' * (20 - progress)}]"
            print(
                f"\r  {bar}  {axis.upper()}: {value:10.6f}  "
                f"step {i:>{len(str(steps))}}/{steps}",
                end='',
                flush=True
            )

            time.sleep(delay)

        print(f"\n{GREEN}✔ Sweep complete{RESET}")
        self._print_prompt()

    def _start_sweep_thread(self, axis, parts, offset=1):
        """Helper to parse sweep parameters and launch a thread."""
        start = float(parts[offset])     if len(parts) > offset     else 0.0
        end   = float(parts[offset + 1]) if len(parts) > offset + 1 else 1.0
        steps = int(parts[offset + 2])   if len(parts) > offset + 2 else 50
        delay = float(parts[offset + 3]) / 1000.0 \
            if len(parts) > offset + 3 else 0.05

        threading.Thread(
            target=self._run_sweep,
            args=(axis, start, end, steps, delay),
            daemon=True
        ).start()

    # ------------------------------------------------------------------
    # Terminal helpers
    # ------------------------------------------------------------------

    def _print_latency_history(self):
        if not self._latency_history:
            print(
                f"\n{YELLOW}No UI button latency data yet"
                f" — press the EMERGENCY STOP button in RQT{RESET}"
            )
            return

        avg      = sum(self._latency_history) / len(self._latency_history)
        mn       = min(self._latency_history)
        mx       = max(self._latency_history)
        all_pass = all(ms < LATENCY_THRESHOLD_MS for ms in self._latency_history)

        print(
            f"\n{CYAN}E-Stop Latency History"
            f" ({len(self._latency_history)} presses):{RESET}"
        )
        for i, ms in enumerate(self._latency_history):
            status = (
                f"{GREEN}✔ PASS{RESET}"
                if ms < LATENCY_THRESHOLD_MS
                else f"{RED}✘ FAIL{RESET}"
            )
            print(f"  Press {i+1:>2}: {ms:6.2f} ms  {status}")

        print(f"\n{CYAN}Summary:{RESET}")
        print(f"  Min      : {mn:.2f} ms")
        print(f"  Max      : {mx:.2f} ms")
        print(f"  Average  : {avg:.2f} ms")
        print(f"  Threshold: {LATENCY_THRESHOLD_MS:.0f} ms")
        print(
            f"  Overall  : {GREEN}{BOLD}PASS ✔{RESET}"
            if all_pass else
            f"  Overall  : {RED}{BOLD}FAIL ✘{RESET}"
        )

    def _print_status(self):
        print(f"\n{CYAN}Position:{RESET}")
        print(f"  X={BOLD}{self.current_x:.6f}{RESET}  "
              f"Y={BOLD}{self.current_y:.6f}{RESET}  "
              f"Z={BOLD}{self.current_z:.6f}{RESET}")
        print(f"{CYAN}Orientation:{RESET}")
        print(f"  Roll={BOLD}{self.current_roll:.6f}{RESET}  "
              f"Pitch={BOLD}{self.current_pitch:.6f}{RESET}  "
              f"Yaw={BOLD}{self.current_yaw:.6f}{RESET}")
        print(f"{CYAN}State:{RESET}  "
              f"E-stops: {BOLD}{self.estop_count}{RESET}  |  "
              f"Publishing: {BOLD}{'YES' if self.publishing else 'NO'}{RESET}")

    def _print_debug(self):
        pose_subs = self.pose_publisher.get_subscription_count()
        print(f"\n{CYAN}Debug info:{RESET}")
        print(f"  /tool_pose subscribers  : {BOLD}{pose_subs}{RESET}"
              f"  {'✔ connected' if pose_subs > 0 else '✘ nobody listening'}")
        print(f"  Total publishes sent    : {BOLD}{self.publish_count}{RESET}")
        print(f"  Publishing active       : "
              f"{BOLD}{'YES' if self.publishing else 'NO'}{RESET}")
        print(f"  Node name               : {self.get_name()}")

    def _print_prompt(self):
        print(f"{YELLOW}> {RESET}", end='', flush=True)

    def _handle_command(self, raw: str) -> bool:
        parts = raw.strip().split()
        if not parts:
            return True

        cmd = parts[0].lower()

        if cmd in ('quit', 'q'):
            return False

        # ── Position ──────────────────────────────────────────────────
        elif cmd == 'x' and len(parts) == 2:
            try:
                self.current_x = float(parts[1])
                print(f"{GREEN}✔ X set to {self.current_x:.6f} m{RESET}")
            except ValueError:
                print(f"{RED}✘ Invalid value: {parts[1]}{RESET}")

        elif cmd == 'y' and len(parts) == 2:
            try:
                self.current_y = float(parts[1])
                print(f"{GREEN}✔ Y set to {self.current_y:.6f} m{RESET}")
            except ValueError:
                print(f"{RED}✘ Invalid value: {parts[1]}{RESET}")

        elif cmd == 'z' and len(parts) == 2:
            try:
                self.current_z = float(parts[1])
                print(f"{GREEN}✔ Z set to {self.current_z:.6f} m{RESET}")
            except ValueError:
                print(f"{RED}✘ Invalid value: {parts[1]}{RESET}")

        elif cmd == 'xyz' and len(parts) == 4:
            try:
                self.current_x = float(parts[1])
                self.current_y = float(parts[2])
                self.current_z = float(parts[3])
                print(
                    f"{GREEN}✔ Position set to "
                    f"X={self.current_x:.6f} "
                    f"Y={self.current_y:.6f} "
                    f"Z={self.current_z:.6f}{RESET}"
                )
            except ValueError:
                print(f"{RED}✘ Invalid values — usage: xyz <x> <y> <z>{RESET}")

        # ── Position sweeps ───────────────────────────────────────────
        elif cmd == 'sweepx':
            self._start_sweep_thread('x', parts)

        elif cmd == 'sweepy':
            self._start_sweep_thread('y', parts)

        elif cmd == 'sweepz':
            self._start_sweep_thread('z', parts)

        elif cmd == 'sweepall':
            start = float(parts[1]) if len(parts) > 1 else 0.0
            end   = float(parts[2]) if len(parts) > 2 else 1.0
            steps = int(parts[3])   if len(parts) > 3 else 50
            delay = float(parts[4]) / 1000.0 if len(parts) > 4 else 0.05

            print(f"{CYAN}Sweeping all position axes simultaneously{RESET}")
            for axis in ('x', 'y', 'z'):
                threading.Thread(
                    target=self._run_sweep,
                    args=(axis, start, end, steps, delay),
                    daemon=True
                ).start()

        # ── Orientation ───────────────────────────────────────────────
        elif cmd == 'roll' and len(parts) == 2:
            try:
                self.current_roll = float(parts[1])
                print(
                    f"{GREEN}✔ Roll set to {self.current_roll:.6f} rad{RESET}"
                )
            except ValueError:
                print(f"{RED}✘ Invalid value: {parts[1]}{RESET}")

        elif cmd == 'pitch' and len(parts) == 2:
            try:
                self.current_pitch = float(parts[1])
                print(
                    f"{GREEN}✔ Pitch set to {self.current_pitch:.6f} rad{RESET}"
                )
            except ValueError:
                print(f"{RED}✘ Invalid value: {parts[1]}{RESET}")

        elif cmd == 'yaw' and len(parts) == 2:
            try:
                self.current_yaw = float(parts[1])
                print(
                    f"{GREEN}✔ Yaw set to {self.current_yaw:.6f} rad{RESET}"
                )
            except ValueError:
                print(f"{RED}✘ Invalid value: {parts[1]}{RESET}")

        elif cmd == 'rpy' and len(parts) == 4:
            try:
                self.current_roll  = float(parts[1])
                self.current_pitch = float(parts[2])
                self.current_yaw   = float(parts[3])
                print(
                    f"{GREEN}✔ Orientation set to "
                    f"Roll={self.current_roll:.6f} "
                    f"Pitch={self.current_pitch:.6f} "
                    f"Yaw={self.current_yaw:.6f}{RESET}"
                )
            except ValueError:
                print(
                    f"{RED}✘ Invalid values — usage: rpy <roll> <pitch> <yaw>{RESET}"
                )

        # ── Orientation sweeps ────────────────────────────────────────
        elif cmd == 'sweeproll':
            self._start_sweep_thread('roll', parts)

        elif cmd == 'sweeppitch':
            self._start_sweep_thread('pitch', parts)

        elif cmd == 'sweepyaw':
            self._start_sweep_thread('yaw', parts)

        elif cmd == 'sweeporientation':
            start = float(parts[1]) if len(parts) > 1 else 0.0
            end   = float(parts[2]) if len(parts) > 2 else 1.0
            steps = int(parts[3])   if len(parts) > 3 else 50
            delay = float(parts[4]) / 1000.0 if len(parts) > 4 else 0.05

            print(f"{CYAN}Sweeping all orientation axes simultaneously{RESET}")
            for axis in ('roll', 'pitch', 'yaw'):
                threading.Thread(
                    target=self._run_sweep,
                    args=(axis, start, end, steps, delay),
                    daemon=True
                ).start()

        # ── Other ─────────────────────────────────────────────────────
        elif cmd == 'reset':
            self.current_x     = 0.0
            self.current_y     = 0.0
            self.current_z     = 0.0
            self.current_roll  = 0.0
            self.current_pitch = 0.0
            self.current_yaw   = 0.0
            print(
                f"{GREEN}✔ All position and orientation reset to 0.0{RESET}"
            )

        elif cmd == 'estop':
            msg = Bool()
            msg.data = True
            self.estop_publisher.publish(msg)
            print(
                f"{RED}🚨 Simulated e-stop published"
                f" (no latency data for terminal commands){RESET}"
            )

        elif cmd == 'disconnect':
            self.publishing = False
            print(
                f"{YELLOW}⚠ Publishing stopped"
                f" — UI should show NO SIGNAL within 1s{RESET}"
            )

        elif cmd == 'connect':
            self.publishing = True
            print(
                f"{GREEN}✔ Publishing resumed — UI should show CONNECTED{RESET}"
            )

        elif cmd == 'latency':
            self._print_latency_history()

        elif cmd == 'status':
            self._print_status()

        elif cmd == 'debug':
            self._print_debug()

        else:
            print(f"{RED}✘ Unknown command: '{raw.strip()}'{RESET}")

        return True

    def run_input_loop(self):
        self._print_status()
        self._print_prompt()

        while self.running:
            try:
                raw = input()
                if not self._handle_command(raw):
                    print(f"\n{CYAN}Shutting down interactive test node.{RESET}")
                    self.running = False
                    break
                self._print_prompt()
            except (EOFError, KeyboardInterrupt):
                self.running = False
                break


def main():
    rclpy.init()
    node = InteractiveTestNode()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    publish_thread = threading.Thread(
        target=node._publish_loop, daemon=True
    )
    publish_thread.start()

    time.sleep(0.3)
    try:
        node.run_input_loop()
    finally:
        node.running = False
        publish_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        print("Goodbye.")


if __name__ == '__main__':
    main()