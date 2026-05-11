import time
import rclpy
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import (
    QWidget, QPushButton, QVBoxLayout, QLabel
)
from python_qt_binding.QtCore import QTimer, pyqtSignal, QObject
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import PoseStamped, PoseArray
import math


class PoseSignalBridge(QObject):
    pose_received      = pyqtSignal(float, float, float, float, float, float)
    goal_received      = pyqtSignal(float, float, float, float, float, float)
    connection_changed = pyqtSignal(bool)
    robot_status_changed = pyqtSignal(bool)  # True = running, False = idle


class UR3eUIPlugin(Plugin):

    CONNECTION_TIMEOUT = 1.0

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('UR3eUIPlugin')
        self.node = rclpy.create_node('ur3e_rqt_ui')

        # Signal bridge
        self.bridge = PoseSignalBridge()
        self.bridge.pose_received.connect(self._update_labels)
        self.bridge.goal_received.connect(self._update_goal_labels)
        self.bridge.connection_changed.connect(self._update_connection_status)
        self.bridge.robot_status_changed.connect(self._update_start_button)

        # Track last message time for connection watchdog
        self._last_msg_time = None
        self._is_connected  = False

        # Start button readiness — both must be true before the button enables
        self._pick_place_ready  = False  # True once pick_place_mtc publishes its first idle status
        self._robot_running     = False  # True while a pick-place is in progress

        # E-stop lock state
        self._estop_locked = False

        # --- Publishers ---
        self.estop_pub = self.node.create_publisher(
            Bool, '/emergency_stop', 10
        )
        self.timestamp_pub = self.node.create_publisher(
            Float64, '/estop_timestamp', 10
        )
        self.start_pub = self.node.create_publisher(
            Bool, '/start_pick_place', 10
        )

        # --- Subscribers ---
        self.pose_sub = self.node.create_subscription(
            PoseStamped,
            '/end_effector_position',
            self.pose_callback,
            10
        )
        self.goal_sub = self.node.create_subscription(
            PoseArray,
            '/robot_goal_poses',
            self.goal_callback,
            10
        )
        self.status_sub = self.node.create_subscription(
            Bool,
            '/pick_place_status',
            self.status_callback,
            10
        )

        # --- UI ---
        self.widget = QWidget()
        layout = QVBoxLayout()
        layout.setSpacing(2)
        layout.setContentsMargins(10, 8, 10, 8)

        # ── Connection status ──────────────────────────────────────────
        self.status_label = QLabel('● NO SIGNAL')
        self.status_label.setStyleSheet(
            'color: red; font-size: 14px; font-weight: bold;'
            'margin-bottom: 4px;'
        )

        # ── Start button (disabled until system is fully ready) ────────
        self.start_button = QPushButton('START PICK & PLACE')
        self.start_button.setFixedHeight(60)
        self.start_button.setEnabled(False)
        self.start_button.setStyleSheet(
            'background-color: #444444; color: #888888; font-size: 16px; font-weight: bold;'
        )
        self.start_button.clicked.connect(self.send_start)

        # ── E-stop button ──────────────────────────────────────────────
        self.estop_button = QPushButton('EMERGENCY STOP')
        self.estop_button.setFixedHeight(80)
        self.estop_button.setStyleSheet(
            'background-color: red; font-size: 20px; margin-bottom: 0px;'
        )
        self.estop_button.clicked.connect(self.send_estop)

        # Unlock button
        self.unlock_button = QPushButton('UNLOCK')
        self.unlock_button.setFixedHeight(40)
        self.unlock_button.setStyleSheet(
            'background-color: grey; color: white;'
            'font-size: 13px; font-weight: bold;'
        )
        self.unlock_button.setEnabled(False)
        self.unlock_button.clicked.connect(self.unlock_estop)

        # ── Current position ───────────────────────────────────────────
        self.position_header = QLabel('Current Position')
        self.position_header.setStyleSheet(
            'font-size: 12px; font-weight: bold; color: grey;'
            'margin-top: 6px; margin-bottom: 0px;'
        )
        self.x_label = QLabel('X: 0.000000 m')
        self.x_label.setStyleSheet('font-size: 28px; margin: 0px;')
        self.y_label = QLabel('Y: 0.000000 m')
        self.y_label.setStyleSheet('font-size: 28px; margin: 0px;')
        self.z_label = QLabel('Z: 0.000000 m')
        self.z_label.setStyleSheet('font-size: 28px; margin: 0px;')

        # ── Current orientation ────────────────────────────────────────
        self.orientation_header = QLabel('Current Orientation')
        self.orientation_header.setStyleSheet(
            'font-size: 12px; font-weight: bold; color: grey;'
            'margin-top: 6px; margin-bottom: 0px;'
        )
        self.roll_label  = QLabel('Roll:  0.000000 rad')
        self.roll_label.setStyleSheet('font-size: 28px; margin: 0px;')
        self.pitch_label = QLabel('Pitch: 0.000000 rad')
        self.pitch_label.setStyleSheet('font-size: 28px; margin: 0px;')
        self.yaw_label   = QLabel('Yaw:   0.000000 rad')
        self.yaw_label.setStyleSheet('font-size: 28px; margin: 0px;')

        # ── Goal position ──────────────────────────────────────────────
        self.goal_position_header = QLabel('Goal Position')
        self.goal_position_header.setStyleSheet(
            'font-size: 12px; font-weight: bold; color: grey;'
            'margin-top: 6px; margin-bottom: 0px;'
        )
        self.goal_x_label = QLabel('X: --')
        self.goal_x_label.setStyleSheet('font-size: 28px; margin: 0px;')
        self.goal_y_label = QLabel('Y: --')
        self.goal_y_label.setStyleSheet('font-size: 28px; margin: 0px;')
        self.goal_z_label = QLabel('Z: --')
        self.goal_z_label.setStyleSheet('font-size: 28px; margin: 0px;')

        # ── Goal orientation ───────────────────────────────────────────
        self.goal_orientation_header = QLabel('Goal Orientation')
        self.goal_orientation_header.setStyleSheet(
            'font-size: 12px; font-weight: bold; color: grey;'
            'margin-top: 6px; margin-bottom: 0px;'
        )
        self.goal_roll_label  = QLabel('Roll:  --')
        self.goal_roll_label.setStyleSheet('font-size: 28px; margin: 0px;')
        self.goal_pitch_label = QLabel('Pitch: --')
        self.goal_pitch_label.setStyleSheet('font-size: 28px; margin: 0px;')
        self.goal_yaw_label   = QLabel('Yaw:   --')
        self.goal_yaw_label.setStyleSheet('font-size: 28px; margin: 0px;')

        # ── Layout assembly ────────────────────────────────────────────
        layout.addWidget(self.status_label)
        layout.addWidget(self.start_button)
        layout.addWidget(self.estop_button)
        layout.addWidget(self.unlock_button)

        layout.addWidget(self.position_header)
        layout.addWidget(self.x_label)
        layout.addWidget(self.y_label)
        layout.addWidget(self.z_label)

        layout.addWidget(self.orientation_header)
        layout.addWidget(self.roll_label)
        layout.addWidget(self.pitch_label)
        layout.addWidget(self.yaw_label)

        layout.addWidget(self.goal_position_header)
        layout.addWidget(self.goal_x_label)
        layout.addWidget(self.goal_y_label)
        layout.addWidget(self.goal_z_label)

        layout.addWidget(self.goal_orientation_header)
        layout.addWidget(self.goal_roll_label)
        layout.addWidget(self.goal_pitch_label)
        layout.addWidget(self.goal_yaw_label)

        layout.addStretch()

        self.widget.setLayout(layout)
        context.add_widget(self.widget)

        # Spin timer
        self._spin_timer = QTimer()
        self._spin_timer.timeout.connect(self._spin_once)
        self._spin_timer.start(50)

        # Watchdog timer
        self._watchdog_timer = QTimer()
        self._watchdog_timer.timeout.connect(self._check_connection)
        self._watchdog_timer.start(500)

    # ------------------------------------------------------------------
    # ROS spin
    # ------------------------------------------------------------------

    def _spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    # ------------------------------------------------------------------
    # Connection watchdog
    # ------------------------------------------------------------------

    def _check_connection(self):
        if self._last_msg_time is None:
            connected = False
        else:
            connected = (
                time.time() - self._last_msg_time
            ) < self.CONNECTION_TIMEOUT

        if connected != self._is_connected:
            self._is_connected = connected
            self.bridge.connection_changed.emit(connected)

    def _update_connection_status(self, connected: bool):
        if connected:
            self.status_label.setText('● CONNECTED')
            self.status_label.setStyleSheet(
                'color: green; font-size: 14px; font-weight: bold;'
            )
        else:
            self.status_label.setText('● NO SIGNAL')
            self.status_label.setStyleSheet(
                'color: red; font-size: 14px; font-weight: bold;'
            )
        self._refresh_start_button()

    # ------------------------------------------------------------------
    # E-stop lock/unlock
    # ------------------------------------------------------------------

    def _apply_locked_state(self):
        self._estop_locked = True
        self.estop_button.setStyleSheet(
            'background-color: #8b0000; color: #555555; font-size: 20px;'
        )
        self.estop_button.setText('EMERGENCY STOP (LOCKED)')
        self.unlock_button.setEnabled(True)
        self.unlock_button.setStyleSheet(
            'background-color: #FFA500; color: black;'
            'font-size: 13px; font-weight: bold;'
        )

    def _apply_unlocked_state(self):
        self._estop_locked = False
        self.estop_button.setStyleSheet(
            'background-color: red; font-size: 20px;'
        )
        self.estop_button.setText('EMERGENCY STOP')
        self.unlock_button.setEnabled(False)
        self.unlock_button.setStyleSheet(
            'background-color: grey; color: white;'
            'font-size: 13px; font-weight: bold;'
        )

    def send_estop(self):
        if self._estop_locked:
            return

        click_time = time.time()

        estop_msg = Bool()
        estop_msg.data = True
        self.estop_pub.publish(estop_msg)

        ts_msg = Float64()
        ts_msg.data = click_time
        self.timestamp_pub.publish(ts_msg)

        self._apply_locked_state()

    def unlock_estop(self):
        if not self._estop_locked:
            return

        unlock_msg = Bool()
        unlock_msg.data = False
        self.estop_pub.publish(unlock_msg)

        self._apply_unlocked_state()

    # ------------------------------------------------------------------
    # Current pose callback
    # ------------------------------------------------------------------

    def pose_callback(self, msg):
        self._last_msg_time = time.time()

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        roll, pitch, yaw = self._quaternion_to_euler(qx, qy, qz, qw)

        self.bridge.pose_received.emit(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            roll,
            pitch,
            yaw
        )

    def _update_labels(self, x, y, z, roll, pitch, yaw):
        self.x_label.setText(f'X: {x:.6f} m')
        self.y_label.setText(f'Y: {y:.6f} m')
        self.z_label.setText(f'Z: {z:.6f} m')
        self.roll_label.setText(f'Roll:  {roll:.6f} rad')
        self.pitch_label.setText(f'Pitch: {pitch:.6f} rad')
        self.yaw_label.setText(f'Yaw:   {yaw:.6f} rad')

    # ------------------------------------------------------------------
    # Goal pose callback
    # ------------------------------------------------------------------

    def goal_callback(self, msg):
        if not msg.poses:
            return

        pose = msg.poses[0]
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        roll, pitch, yaw = self._quaternion_to_euler(qx, qy, qz, qw)

        self.bridge.goal_received.emit(
            pose.position.x,
            pose.position.y,
            pose.position.z,
            roll,
            pitch,
            yaw
        )

    def _update_goal_labels(self, x, y, z, roll, pitch, yaw):
        self.goal_x_label.setText(f'X: {x:.6f} m')
        self.goal_y_label.setText(f'Y: {y:.6f} m')
        self.goal_z_label.setText(f'Z: {z:.6f} m')
        self.goal_roll_label.setText(f'Roll:  {roll:.6f} rad')
        self.goal_pitch_label.setText(f'Pitch: {pitch:.6f} rad')
        self.goal_yaw_label.setText(f'Yaw:   {yaw:.6f} rad')

    # ------------------------------------------------------------------
    # Shared quaternion to euler conversion
    # ------------------------------------------------------------------

    def _quaternion_to_euler(self, qx, qy, qz, qw):
        roll  = math.atan2(
            2.0 * (qw * qx + qy * qz),
            1.0 - 2.0 * (qx * qx + qy * qy)
        )
        pitch = math.asin(
            max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
        )
        yaw   = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )
        return roll, pitch, yaw

    # ------------------------------------------------------------------
    # Start button
    # ------------------------------------------------------------------

    def send_start(self):
        msg = Bool()
        msg.data = True
        self.start_pub.publish(msg)
        self._robot_running = True
        self._refresh_start_button()

    def status_callback(self, msg: Bool):
        self._robot_running = msg.data
        if not msg.data:
            self._pick_place_ready = True  # node has fully started and is idle
        self.bridge.robot_status_changed.emit(msg.data)

    def _update_start_button(self, running: bool):
        self._refresh_start_button()

    def _refresh_start_button(self):
        if self._is_connected and self._pick_place_ready and not self._robot_running:
            self.start_button.setEnabled(True)
            self.start_button.setStyleSheet(
                'background-color: #007700; color: white; font-size: 16px; font-weight: bold;'
            )
        else:
            self.start_button.setEnabled(False)
            self.start_button.setStyleSheet(
                'background-color: #444444; color: #888888; font-size: 16px; font-weight: bold;'
            )

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def shutdown_plugin(self):
        self._spin_timer.stop()
        self._watchdog_timer.stop()
        self.node.destroy_node()
