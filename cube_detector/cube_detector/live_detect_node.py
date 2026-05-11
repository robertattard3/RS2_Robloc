#!/usr/bin/env python3
"""
cube_detector ROS2 node
Detects red/blue cubes with a RealSense D435i and publishes:
  /cube_detections  (cube_detector_msgs/CubeDetectionArray)  — colour + XYZ + angle
  /cube_poses       (geometry_msgs/PoseArray)                 — XYZ + quaternion orientation
"""
import math

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from std_msgs.msg import Header
from cube_detector_msgs.msg import CubeDetection, CubeDetectionArray

# ── HSV sampler (press H to enable, then click anywhere on the image) ─────────
_hsv_sample_mode = False
_last_frame_hsv  = None   # updated each frame, read by mouse callback

def _mouse_callback(event, x, y, flags, param):
    if event != cv2.EVENT_LBUTTONDOWN or not _hsv_sample_mode:
        return
    if _last_frame_hsv is None:
        return
    s = 15
    fh, fw = _last_frame_hsv.shape[:2]
    x1, x2 = max(0, x - s), min(fw, x + s)
    y1, y2 = max(0, y - s), min(fh, y + s)
    roi = _last_frame_hsv[y1:y2, x1:x2]
    print(
        f"\n[HSV SAMPLE] pixel=({x},{y})\n"
        f"  H: mean={roi[:,:,0].mean():.1f}  "
        f"min={roi[:,:,0].min()}  max={roi[:,:,0].max()}\n"
        f"  S: mean={roi[:,:,1].mean():.1f}  "
        f"min={roi[:,:,1].min()}  max={roi[:,:,1].max()}\n"
        f"  V: mean={roi[:,:,2].mean():.1f}  "
        f"min={roi[:,:,2].min()}  max={roi[:,:,2].max()}\n"
        f"  Suggested range: "
        f"lower=[{max(0,int(roi[:,:,0].min())-5)}, "
        f"{max(0,int(roi[:,:,1].min())-20)}, "
        f"{max(0,int(roi[:,:,2].min())-20)}]  "
        f"upper=[{min(180,int(roi[:,:,0].max())+5)}, "
        f"255, 255]"
    )

# ── HSV colour ranges (tuned from live HSV sampler 07/05/2025) ───────────────


RED_LOWER_1 = np.array([0,  99, 228])   # H:18-45 covers H=30 orange-red cube
RED_UPPER_1 = np.array([15, 255, 255])   # S floor=35 excludes background (S=11)
RED_LOWER_2 = np.array([0,   0,   0])    # disabled — not needed
RED_UPPER_2 = np.array([0,   0,   0])    # disabled

# H:95-120 covers H=105 blue cube, S floor=80 excludes background
BLUE_LOWER = np.array([95,  80, 150])
BLUE_UPPER = np.array([120, 255, 255])
 
COLOUR_DISPLAY = {
    "Red":     (0,   0,   220),
    "Blue":    (220, 80,  0  ),
    "Unknown": (128, 128, 128)
}



# ── Detection helpers ─────────────────────

def get_colour_score(image, x, y, w, h, lower1, upper1,
                     lower2=None, upper2=None):
    margin_x = int(w * 0.20)
    margin_y = int(h * 0.20)
    roi = image[y + margin_y: y + h - margin_y,
                x + margin_x: x + w - margin_x]
    if roi.size == 0:
        return 0.0
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_roi, lower1, upper1)
    if lower2 is not None:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv_roi, lower2, upper2))
    total = roi.shape[0] * roi.shape[1]
    return cv2.countNonZero(mask) / total if total > 0 else 0.0


def classify_top_face(image, x, y, w, h):
    red_score  = get_colour_score(image, x, y, w, h,
                                  RED_LOWER_1, RED_UPPER_1,
                                  RED_LOWER_2, RED_UPPER_2)
    blue_score = get_colour_score(image, x, y, w, h,
                                  BLUE_LOWER, BLUE_UPPER)
    if red_score < 0.15 and blue_score < 0.15:
        return "Unknown", red_score, blue_score
    return ("Red" if red_score > blue_score else "Blue"), red_score, blue_score


def find_cube_candidates(image, lower1, upper1, lower2=None, upper2=None):
    global _last_frame_hsv
    blurred = cv2.GaussianBlur(image, (7, 7), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    _last_frame_hsv = hsv  # expose for mouse HSV sampler

    # Only apply lower range if not disabled (all zeros = disabled)
    if np.any(upper1 > 0):
        mask = cv2.inRange(hsv, lower1, upper1)
    else:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    if lower2 is not None:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower2, upper2))

    kernel_open  = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel_open,  iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=3)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        if area < 3000 or area > 60000:               continue
        if not (0.5 <= w / float(h) <= 2.0):          continue
        if w < 40 or h < 40:                           continue
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        solidity  = area / hull_area if hull_area > 0 else 0
        if solidity < 0.75:                            continue
        if min(w, h) / max(w, h) < 0.55:              continue
        candidates.append((x, y, w, h, contour, area))
    return candidates, mask


def remove_duplicates(cubes, overlap_threshold=0.3):
    if not cubes:
        return []
    indices = cv2.dnn.NMSBoxes(
        [(x, y, w, h) for (x, y, w, h, _, _) in cubes],
        [float(a) for (_, _, _, _, _, a) in cubes],
        score_threshold=0.0, nms_threshold=overlap_threshold,
    )
    return [cubes[i] for i in indices.flatten()] if len(indices) > 0 else []


def get_depth_at_centroid(depth_image, cx, cy):
    """Sample median depth over a 5×5 region for robustness."""
    x1 = max(0, cx - 2);  x2 = min(depth_image.shape[1] - 1, cx + 2)
    y1 = max(0, cy - 2);  y2 = min(depth_image.shape[0] - 1, cy + 2)
    region  = depth_image[y1:y2, x1:x2]
    samples = region[region > 0]
    return float(np.median(samples)) / 1000.0 if len(samples) > 0 else 0.0


def angle_to_quaternion(angle_deg: float) -> Quaternion:
    """Convert a minAreaRect angle to a Z-axis quaternion (camera frame).

    Cubes have 90-degree rotational symmetry, so we normalise into [0°, 90°)
    before converting to avoid discontinuous jumps in the published orientation.
    """
    yaw = math.radians(angle_deg % 90)
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw / 2),
        w=math.cos(yaw / 2),
    )


# ── Main processing pipeline ─────────────────────────────────────────────────

def process_frame(colour_frame, depth_frame, depth_intrin):
    """
    colour image → detect cubes → classify colour → get 3D coordinates.
    Returns (annotated_frame, mask_red, mask_blue, detections).
    Each detection is a dict with keys: colour, x, y, z, angle_deg.
    """
    frame       = np.asanyarray(colour_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    output      = frame.copy()
    detections  = []

    red_candidates,  mask_red  = find_cube_candidates(
        frame, RED_LOWER_1, RED_UPPER_1, RED_LOWER_2, RED_UPPER_2)
    blue_candidates, mask_blue = find_cube_candidates(
        frame, BLUE_LOWER, BLUE_UPPER)

    unique_cubes = remove_duplicates(red_candidates + blue_candidates)

    for (x, y, w, h, contour, area) in unique_cubes:

        colour, r_score, b_score = classify_top_face(frame, x, y, w, h)
        box_colour = COLOUR_DISPLAY[colour]

        hull      = cv2.convexHull(contour)
        rect      = cv2.minAreaRect(hull)
        angle     = rect[2]
        box       = np.intp(cv2.boxPoints(rect))
        cv2.drawContours(output, [box], 0, box_colour, 3)

        inner_rect = (rect[0], (rect[1][0] * 0.6, rect[1][1] * 0.6), angle)
        cv2.drawContours(output, [np.intp(cv2.boxPoints(inner_rect))],
                         0, (0, 255, 255), 1)

        M = cv2.moments(contour)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        cv2.circle(output, (cx, cy), 6, (0, 0, 255), -1)

        depth_m = get_depth_at_centroid(depth_image, cx, cy)
        if depth_m > 0:
            X, Y, Z = rs.rs2_deproject_pixel_to_point(
                depth_intrin, [cx, cy], depth_m
            )
            coord_str = f"X:{X:.3f} Y:{Y:.3f} Z:{Z:.3f}m"
        else:
            X, Y, Z   = 0.0, 0.0, 0.0
            coord_str = "Depth: N/A"

        detections.append({
            "colour":    colour,
            "x":         float(X),
            "y":         float(Y),
            "z":         float(Z),
            "angle_deg": float(angle),
        })

        top_y   = int(min(box[:, 1]))
        top_x   = int(box[np.argmin(box[:, 1])][0])
        label_y = top_y - 10 if top_y - 10 > 20 else top_y + 25

        cv2.putText(output, f"{colour} cube  {angle:.1f}deg",
                    (top_x, label_y),
                    cv2.FONT_HERSHEY_DUPLEX, 0.55, box_colour, 2)
        cv2.putText(output, coord_str,
                    (top_x, label_y + 20),
                    cv2.FONT_HERSHEY_DUPLEX, 0.45, box_colour, 1)
        cv2.putText(output, f"R:{r_score:.0%} B:{b_score:.0%}",
                    (top_x, label_y + 38),
                    cv2.FONT_HERSHEY_DUPLEX, 0.40, box_colour, 1)
        cv2.putText(output, f"({cx},{cy})", (cx + 8, cy),
                    cv2.FONT_HERSHEY_DUPLEX, 0.45, (0, 0, 255), 1)

        print(f"  {colour} cube | centroid:({cx},{cy}) | "
              f"angle:{angle:.1f}deg | {coord_str} | "
              f"R:{r_score:.0%} B:{b_score:.0%}")

    cv2.putText(output, f"Cubes: {len(unique_cubes)}", (10, 30),
                cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255, 255), 2)

    return output, mask_red, mask_blue, detections


# ── ROS2 node entry point ─────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = rclpy.create_node("cube_detector")
    logger = node.get_logger()

    # Publishers
    pub      = node.create_publisher(CubeDetectionArray, "/cube_detections", 10)
    pose_pub = node.create_publisher(PoseArray,           "/cube_poses",      10)

    # RealSense pipeline
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)

    profile      = pipeline.start(config)
    depth_intrin = (profile.get_stream(rs.stream.depth)
                           .as_video_stream_profile().get_intrinsics())
    align = rs.align(rs.stream.color)

    logger.info("cube_detector node started")
    logger.info("Publishing on /cube_detections and /cube_poses")
    logger.info("Controls: Q=quit | M=toggle masks | S=save | H=toggle HSV sampler")
    show_masks   = False
    global _hsv_sample_mode

    # Register mouse callback for HSV sampler
    cv2.namedWindow("Cube Detection")
    cv2.setMouseCallback("Cube Detection", _mouse_callback)

    try:
        while rclpy.ok():
            frames       = pipeline.wait_for_frames()
            aligned      = align.process(frames)
            colour_frame = aligned.get_color_frame()
            depth_frame  = aligned.get_depth_frame()
            if not colour_frame or not depth_frame:
                continue

            output, mask_red, mask_blue, detections = process_frame(
                colour_frame, depth_frame, depth_intrin)

            # ── Build ROS2 messages ──────────────────────────────────────────
            stamp  = node.get_clock().now().to_msg()
            header = Header(stamp=stamp,
                            frame_id="camera_color_optical_frame")

            det_msg  = CubeDetectionArray(header=header)
            pose_msg = PoseArray(header=header)

            for d in detections:
                det_msg.detections.append(CubeDetection(
                    colour=d["colour"],
                    x=d["x"], y=d["y"], z=d["z"],
                    angle_deg=d["angle_deg"],
                ))
                pose_msg.poses.append(Pose(
                    position=Point(x=d["x"], y=d["y"], z=d["z"]),
                    orientation=angle_to_quaternion(d["angle_deg"]),
                ))

            pub.publish(det_msg)
            pose_pub.publish(pose_msg)

            # Process any pending ROS2 callbacks without blocking the loop
            rclpy.spin_once(node, timeout_sec=0)

            # ── Display ──────────────────────────────────────────────────────
            if show_masks:
                r_bgr = cv2.cvtColor(mask_red,  cv2.COLOR_GRAY2BGR)
                b_bgr = cv2.cvtColor(mask_blue, cv2.COLOR_GRAY2BGR)
                r_bgr[mask_red  > 0] = (0,   0,   255)
                b_bgr[mask_blue > 0] = (255, 100,  0 )
                cv2.putText(r_bgr, "Red",  (10, 30),
                            cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2)
                cv2.putText(b_bgr, "Blue", (10, 30),
                            cv2.FONT_HERSHEY_DUPLEX, 1, (255, 100, 0), 2)
                cv2.imshow("Colour Masks", np.hstack([r_bgr, b_bgr]))
            else:
                try:
                    cv2.destroyWindow("Colour Masks")
                except Exception:
                    pass

            cv2.imshow("Cube Detection", output)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                logger.info("Quitting...")
                break
            elif key == ord('m'):
                show_masks = not show_masks
                logger.info(f"Mask view: {'ON' if show_masks else 'OFF'}")
            elif key == ord('s'):
                filename = f"capture_{cv2.getTickCount()}.jpg"
                cv2.imwrite(filename, output)
                logger.info(f"Saved: {filename}")
            elif key == ord('h'):
                _hsv_sample_mode = not _hsv_sample_mode
                logger.info(f"HSV sampler: {'ON — click on the image to sample colours' if _hsv_sample_mode else 'OFF'}")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

