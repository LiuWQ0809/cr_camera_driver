#!/bin/bash

# Monitor FPS for camera topics using a single ROS 2 node.
# Usage:
#   monitor_camera_topics_fps.sh [-i|--interval <seconds>] [topic1 topic2 ...]

set -euo pipefail

INTERVAL="1.0"
TOPICS=(
  "/cr/camera/bgr/front_right_960_768"
  "/cr/camera/bgr/front_left_960_768"
  "/cr/camera/bgr/left_960_768"
  "/cr/camera/bgr/right_960_768"
  "/cr/camera/bgr/rear_960_768"
)
# TOPICS=(
#   "/cam0/compressed"
#   "/cam1/compressed"
#   "/cam2/compressed"
#   "/cam3/compressed"
#   "/cam4/compressed"
# )

usage() {
  cat <<'EOF'
Usage: monitor_camera_topics_fps.sh [-i|--interval <seconds>] [topic1 topic2 ...]

Options:
  -i, --interval    Print interval in seconds (default: 1.0)
  -h, --help        Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -i|--interval)
      INTERVAL="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    *)
      break
      ;;
  esac
done

if [[ $# -gt 0 ]]; then
  TOPICS=("$@")
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "ERROR: python3 not found"
  exit 1
fi

python3 - "$INTERVAL" "${TOPICS[@]}" <<'PY'
import datetime
import os
import sys
import time

def main() -> int:
    if len(sys.argv) < 3:
        print("ERROR: interval and topics are required", file=sys.stderr)
        return 1

    try:
        interval = float(sys.argv[1])
    except ValueError:
        print("ERROR: invalid interval", file=sys.stderr)
        return 1

    if interval <= 0.0:
        interval = 1.0

    topics = sys.argv[2:]

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
    except Exception as exc:
        print("ERROR: failed to import ROS 2 Python packages:", exc, file=sys.stderr)
        print("Make sure the ROS 2 environment is sourced.", file=sys.stderr)
        return 1

    rclpy.init(args=None)

    class FPSNode(Node):
        def __init__(self, topic_list, print_interval):
            super().__init__(f"camera_topic_fps_monitor_{os.getpid()}")
            self._topics = topic_list
            self._interval = print_interval
            self._counts = {topic: 0 for topic in topic_list}
            self._totals = {topic: 0 for topic in topic_list}
            self._last_print = time.monotonic()

            for topic in topic_list:
                self.create_subscription(
                    Image,
                    topic,
                    self._make_cb(topic),
                    qos_profile_sensor_data,
                )

            self.create_timer(self._interval, self._print_stats)

        def _make_cb(self, topic):
            def _cb(_msg):
                self._counts[topic] += 1
                self._totals[topic] += 1
            return _cb

        def _print_stats(self):
            now = time.monotonic()
            dt = max(now - self._last_print, 1e-6)
            self._last_print = now

            ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            width = max(len(t) for t in self._topics)
            print(f"time: {ts}  interval: {dt:.2f}s")
            for topic in self._topics:
                count = self._counts[topic]
                self._counts[topic] = 0
                fps = count / dt
                total = self._totals[topic]
                print(f"{topic.ljust(width)}  fps={fps:6.2f}  total={total:8d}")
            print("")
            sys.stdout.flush()

    node = FPSNode(topics, interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
PY
