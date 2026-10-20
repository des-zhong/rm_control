"""Passive experiment recording; never publishes or invokes robot services."""
import csv
from datetime import datetime
import json
from pathlib import Path
import re
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.serialization import serialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


COMMANDS = {
    0: 'stop_requested', 10: 'move_requested', 20: 'rm_strike_requested',
    30: 'catch_approach_requested', 32: 'gripper_close_requested',
    34: 'catch_accepted_transport_requested', 35: 'release_requested',
    36: 'retreat_requested', 37: 'return_orientation_requested',
    39: 'reset_complete_requested',
}


def selected(topic):
    """Include raw camera evidence and action feedback, including hidden topics."""
    if topic in ('/motions', '/to_rvo', '/rvo2', '/ctrl', '/yscomm', '/rosout'):
        return True
    return bool(re.fullmatch(
        r'/RM[1-6]/(cmd_vel|state_id|arm_position|gripper_state|'
        r'grip_vision/(caught|confidence|caught_score|debug_image)|'
        r'camera/image_color|(?:gripper|move_arm)/_action/(feedback|status))', topic))


def topic_metadata(name, type_name):
    """Support both Jazzy's required id and Galactic's older constructor."""
    fields = dict(name=name, type=type_name, serialization_format='cdr')
    try:
        return rosbag2_py.TopicMetadata(id=0, **fields)
    except TypeError:
        return rosbag2_py.TopicMetadata(**fields)


class DataRecorder(Node):
    def __init__(self):
        super().__init__('experiment_data_recorder')
        root = Path(str(self.declare_parameter('output_dir', 'run_data').value))
        root = root.expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        self.folder = root / datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
        self.folder.mkdir(exist_ok=False)
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(uri=str(self.folder / 'bag'), storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
        self.files = []
        self.events = self.csv_file('events.csv', [
            'receive_unix_ns', 'receive_monotonic_ns', 'receive_ros_ns',
            'topic', 'entity', 'event', 'previous', 'value'])
        self.trajectory = self.csv_file('trajectory.csv', [
            'receive_unix_ns', 'receive_monotonic_ns', 'receive_ros_ns',
            'index', 'entity', 'x', 'y', 'theta', 'vx_raw', 'vy_raw', 'omega_raw'])
        self.messages = (self.folder / 'messages.jsonl').open('w', encoding='utf-8')
        self.files.append(self.messages)
        self.subscriptions_by_topic = {}
        self.last = {}
        self.counts = {}
        self.metadata = {
            'started_at': datetime.now().astimezone().isoformat(),
            'folder': str(self.folder), 'topics': {},
            'notes': [
                'All CSV/JSONL timestamps are recorder receive times, not acquisition times.',
                'Image source headers and all serialized messages are preserved in bag.',
                'Events are observed command/status/value transitions, not physical success labels.',
                'Trajectory identity assumes index 0=ball, 1..6=RM, remaining=YS; verify mocap order.',
                'Velocity is recorded unchanged; existing mocap scaling may be incorrect.',
                'Best-effort volatile subscription avoids blocking publishers; delivery is not guaranteed.',
            ],
        }
        self.event('/recorder', 'recorder', 'recording_started', '', str(self.folder))
        self.discover()
        self.create_timer(1.0, self.discover)
        self.create_timer(1.0, self.flush)
        self.get_logger().info(f'Recording ready: {self.folder}')

    def csv_file(self, name, header):
        handle = (self.folder / name).open('w', newline='', encoding='utf-8')
        self.files.append(handle)
        writer = csv.writer(handle)
        writer.writerow(header)
        return writer

    def timestamps(self):
        return (time.time_ns(), time.monotonic_ns(), self.get_clock().now().nanoseconds)

    def event(self, topic, entity, event, previous, value, stamps=None):
        self.events.writerow([*(stamps or self.timestamps()), topic, entity,
                              event, previous, value])

    def transition(self, topic, entity, value, label, stamps):
        key = (topic, entity)
        if key not in self.last or self.last[key] != value:
            self.event(topic, entity, label, self.last.get(key, ''), value, stamps)
            self.last[key] = value

    def discover(self):
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        for topic, types in self.get_topic_names_and_types():
            if not selected(topic) or topic in self.subscriptions_by_topic:
                continue
            try:
                type_name = types[0]
                msg_type = get_message(type_name)
                self.writer.create_topic(topic_metadata(topic, type_name))
                self.subscriptions_by_topic[topic] = self.create_subscription(
                    msg_type, topic, lambda msg, t=topic: self.record(t, msg), qos)
                self.metadata['topics'][topic] = type_name
                self.event(topic, 'topic', 'subscription_started', '', type_name)
            except (ImportError, AttributeError, RuntimeError, ValueError) as exc:
                self.get_logger().warning(f'Cannot subscribe to {topic}: {exc}')
        self.save_metadata()

    def record(self, topic, msg):
        stamps = self.timestamps()
        # Bag timestamps use the same wall clock as CSV for offline correlation.
        self.writer.write(topic, serialize_message(msg), stamps[0])
        self.counts[topic] = self.counts.get(topic, 0) + 1
        if hasattr(msg, 'encoding') and hasattr(msg, 'data'):
            return  # Full image bytes and source headers are already in the bag.
        self.messages.write(json.dumps({
            'receive_unix_ns': stamps[0], 'receive_monotonic_ns': stamps[1],
            'receive_ros_ns': stamps[2], 'topic': topic,
            'message': message_to_ordereddict(msg),
        }, ensure_ascii=False) + '\n')
        if topic == '/motions':
            for i, pose in enumerate(msg.poses):
                entity = 'ball' if i == 0 else (f'RM{i}' if i <= 6 else f'YS{i-6}')
                vel = msg.twists[i] if i < len(msg.twists) else None
                self.trajectory.writerow([
                    *stamps, i, entity, pose.x, pose.y, pose.theta,
                    vel.x if vel else '', vel.y if vel else '', vel.theta if vel else ''])
        elif topic in ('/ctrl', '/to_rvo'):
            for i, code in enumerate(msg.code):
                if i:
                    self.transition(topic, f'RM{i}', code,
                                    COMMANDS.get(code, 'command_changed'), stamps)
        elif topic == '/yscomm':
            for i, comm in enumerate(msg.comm, 1):
                self.transition(topic, f'YS{i}', comm,
                                {0: 'ys_standby_requested', 1: 'ys_prepare_requested',
                                 2: 'ys_kick_requested'}.get(comm, 'ys_command_changed'), stamps)
        elif topic.endswith('/state_id'):
            self.transition(topic, topic.split('/')[1], msg.state,
                            'execution_status_changed', stamps)
        elif topic.endswith('/grip_vision/caught'):
            self.transition(topic, topic.split('/')[1], msg.data,
                            'visual_caught_changed', stamps)

    def save_metadata(self):
        self.metadata['received_message_counts'] = self.counts
        path = self.folder / 'metadata.json'
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2),
                             encoding='utf-8')
        temporary.replace(path)

    def flush(self):
        for handle in self.files:
            handle.flush()

    def close(self):
        self.event('/recorder', 'recorder', 'recording_stopped', '', '')
        self.metadata['stopped_at'] = datetime.now().astimezone().isoformat()
        self.save_metadata()
        for handle in self.files:
            handle.close()
        # Galactic SequentialWriter finalizes metadata when destroyed.
        del self.writer


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DataRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
