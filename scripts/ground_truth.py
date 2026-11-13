#!/usr/bin/env python3
"""
Publish the ground truth of a generated raspberry field as RViz markers.

Reads <world_dir>/markers.json and publishes a latched visualization_msgs/MarkerArray
on /raspberry_field/ground_truth: one sphere per fruit (coloured by ripeness stage,
sized by its diameter), one cylinder per plant, small white spheres for flowers.

Parameters:
    world_dir (string, required)   directory produced by generate_field.py
    frame_id  (string, "world")    frame the markers are expressed in
"""
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

STAGE_COLORS = {
    'bud': (0.9, 0.95, 0.85),
    'flower': (1.0, 1.0, 1.0),
    'green': (0.45, 0.65, 0.25),
    'white': (0.85, 0.85, 0.65),
    'pink': (0.95, 0.5, 0.5),
    'ripe': (0.8, 0.05, 0.15),
    'overripe': (0.35, 0.02, 0.12),
}


class GroundTruth(Node):
    def __init__(self):
        super().__init__('raspberry_ground_truth')
        world_dir = self.declare_parameter('world_dir', '').value
        self.frame_id = self.declare_parameter('frame_id', 'world').value
        if not world_dir:
            raise SystemExit('parameter world_dir is required')
        markers = json.loads((Path(world_dir) / 'markers.json').read_text())
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(MarkerArray, '/raspberry_field/ground_truth', qos)
        self.msg = self.build(markers)
        self.publish()
        self.create_timer(2.0, self.publish)
        n_fruit = sum(1 for m in markers if m['marker_type'] == 'FRUIT')
        self.get_logger().info(f'loaded {len(markers)} markers ({n_fruit} fruits) from {world_dir}')

    def build(self, markers):
        arr = MarkerArray()
        for m in markers:
            mk = Marker()
            mk.header.frame_id = self.frame_id
            mk.id = int(m['id'])
            mk.action = Marker.ADD
            mk.pose.orientation.w = 1.0
            x, y, z = m['translation']
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = float(x), float(y), float(z)
            mk.color.a = 1.0
            if m['marker_type'] == 'PLANT':
                mk.ns, mk.type = 'plants', Marker.CYLINDER
                mk.scale.x = mk.scale.y = 0.15
                mk.scale.z = 0.05
                mk.color.r, mk.color.g, mk.color.b = 0.3, 0.1, 0.3
            else:
                mk.ns = 'fruits' if m['marker_type'] == 'FRUIT' else 'flowers'
                mk.type = Marker.SPHERE
                d = float(m.get('diameter', 0.012))
                mk.scale.x = mk.scale.y = mk.scale.z = d
                mk.color.r, mk.color.g, mk.color.b = STAGE_COLORS.get(m.get('stage'), (1.0, 1.0, 0.0))
            arr.markers.append(mk)
        return arr

    def publish(self):
        now = self.get_clock().now().to_msg()
        for mk in self.msg.markers:
            mk.header.stamp = now
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    node = GroundTruth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
