import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from interactive_markers import InteractiveMarkerServer
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class FrameServer(Node):

    def __init__(self):
        super().__init__('frame_server')

        self.server = InteractiveMarkerServer(self, "frame_markers")
        self.tf_broadcaster = TransformBroadcaster(self)

        self.frames = {}
        self.transforms = {}

        self.create_subscription(
            String,
            "/create_frame",
            self.create_marker_callback,
            10
        )

        self.create_timer(0.1, self.broadcast_transforms)

    def create_marker_callback(self, msg):
        name = msg.data

        if name in self.frames:
            self.get_logger().warn(f"{name} already exists")
            return

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = name
        int_marker.description = name
        int_marker.scale = 1.0

        int_marker.pose.position.z = 1.0

        # Add a box marker for visualization
        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = 0.1
        box.scale.y = 0.1
        box.scale.z = 0.1
        box.color.r = 1.0
        box.color.g = 0.0
        box.color.b = 0.0
        box.color.a = 1.0

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(box)
        int_marker.controls.append(control)

        # Add interactive controls for translation and rotation
        # Translation along the X-axis
        control = InteractiveMarkerControl()
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        int_marker.controls.append(control)

        # Translation along the Y-axis
        control = InteractiveMarkerControl()
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        int_marker.controls.append(control)

        # Translation along the Z-axis
        control = InteractiveMarkerControl()
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        int_marker.controls.append(control)

        # Rotation around the X-axis
        control = InteractiveMarkerControl()
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        int_marker.controls.append(control)

        # Rotation around the Y-axis
        control = InteractiveMarkerControl()
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        int_marker.controls.append(control)

        # Rotation around the Z-axis
        control = InteractiveMarkerControl()
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        int_marker.controls.append(control)

        # Insert the marker into the server
        self.server.insert(int_marker)

        # Register the feedback callback
        self.server.setCallback(name, self.feedback_callback)

        self.server.applyChanges()

        self.frames[name] = True
        self.get_logger().info(f"Created frame {name}")

    def feedback_callback(self, feedback):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = feedback.marker_name

        # Convert Point to Vector3
        tf_msg.transform.translation.x = feedback.pose.position.x
        tf_msg.transform.translation.y = feedback.pose.position.y
        tf_msg.transform.translation.z = feedback.pose.position.z

        tf_msg.transform.rotation = feedback.pose.orientation

        self.transforms[feedback.marker_name] = tf_msg

    def broadcast_transforms(self):
        # Broadcast all stored transforms
        for name, tf_msg in self.transforms.items():
            tf_msg.header.stamp = self.get_clock().now().to_msg()  # Update timestamp
            self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = FrameServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()