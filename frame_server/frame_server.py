import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from interactive_markers import InteractiveMarkerServer
from tf2_ros import TransformBroadcaster
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class FrameServer(Node):

    def __init__(self):
        super().__init__('frame_server')

        self.server = InteractiveMarkerServer(self, "frame_markers")
        self.tf_broadcaster = TransformBroadcaster(self)

        # TF listener to look up external frames
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.frames = {}
        self.transforms = {}

        # Publishers / Subscribers
        self.pub_active_frames = self.create_publisher(String, '/active_frames', 10)

        self.create_subscription(String, "/create_frame", self.create_marker_callback, 10)
        self.create_subscription(String, "/delete_frame", self.delete_marker_callback, 10)
        self.create_subscription(String, "/rename_frame", self.rename_marker_callback, 10)

        self.create_timer(0.1, self.broadcast_transforms)

    def publish_active_frames(self):
        msg = String()
        # Publish a comma-separated list of active frame names
        msg.data = ",".join(self.frames.keys())
        self.pub_active_frames.publish(msg)

    def _create_interactive_marker(self, name, initial_tf=None):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = name
        int_marker.description = name
        int_marker.scale = 1.0

        if initial_tf:
            # Reapply previous transform if recreating (e.g., during rename)
            int_marker.pose.position.x = initial_tf.transform.translation.x
            int_marker.pose.position.y = initial_tf.transform.translation.y
            int_marker.pose.position.z = initial_tf.transform.translation.z
            int_marker.pose.orientation = initial_tf.transform.rotation
        else:
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

        # Append standard translation/rotation controls...
        axes = [
            ("move_x", InteractiveMarkerControl.MOVE_AXIS, [1.0, 1.0, 0.0, 0.0]),
            ("move_y", InteractiveMarkerControl.MOVE_AXIS, [1.0, 0.0, 1.0, 0.0]),
            ("move_z", InteractiveMarkerControl.MOVE_AXIS, [1.0, 0.0, 0.0, 1.0]),
            ("rotate_x", InteractiveMarkerControl.ROTATE_AXIS, [1.0, 1.0, 0.0, 0.0]),
            ("rotate_y", InteractiveMarkerControl.ROTATE_AXIS, [1.0, 0.0, 1.0, 0.0]),
            ("rotate_z", InteractiveMarkerControl.ROTATE_AXIS, [1.0, 0.0, 0.0, 1.0])
        ]

        for ctrl_name, mode, orient in axes:
            ctrl = InteractiveMarkerControl()
            ctrl.name = ctrl_name
            ctrl.interaction_mode = mode
            ctrl.orientation.w, ctrl.orientation.x, ctrl.orientation.y, ctrl.orientation.z = orient
            int_marker.controls.append(ctrl)

        self.server.insert(int_marker)
        self.server.setCallback(name, self.feedback_callback)
        self.server.applyChanges()

        self.frames[name] = True
        
        if initial_tf:
            initial_tf.child_frame_id = name
            self.transforms[name] = initial_tf

    def create_marker_callback(self, msg):
        # Parse "name" or "name|reference_frame"
        parts = msg.data.split('|')
        name = parts[0]
        reference_frame = parts[1] if len(parts) > 1 else None

        if name in self.frames:
            self.get_logger().warn(f"{name} already exists")
            return

        # If a reference frame is given, copy its orientation
        initial_tf = None
        if reference_frame:
            # First check our own transforms, then look up from TF tree
            if reference_frame in self.transforms:
                ref_tf = self.transforms[reference_frame]
                initial_tf = TransformStamped()
                initial_tf.header.frame_id = "map"
                initial_tf.child_frame_id = name
                initial_tf.transform.translation.x = 0.0
                initial_tf.transform.translation.y = 0.0
                initial_tf.transform.translation.z = 1.0
                initial_tf.transform.rotation = ref_tf.transform.rotation
                self.get_logger().info(
                    f"Created frame '{name}' with orientation from internal '{reference_frame}'"
                )
            else:
                # Try to look up the frame from the TF tree
                try:
                    tf_stamped = self.tf_buffer.lookup_transform(
                        "map", reference_frame, rclpy.time.Time())
                    initial_tf = TransformStamped()
                    initial_tf.header.frame_id = "map"
                    initial_tf.child_frame_id = name
                    # Only copy orientation, set default position
                    initial_tf.transform.translation.x = 0.0
                    initial_tf.transform.translation.y = 0.0
                    initial_tf.transform.translation.z = 1.0
                    initial_tf.transform.rotation = tf_stamped.transform.rotation
                    self.get_logger().info(
                        f"Created frame '{name}' with orientation from TF '{reference_frame}'"
                    )
                except Exception as e:
                    self.get_logger().warn(
                        f"Reference frame '{reference_frame}' not found in TF tree: {e}. "
                        f"Creating with default orientation."
                    )

        self._create_interactive_marker(name, initial_tf)
        self.get_logger().info(f"Created frame {name}")
        self.publish_active_frames()

    def delete_marker_callback(self, msg):
        name = msg.data
        if name in self.frames:
            del self.frames[name]
            if name in self.transforms:
                del self.transforms[name]
            self.server.erase(name)
            self.server.applyChanges()
            self.get_logger().info(f"Deleted frame {name}")
            self.publish_active_frames()

    def rename_marker_callback(self, msg):
        try:
            old_name, new_name = msg.data.split('|')
        except ValueError:
            self.get_logger().error("Rename message format invalid. Expected 'old|new'")
            return

        if old_name in self.frames and new_name not in self.frames:
            # Capture old state
            old_tf = self.transforms.get(old_name)
            
            # Clean up old marker
            del self.frames[old_name]
            if old_name in self.transforms:
                del self.transforms[old_name]
            self.server.erase(old_name)

            # Create new marker with old pose
            self._create_interactive_marker(new_name, old_tf)
            self.get_logger().info(f"Renamed frame {old_name} to {new_name}")
            self.publish_active_frames()

    def feedback_callback(self, feedback):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = feedback.marker_name

        tf_msg.transform.translation.x = feedback.pose.position.x
        tf_msg.transform.translation.y = feedback.pose.position.y
        tf_msg.transform.translation.z = feedback.pose.position.z
        tf_msg.transform.rotation = feedback.pose.orientation

        self.transforms[feedback.marker_name] = tf_msg

    def broadcast_transforms(self):
        for name, tf_msg in self.transforms.items():
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(tf_msg)

def main():
    rclpy.init()
    node = FrameServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()