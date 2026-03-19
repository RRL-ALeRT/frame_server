import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl
from interactive_markers import InteractiveMarkerServer
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class FrameServer(Node):

    def __init__(self):
        super().__init__('frame_server')

        self.server = InteractiveMarkerServer(self, "frame_markers")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.frames = {}
        self.transforms = {}

        self.pub_active_frames = self.create_publisher(String, '/active_frames', 10)

        self.create_subscription(String, "/create_frame", self.create_marker_callback, 10)
        self.create_subscription(String, "/delete_frame", self.delete_marker_callback, 10)
        self.create_subscription(String, "/rename_frame", self.rename_marker_callback, 10)

        self.create_timer(0.1, self.broadcast_transforms)

    def publish_active_frames(self):
        msg = String()
        msg.data = ",".join(self.frames.keys())
        self.pub_active_frames.publish(msg)

    def _create_interactive_marker(self, name, initial_pose_stamped=None):
        frame_id = "map"

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.name = name
        int_marker.description = name
        int_marker.scale = 1.0

        if initial_pose_stamped:
            int_marker.pose.position.x = initial_pose_stamped.transform.translation.x
            int_marker.pose.position.y = initial_pose_stamped.transform.translation.y
            int_marker.pose.position.z = initial_pose_stamped.transform.translation.z
            int_marker.pose.orientation = initial_pose_stamped.transform.rotation
        else:
            int_marker.pose.position.z = 1.0

        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = 0.1
        box.scale.y = 0.1
        box.scale.z = 0.1
        box.color.r = 1.0
        box.color.a = 1.0

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(box)
        int_marker.controls.append(control)

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

        tf_msg = TransformStamped()
        tf_msg.header.frame_id = frame_id
        tf_msg.child_frame_id = name
        tf_msg.transform.translation.x = int_marker.pose.position.x
        tf_msg.transform.translation.y = int_marker.pose.position.y
        tf_msg.transform.translation.z = int_marker.pose.position.z
        tf_msg.transform.rotation = int_marker.pose.orientation
        self.transforms[name] = tf_msg

    def create_marker_callback(self, msg):
        parts = msg.data.split('|')
        name = parts[0]
        reference_frame = parts[1] if len(parts) > 1 else None

        if name in self.frames:
            self.get_logger().warn(f"{name} already exists")
            return

        initial_pose = None
        if reference_frame:
            try:
                initial_pose = self.tf_buffer.lookup_transform("map", reference_frame, rclpy.time.Time())
            except Exception as e:
                self.get_logger().warn(f"Reference frame '{reference_frame}' not found: {e}")

        self._create_interactive_marker(name, initial_pose)
        self.publish_active_frames()

    def delete_marker_callback(self, msg):
        name = msg.data
        if name in self.frames:
            del self.frames[name]
            if name in self.transforms:
                del self.transforms[name]
            self.server.erase(name)
            self.server.applyChanges()
            self.publish_active_frames()

    def rename_marker_callback(self, msg):
        try:
            old_name, new_name = msg.data.split('|')
        except ValueError:
            return

        if old_name in self.frames and new_name not in self.frames:
            old_tf = self.transforms.get(old_name)
            self.delete_marker_callback(String(data=old_name))
            self._create_interactive_marker(new_name, old_tf)
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
        now = self.get_clock().now().to_msg()
        for tf_msg in self.transforms.values():
            tf_msg.header.stamp = now
            self.tf_broadcaster.sendTransform(tf_msg)

def main():
    rclpy.init()
    node = FrameServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
