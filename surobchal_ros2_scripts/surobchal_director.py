import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty
from typing import List, Tuple
from collections import namedtuple

CameraArm = namedtuple('CameraManipulator', ('cp', 'js', 'servo_jp'))
InstrumentArm = namedtuple(
    'InstrumentArm',
    ('T_b_w', 'jaw_servo_jp', 'cp', 'cv', 'js', 'servo_cp', 'servo_jp', 'servo_jv'),
)

class SRCDirector(Node):
    def __init__(self):
        super().__init__('src_director')

        self._register_task3_setup_topics()
        self._register_manipulators()
        self._register_markers_subs()
        self._needle_pose_sub = self.create_subscription(
            PoseStamped, '/CRTK/Needle/measured_cp', self.update_pose_cb, 10
        )

        self.get_logger().info('Node initalized.')

    def _register_task3_setup_topics(self) -> None:
        self._task3_setup_ready = False
        self._task3_setup_init_pub = self.create_publisher(
            Empty, '/CRTK/scene/task_3_setup/init', 10
        )
        self._task3_setup_ready_sub = self.create_subscription(
            Empty, '/CRTK/scene/task_3_setup/ready', self.update_pose_cb, 10
        )

    def _register_manipulators(self) -> None:
        self._ecm_topics = self._prepare_camera_arm_topics()
        self._psm1_topics = self._prepare_instrument_arm_topics('psm1')
        self._psm2_topics = self._prepare_instrument_arm_topics('psm2')

    def _prepare_camera_arm_topics(self) -> CameraArm:
        cp = self.create_subscription(
            PoseStamped, '/CRTK/ecm/measured_cp', self.update_pose_cb, 10
        )
        js = self.create_subscription(
            JointState, '/CRTK/ecm/measured_js', self.update_pose_cb, 10
        )
        servo_jp = self.create_publisher(JointState, '/CRTK/ecm/servo_jp', 10)
        return CameraArm(cp, js, servo_jp)

    def _prepare_instrument_arm_topics(self, psm_name: str) -> InstrumentArm:
        T_b_w = self.create_subscription(
            PoseStamped, f'/CRTK/{psm_name}/T_b_w', self.update_pose_cb, 10
        )
        jaw_servo_jp = self.create_publisher(
            JointState, f'/CRTK/{psm_name}/jaw/servo_jp', 10
        )
        cp = self.create_subscription(
            PoseStamped, f'/CRTK/{psm_name}/measured_cp', self.update_pose_cb, 10
        )
        cv = self.create_subscription(
            TwistStamped, f'/CRTK/{psm_name}/measured_cv', self.update_pose_cb, 10
        )
        js = self.create_subscription(
            JointState, f'/CRTK/{psm_name}/measured_js', self.update_pose_cb, 10
        )
        servo_cp = self.create_publisher(PoseStamped, f'/CRTK/{psm_name}/servo_cp', 10)
        servo_jp = self.create_publisher(JointState, f'/CRTK/{psm_name}/servo_jp', 10)
        servo_jv = self.create_publisher(JointState, f'/CRTK/{psm_name}/servo_jv', 10)
        return InstrumentArm(
            T_b_w, jaw_servo_jp, cp, cv, js, servo_cp, servo_jp, servo_jv
        )

    def _register_markers_subs(self) -> None:
        self._entry_markers_poses_subs = self._create_subscribers_from_list(
            [
                ('/CRTK/Entry1/measured_cp', PoseStamped, self.update_pose_cb),
                ('/CRTK/Entry2/measured_cp', PoseStamped, self.update_pose_cb),
                ('/CRTK/Entry3/measured_cp', PoseStamped, self.update_pose_cb),
                ('/CRTK/Entry4/measured_cp', PoseStamped, self.update_pose_cb),
            ]
        )
        self._exit_markers_poses_subs = self._create_subscribers_from_list(
            [
                ('/CRTK/Exit1/measured_cp', PoseStamped, self.update_pose_cb),
                ('/CRTK/Exit2/measured_cp', PoseStamped, self.update_pose_cb),
                ('/CRTK/Exit3/measured_cp', PoseStamped, self.update_pose_cb),
                ('/CRTK/Exit4/measured_cp', PoseStamped, self.update_pose_cb),
            ]
        )

    def _create_subscribers_from_list(self, subscriber_data_list: List[Tuple]) -> Tuple:
        subs = [None] * len(subscriber_data_list)
        for idx, (topic, msg_type, callback) in enumerate(subscriber_data_list):
            subs[idx] = self.create_subscription(msg_type, topic, callback, 10)
        return tuple(subs)

    def update_pose_cb(self, msg: PoseStamped) -> None:
        pass

    def _task3_setup_ready_cb(self, msg: Empty) -> None:
        self._task3_setup_ready = True


def main(args=None):
    print('[surobchal_ros2] Dzien dobry from surgicial_robotics_challange_ros2_scripts!')
    print(f'[surobchal_ros2] Input arguments are:\n{args}')
    rclpy.init(args=args)

    src_director = SRCDirector()

    try:
        rclpy.spin(src_director)
    except KeyboardInterrupt:
        print('[surobchal_ros2] Interrupted')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
