import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty
from typing import List, Tuple, Callable, Any
from collections import namedtuple

CameraArm = namedtuple('CameraManipulator', ('cp', 'js', 'servo_jp'))
InstrumentArm = namedtuple(
    'InstrumentArm',
    ('T_b_w', 'jaw_servo_jp', 'cp', 'cv', 'js', 'servo_cp', 'servo_jp', 'servo_jv'),
)

# TODO analyze case of ROS1 subscribers to ROS2 publishers - does it work without hacks?
# TODO write methods for publishing servos states


class SRCDirector(Node):
    def __init__(self):
        super().__init__('src_director')

        self._topics_data = {}

        self._register_task3_setup_topics()
        self._register_manipulators()
        self._register_markers_subs(num_of_markers=4)

        self._topics_data['needle_cp'] = PoseStamped()
        self._needle_pose = PoseStamped()
        self._needle_pose_sub = self.create_subscription(
            PoseStamped,
            '/CRTK/Needle/measured_cp',
            self._create_data_update_cb('needle_cp'),
            10,
        )

        self.get_logger().info('Node initalized.')

    def _register_task3_setup_topics(self) -> None:
        self._task3_setup_ready = False
        self._task3_setup_init_pub = self.create_publisher(
            Empty, '/CRTK/scene/task_3_setup/init', 10
        )
        self._task3_setup_ready_sub = self.create_subscription(
            Empty, '/CRTK/scene/task_3_setup/ready', self._task3_setup_ready_cb, 10
        )

    def _register_manipulators(self) -> None:
        self._allocate_manipulators_data_memory()
        self._ecm_topics = self._prepare_camera_arm_topics()
        self._psm1_topics = self._prepare_instrument_arm_topics('psm1')
        self._psm2_topics = self._prepare_instrument_arm_topics('psm2')

    def _allocate_manipulators_data_memory(self) -> None:
        self._topics_data['ecm_cp'] = PoseStamped()
        self._topics_data['ecm_js'] = JointState()

        for name in ('psm1', 'psm2'):
            self._topics_data[f'{name}_T_b_w'] = PoseStamped()
            self._topics_data[f'{name}_cp'] = PoseStamped()
            self._topics_data[f'{name}_cv'] = TwistStamped()
            self._topics_data[f'{name}_js'] = JointState()

    def _prepare_camera_arm_topics(self) -> CameraArm:
        cp = self.create_subscription(
            PoseStamped,
            '/CRTK/ecm/measured_cp',
            self._create_data_update_cb('ecm_cp'),
            10,
        )
        js = self.create_subscription(
            JointState,
            '/CRTK/ecm/measured_js',
            self._create_data_update_cb('ecm_js'),
            10,
        )
        servo_jp = self.create_publisher(JointState, '/CRTK/ecm/servo_jp', 10)
        return CameraArm(cp, js, servo_jp)

    def _prepare_instrument_arm_topics(self, psm_name: str) -> InstrumentArm:
        T_b_w = self.create_subscription(
            PoseStamped,
            f'/CRTK/{psm_name}/T_b_w',
            self._create_data_update_cb(f'{psm_name}_T_b_w'),
            10,
        )
        jaw_servo_jp = self.create_publisher(
            JointState, f'/CRTK/{psm_name}/jaw/servo_jp', 10
        )
        cp = self.create_subscription(
            PoseStamped,
            f'/CRTK/{psm_name}/measured_cp',
            self._create_data_update_cb(f'{psm_name}_cp'),
            10,
        )
        cv = self.create_subscription(
            TwistStamped,
            f'/CRTK/{psm_name}/measured_cv',
            self._create_data_update_cb(f'{psm_name}_cv'),
            10,
        )
        js = self.create_subscription(
            JointState,
            f'/CRTK/{psm_name}/measured_js',
            self._create_data_update_cb(f'{psm_name}_js'),
            10,
        )
        servo_cp = self.create_publisher(PoseStamped, f'/CRTK/{psm_name}/servo_cp', 10)
        servo_jp = self.create_publisher(JointState, f'/CRTK/{psm_name}/servo_jp', 10)
        servo_jv = self.create_publisher(JointState, f'/CRTK/{psm_name}/servo_jv', 10)
        return InstrumentArm(
            T_b_w, jaw_servo_jp, cp, cv, js, servo_cp, servo_jp, servo_jv
        )

    def _register_markers_subs(self, num_of_markers: int) -> None:
        entry_markers = [None] * num_of_markers
        exit_markers = [None] * num_of_markers

        for idx in range(num_of_markers):
            entry_marker_key = f'marker_entry{idx+1}_cp'
            exit_marker_key = f'marker_exit{idx+1}_cp'

            self._topics_data[entry_marker_key] = PoseStamped()
            self._topics_data[exit_marker_key] = PoseStamped()
            entry_markers[idx] = (
                f'/CRTK/Entry{idx+1}/measured_cp',
                PoseStamped,
                self._create_data_update_cb(entry_marker_key),
            )
            exit_markers[idx] = (
                f'/CRTK/Exit{idx+1}/measured_cp',
                PoseStamped,
                self._create_data_update_cb(exit_marker_key),
            )

        self._entry_markers_poses_subs = self._create_subscribers_from_list(
            entry_markers
        )
        self._exit_markers_poses_subs = self._create_subscribers_from_list(exit_markers)

    def _create_subscribers_from_list(self, subscriber_data_list: List[Tuple]) -> Tuple:
        subs = [None] * len(subscriber_data_list)
        for idx, (topic, msg_type, callback) in enumerate(subscriber_data_list):
            subs[idx] = self.create_subscription(msg_type, topic, callback, 10)
        return tuple(subs)

    def _task3_setup_ready_cb(self, msg: Empty) -> None:
        self._task3_setup_ready = True

    def _create_data_update_cb(self, topics_data_entry: str) -> Callable[[Any], None]:
        self.get_logger().info(
            f'Registring callback for {topics_data_entry} subscriber.'
        )
        return lambda msg: self._topics_data.update({topics_data_entry: msg})

    def setup_task3(self) -> None:
        self.get_logger().info('Preparing setup for Task 3.')
        self._task3_setup_init_pub.publish(Empty())
        while not self._task3_setup_ready:
            sleep(0.1)
        self.get_logger().info('Task 3 setup is prepared.')

    def get_topic_data(self, arg: str) -> Any:
        return self.__getitem__[arg]

    def __getitem__(self, arg: str) -> Any:
        return self._topics_data[arg]


def main(args=None):
    print(
        '[surobchal_ros2] Dzien dobry from surgicial_robotics_challange_ros2_scripts!'
    )
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
