from threading import Thread
from time import sleep
from typing import List, Tuple, Callable, Any
from collections import namedtuple
from copy import copy

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty

CameraArm = namedtuple('CameraManipulator', ('cp', 'js', 'servo_jp'))
InstrumentArm = namedtuple(
    'InstrumentArm',
    ('T_b_w', 'jaw_servo_jp', 'cp', 'cv', 'js', 'servo_cp', 'servo_jp', 'servo_jv'),
)

class SRCDirector(Node):
    def __init__(self):
        super().__init__('src_director')

        self._topics_data = {}
        self._cb_group = ReentrantCallbackGroup()

        self._register_task3_setup_topics()
        self._register_manipulators()
        self._register_markers_subs(num_of_markers=4)
        self._register_needle_pose_sub()

        self.get_logger().info('Node initalized.')

    def _register_task3_setup_topics(self) -> None:
        self._task3_setup_ready = False
        self._task3_setup_init_pub = self.create_publisher(
            Empty, '/CRTK/scene/task_3_setup/init', 10
        )
        self._task3_setup_ready_sub = self.create_subscription(
            Empty,
            '/CRTK/scene/task_3_setup/ready',
            self._task3_setup_ready_cb,
            qos_profile=10,
            callback_group=self._cb_group,
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
            qos_profile=10,
            callback_group=self._cb_group,
        )
        js = self.create_subscription(
            JointState,
            '/CRTK/ecm/measured_js',
            self._create_data_update_cb('ecm_js'),
            qos_profile=10,
            callback_group=self._cb_group,
        )
        servo_jp = self.create_publisher(JointState, '/CRTK/ecm/servo_jp', 10)
        return CameraArm(cp, js, servo_jp)

    def _prepare_instrument_arm_topics(self, psm_name: str) -> InstrumentArm:
        T_b_w = self.create_subscription(
            PoseStamped,
            f'/CRTK/{psm_name}/T_b_w',
            self._create_data_update_cb(f'{psm_name}_T_b_w'),
            qos_profile=10,
            callback_group=self._cb_group,
        )
        jaw_servo_jp = self.create_publisher(
            JointState, f'/CRTK/{psm_name}/jaw/servo_jp', 10
        )
        cp = self.create_subscription(
            PoseStamped,
            f'/CRTK/{psm_name}/measured_cp',
            self._create_data_update_cb(f'{psm_name}_cp'),
            qos_profile=10,
            callback_group=self._cb_group,
        )
        cv = self.create_subscription(
            TwistStamped,
            f'/CRTK/{psm_name}/measured_cv',
            self._create_data_update_cb(f'{psm_name}_cv'),
            qos_profile=10,
            callback_group=self._cb_group,
        )
        js = self.create_subscription(
            JointState,
            f'/CRTK/{psm_name}/measured_js',
            self._create_data_update_cb(f'{psm_name}_js'),
            qos_profile=10,
            callback_group=self._cb_group,
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
                self._cb_group,
            )
            exit_markers[idx] = (
                f'/CRTK/Exit{idx+1}/measured_cp',
                PoseStamped,
                self._create_data_update_cb(exit_marker_key),
                self._cb_group,
            )

        self._entry_markers_poses_subs = self._create_subscribers_from_list(
            entry_markers
        )
        self._exit_markers_poses_subs = self._create_subscribers_from_list(exit_markers)

    def _create_subscribers_from_list(self, subscriber_data_list: List[Tuple]) -> Tuple:
        subs = [None] * len(subscriber_data_list)
        for idx, (topic, msg_type, callback, cb_group) in enumerate(
            subscriber_data_list
        ):
            subs[idx] = self.create_subscription(
                msg_type,
                topic,
                callback,
                qos_profile=10,
                callback_group=cb_group,
            )
        return tuple(subs)

    def _register_needle_pose_sub(self) -> None:
        self._topics_data['needle_cp'] = PoseStamped()
        self._needle_pose = PoseStamped()
        self._needle_pose_sub = self.create_subscription(
            PoseStamped,
            '/CRTK/Needle/measured_cp',
            self._create_data_update_cb('needle_cp'),
            qos_profile=10,
            callback_group=self._cb_group,
        )

    def _task3_setup_ready_cb(self, msg: Empty) -> None:
        self._task3_setup_ready = True

    def _create_data_update_cb(self, topics_data_entry: str) -> Callable[[Any], None]:
        self.get_logger().info(
            f'Registring callback for {topics_data_entry} subscriber.'
        )
        return lambda msg: self._topics_data.update({topics_data_entry: msg})

    def setup_task3(self) -> None:
        self.get_logger().info('Preparing setup for Task 3.')
        self._task3_setup_ready = False
        self._task3_setup_init_pub.publish(Empty())
        while self._task3_setup_ready == False:
            rclpy.spin_once(self)
        self.get_logger().info('Task 3 setup is prepared.')

    def set_camera_arm_joint_state(self, js: JointState) -> None:
        self._ecm_topics.servo_jp.publish(js)
        self.get_logger().info('Published new joint state for camera arm (ESM).')

    def set_psm1_arm_states(
        self,
        jaw_jp: JointState = None,
        cp: PoseStamped = None,
        js: JointState = None,
        jv: JointState = None,
    ) -> None:
        if jaw_jp is None:
            jaw_jp = self.get_topic_data('psm1_T_b_w')
        if cp is None:
            cp = self.get_topic_data('psm1_cp')
        if js is None:
            js = self.get_topic_data('psm1_js')
        if jv is None:
            jv = self.get_topic_data('psm1_cv')

        # self._psm1_topics.jaw_servo_jp.publish(jaw_jp)
        self._psm1_topics.servo_cp.publish(cp)
        # self._psm1_topics.servo_jp.publish(js)
        # self._psm1_topics.servo_jv.publish(jv)
        self.get_logger().info('Published new state for instrument arm (PSM1).')

    def set_psm2_arm_states(
        self,
        jaw_jp: JointState = None,
        cp: PoseStamped = None,
        js: JointState = None,
        jv: JointState = None,
    ) -> None:
        if jaw_jp is None:
            jaw_jp = self.get_topic_data('psm2_T_b_w')
        if cp is None:
            cp = self.get_topic_data('psm2_cp')
        if js is None:
            js = self.get_topic_data('psm2_js')
        if jv is None:
            jv = self.get_topic_data('psm2_cv')
    
        # self._psm2_topics.jaw_servo_jp.publish(jaw_jp)
        self._psm2_topics.servo_cp.publish(cp)
        # self._psm2_topics.servo_jp.publish(js)
        # self._psm2_topics.servo_jv.publish(jv)
        self.get_logger().info('Published new state for instrument arm (PSM2).')

    def get_topic_data(self, arg: str) -> Any:
        return self.__getitem__(arg)

    def __getitem__(self, arg: str) -> Any:
        return self._topics_data[arg]


def main(args=None):
    print(
        '[surobchal_ros2] Dzien dobry from surgicial_robotics_challange_ros2_scripts!'
    )
    print(f'[surobchal_ros2] Input arguments are:\n{args}')
    rclpy.init(args=args)

    src_director = SRCDirector()
    sleep(1)  # let's ensure that dynamic ROS1 briging have time to kick in

    executor = MultiThreadedExecutor()
    executor.add_node(src_director)

    print('[surobchal_ros2] Creating new thread for spinning the node.')
    executor_thread = Thread(target=executor.spin_once(), daemon=True)
    executor_thread.start()
    print('[surobchal_ros2] Thread started. Proceeding to the infinity loop.')

    try:
        while True:
            src_director.setup_task3()
            
            # TODO debug why this dosen't work 
            src_director.get_logger().info('Moving towards Entry Point')
            target_point = src_director['psm1_cp']
            entry_point = src_director['marker_entry1_cp']

            target_point.pose = copy(entry_point.pose)
            target_point.pose.position.z += 0.01 # meters
            src_director.set_psm1_arm_states(cp=target_point)
            src_director.get_logger().info('Waiting about 10 secs to do it again.')
            for _ in range(10000):
                sleep(0.001)
    except KeyboardInterrupt:
        print('[surobchal_ros2] Interrupted')

    src_director.destroy_node()
    try:
        rclpy.shutdown()
    except:
        pass
    executor_thread.join()


if __name__ == '__main__':
    main()
