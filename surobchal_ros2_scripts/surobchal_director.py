from threading import Thread
from time import sleep
from typing import List, Tuple, Callable, Any
from collections import namedtuple
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from ambf_msgs.msg import CameraState

from PyKDL import Rotation

import math

CameraArm = namedtuple(
    'CameraManipulator', ('cp', 'js', 'servo_jp', 'left_cp', 'right_cp')
)
InstrumentArm = namedtuple(
    'InstrumentArm',
    ('T_b_w', 'jaw_servo_jp', 'cp', 'cv', 'js', 'servo_cp', 'servo_jp', 'servo_jv'),
)


class SRCDirector(Node):
    def __init__(self):
        super().__init__('src_director')

        self._topics_data = {}
        self._cb_group = ReentrantCallbackGroup()
        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_static_broadcaster = StaticTransformBroadcaster(self)

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
        ecm_cp = self.create_subscription(
            PoseStamped,
            '/CRTK/ecm/measured_cp',
            self._create_pose_update_cb(
                'ecm_cp', source_frame='world', target_frame='camera_frame', static=True
            ),
            qos_profile=10,
            callback_group=self._cb_group,
        )
        camera_left_cp = self.create_subscription(
            CameraState,
            '/ambf/env/cameras/cameraL/State',
            self._create_camera_pose_update_cb(
                'camera_left_cp',
                source_frame='camera_frame',
                target_frame='cameraL',
                static=True,
            ),
            qos_profile=10,
            callback_group=self._cb_group,
        )
        camera_right_cp = self.create_subscription(
            CameraState,
            '/ambf/env/cameras/cameraR/State',
            self._create_camera_pose_update_cb(
                'camera_right_cp',
                source_frame='camera_frame',
                target_frame='cameraR',
                static=True,
            ),
            qos_profile=10,
            callback_group=self._cb_group,
        )
        ecm_js = self.create_subscription(
            JointState,
            '/CRTK/ecm/measured_js',
            self._create_data_update_cb('ecm_js'),
            qos_profile=10,
            callback_group=self._cb_group,
        )
        servo_jp = self.create_publisher(JointState, '/CRTK/ecm/servo_jp', 10)
        return CameraArm(ecm_cp, ecm_js, servo_jp, camera_left_cp, camera_right_cp)

    def _prepare_instrument_arm_topics(self, psm_name: str) -> InstrumentArm:
        T_b_w = self.create_subscription(
            PoseStamped,
            f'/CRTK/{psm_name}/T_b_w',
            self._create_pose_update_cb(
                f'{psm_name}_T_b_w', target_frame=f'{psm_name}/baselink', static=True
            ),
            qos_profile=10,
            callback_group=self._cb_group,
        )
        jaw_servo_jp = self.create_publisher(
            JointState, f'/CRTK/{psm_name}/jaw/servo_jp', 10
        )
        cp = self.create_subscription(
            PoseStamped,
            f'/CRTK/{psm_name}/measured_cp',
            # self._create_data_update_cb(f'{psm_name}_cp'),
            self._create_pose_update_cb(f'{psm_name}_cp'),
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
            entry_marker_key = f'marker_entry{idx+1}'
            exit_marker_key = f'marker_exit{idx+1}'

            self._topics_data[entry_marker_key] = PoseStamped()
            self._topics_data[exit_marker_key] = PoseStamped()
            entry_markers[idx] = (
                f'/CRTK/Entry{idx+1}/measured_cp',
                PoseStamped,
                # self._create_data_update_cb(entry_marker_key),
                self._create_pose_update_cb(
                    f'{entry_marker_key}_cp', target_frame=entry_marker_key, static=True
                ),
                self._cb_group,
            )
            exit_markers[idx] = (
                f'/CRTK/Exit{idx+1}/measured_cp',
                PoseStamped,
                # self._create_data_update_cb(exit_marker_key),
                self._create_pose_update_cb(
                    f'{exit_marker_key}_cp', target_frame=exit_marker_key, static=True
                ),
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
            # self._create_data_update_cb('needle_cp'),
            self._create_pose_update_cb('needle_cp', target_frame='needle'),
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

    def _create_pose_update_cb(
        self,
        topics_data_entry: str,
        source_frame: str = None,
        target_frame: str = None,
        static: bool = False,
    ) -> Callable[[Any], None]:
        self.get_logger().info(
            f'Registring pose callback & transform publish for {topics_data_entry} subscriber.'
        )
        tf_broadcaster = self._tf_static_broadcaster if static else self._tf_broadcaster
        trg_frame = topics_data_entry if target_frame is None else target_frame

        def update_data_and_publish_transform(msg: PoseStamped) -> None:
            self._topics_data.update({topics_data_entry: msg})

            src_frame = msg.header.frame_id if source_frame is None else source_frame
            t = TransformStamped()
            t.header = msg.header
            t.header.frame_id = src_frame
            t.child_frame_id = trg_frame
            t.transform.translation.x = msg.pose.position.x
            t.transform.translation.y = msg.pose.position.y
            t.transform.translation.z = msg.pose.position.z
            t.transform.rotation.x = msg.pose.orientation.x
            t.transform.rotation.y = msg.pose.orientation.y
            t.transform.rotation.z = msg.pose.orientation.z
            t.transform.rotation.w = msg.pose.orientation.w
            tf_broadcaster.sendTransform(t)

        return update_data_and_publish_transform

    def _create_camera_pose_update_cb(
        self,
        topics_data_entry: str,
        source_frame: str = None,
        target_frame: str = None,
        static: bool = False,
    ) -> Callable[[Any], None]:
        pose_update_and_transform_cb = self._create_pose_update_cb(
            topics_data_entry, source_frame, target_frame, static
        )

        def camera_pose_update_cb(msg: CameraState) -> None:
            pose_msg = self._parse_camera_state_to_pose_stamped(msg)
            pose_update_and_transform_cb(pose_msg)

        return camera_pose_update_cb

    def _parse_camera_state_to_pose_stamped(self, msg: CameraState) -> PoseStamped:
        result = PoseStamped()
        # TODO verify why this magic number makes more sense!
        simulation_unit_to_meters_coeff = 10
        result.header = msg.header
        result.pose.position.x = msg.pose.position.x / simulation_unit_to_meters_coeff
        result.pose.position.y = msg.pose.position.y / simulation_unit_to_meters_coeff
        result.pose.position.z = msg.pose.position.z / simulation_unit_to_meters_coeff
        return result

    def setup_task3(self) -> None:
        self.get_logger().info('Preparing setup for Task 3.')
        self._task3_setup_ready = False
        self._task3_setup_init_pub.publish(Empty())
        while self._task3_setup_ready == False:
            rclpy.spin_once(self)
        self.get_logger().info('Task 3 setup is prepared.')

    def set_camera_arm_joint_state(self, js: JointState) -> None:
        self._ecm_topics.servo_jp.publish(js)
        self.get_logger().debug('Published new joint state for camera arm (ESM).')

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
        self.get_logger().debug('Published new state for instrument arm (PSM1).')

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
        self.get_logger().debug('Published new state for instrument arm (PSM2).')

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
    sleep(2)  # let's ensure that dynamic ROS1 briging have time to kick in

    executor = MultiThreadedExecutor()
    executor.add_node(src_director)

    print('[surobchal_ros2] Creating new thread for spinning the node.')
    executor_thread = Thread(target=executor.spin_once(), daemon=True)
    executor_thread.start()
    print('[surobchal_ros2] Thread started. Proceeding to the infinity loop.')

    servo_cp_msg = PoseStamped()
    servo_cp_msg.pose.position.z = -1.0
    R_7_0 = Rotation.RPY(3.14, 0.0, 1.57079)

    servo_cp_msg.pose.orientation.x = R_7_0.GetQuaternion()[0]
    servo_cp_msg.pose.orientation.y = R_7_0.GetQuaternion()[1]
    servo_cp_msg.pose.orientation.z = R_7_0.GetQuaternion()[2]

    src_director.get_logger().info('Preapring Task3')
    src_director.setup_task3()
    src_director.get_logger().info('Task3 prepared')

    servo_jp_target = deepcopy(src_director['ecm_js'])
    # servo_jp_msg.position = [0., 0., 1.0, 0., 0., 0.]    

    done = False
    try:
        while not done:
            # TODO debug why this dosen't work
            # src_director.get_logger().info('Moving towards Entry Point')
            # target_point = copy(src_director['psm1_cp'])
            # entry_point = copy(src_director['marker_entry1_cp'])

            t, ns = src_director.get_clock().now().seconds_nanoseconds()
            servo_jp_msg = deepcopy(servo_jp_target)
            servo_jp_msg.position[1] += 0.05 * math.sin(t)
            # servo_jp_msg.position[1] += 0.02 * math.cos(t/100)
            src_director.set_camera_arm_joint_state(servo_jp_msg)

            # target_point.pose = copy(entry_point.pose)
            # target_point.pose.position.z -= 0.01  # meters

            # servo_jp_msg = JointState()
            # servo_jp_msg.position = [0., 0., 1.0, 0., 0., 0.]

            
            # target_point.pose.position.x = (
            #     entry_point.pose.position.x
            # )  # + 0.2 * math.sin(t)
            # target_point.pose.position.y = (
            #     entry_point.pose.position.y
            # )  # + 0.2 * math.cos(t)
            # src_director.set_psm2_arm_states(cp=target_point)
            # TODO: add wait for reaching desire state (with some boundries)
            # TODO Add wait with async
            # done = True
            rclpy.spin_once(src_director)

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
