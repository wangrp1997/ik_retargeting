from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    venv_python = '/home/rw/ros2_ws/venv/bin/python3'
    glove_script = '/home/rw/ros2_ws/src/ik_retargeting/glove/glove/read_and_send_zmq.py'
    telekinesis_script = '/home/rw/ros2_ws/src/ik_retargeting/telekinesis/telekinesis/leap_ik.py'

    return LaunchDescription([
        ExecuteProcess(
            cmd=[venv_python, glove_script],
            name='read_and_send_zmq',
            output='screen',
            emulate_tty=True
        ),
        ExecuteProcess(
            cmd=[venv_python, telekinesis_script],
            name='leap_ik',
            output='screen',
            emulate_tty=True,
            additional_env={'isLeft': 'False'}  # 如果需要传参数可以这样写
        )
    ])