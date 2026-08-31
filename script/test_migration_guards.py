from pathlib import Path


def test_installed_scripts_have_no_ros1_runtime_calls():
    directory = Path(__file__).parent
    forbidden = (
        'import rospy',
        'roscore',
        "'rosnode'",
        "'roslaunch'",
        "'rosservice'",
        "'rostopic'",
        "'rosrun'",
        "'rospack'",
        'rosbag.Bag',
    )
    for path in directory.glob('*.py'):
        if path.name.startswith('test_'):
            continue
        text = path.read_text(encoding='utf-8')
        for token in forbidden:
            assert token not in text, f'{path.name} still contains {token}'


def test_main_gui_uses_ros2_service_fields_and_launch_files():
    text = (Path(__file__).parent / 'aNAV_ranger.py').read_text()
    assert 'x2bot_teleop/srv/SetInt' in text
    assert 'current_id:' in text
    assert 'currentID:' not in text
    assert '3startlocation.launch.py' in text
    assert "['rviz2', '-d', config]" in text
