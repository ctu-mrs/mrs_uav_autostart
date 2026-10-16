# no test node: automatic start refuses to start on its own, before it would talk to anyone
add_ros_isolated_launch_test(test.py TIMEOUT 120)
