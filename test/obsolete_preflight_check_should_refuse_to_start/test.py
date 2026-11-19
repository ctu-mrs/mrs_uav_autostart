import unittest
import os

import launch
import launch_ros
import launch_testing.actions
import launch_testing.asserts
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_test_description():

    ld = launch.LaunchDescription()

    launch_file_path = os.path.abspath(__file__)
    launch_dir = os.path.dirname(launch_file_path)

    current_rmw = os.environ.get('RMW_IMPLEMENTATION', '')

    if current_rmw == 'rmw_zenoh_cpp':
        ld.add_action(
            launch_ros.actions.Node(
                package='rmw_zenoh_cpp',
                namespace='',
                executable='rmw_zenohd',
                name='zenoh_router',
                output='screen'
            )
        )

    # only automatic start: it must refuse to start before it would talk to anyone
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('mrs_uav_autostart'),
                    'launch',
                    'automatic_start.launch.py'
                    ])
                ]),
                launch_arguments={
                    'uav_name': 'uav1',
                    'use_sim_time': 'false',
                    'custom_config': launch_dir+"/config/custom_config.yaml",
                }.items()
            )
        )

    # starts the python test part down below, right away: the node shuts down on its own
    ld.add_action(launch_testing.actions.ReadyToTest())

    return ld

# #{ class ObsoleteParameterTest(unittest.TestCase)

class ObsoleteParameterTest(unittest.TestCase):

    def test_refuses_to_start(self, proc_output, proc_info, timeout=60):

        """Check that automatic start reports the obsolete parameter and shuts down"""

        proc_output.assertWaitFor("obsolete parameter 'mrs_uav_autostart/preflight_check' found in custom_config", timeout=timeout)

        proc_info.assertWaitForShutdown(process='component_container', timeout=timeout)

        # it never got to the end of its initialization
        self.assertFalse(proc_output.waitFor("AUTOMATIC START INITIALIZED", timeout=1))

# #} end of

# #{ Post-shutdown tests

@launch_testing.post_shutdown_test()
class ObsoleteParameterTestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        """Check that automatic start exited with an error."""
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[1], process='component_container')

# #} end of Post-shutdown tests
