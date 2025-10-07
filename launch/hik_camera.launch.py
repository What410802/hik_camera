import os
import ament_index_python
import launch, launch_ros
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

	return launch.LaunchDescription([
		# Startup params, supporting cmdline input
		launch.actions.DeclareLaunchArgument(
			'camera_serial',
			default_value='',
			description='If empty then choose 1st camera'
		),
		
		# Camera Node
		launch_ros.actions.Node(
			package='hik_camera',
			executable='hik_camera_node',
			name='hik_camera_node',
			output='screen', # Logging to cmdline
			parameters=[
				{'camera_serial': LaunchConfiguration('camera_serial')},
				{'image_topic': '/image_raw'},
				{"capture_timeout_msec": 1000},
				
				{'exposure_time': 1000.},
				{'gain': 1.0},
				{'frame_rate': 20},
				{'pixel_format': 'rgb8'},
			]
		)
	])

## PixelFormat	IEnumeration
# 0x01080001:
# 0x01100003:Mono10
# 0x010C0004:Mono10Packed
# 0x01100005:Mono12
# 0x010C0006:Mono12Packed
# 0x01100007:Mono16
# 0x02180014:RGB8Packed
# 0x02100032:YUV422_8
# 0x0210001F:YUV422_8_UYVY
# 0x01080008:BayerGR8
# 0x01080009:BayerRG8
# 0x0108000A:BayerGB8
# 0x0108000B:BayerBG8
# 0x0110000e:BayerGB10
# 0x01100012:BayerGB12
# 0x010C002C:BayerGB12Packed
