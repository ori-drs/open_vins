# This message is published in publish_loopclosure_information() function

# Camera image timestamp
std_msgs/Header header

# Array of active features in camera image
ov_msckf/OVActiveFeature[] data

# Time offset base IMU to camera (t_imu = t_cam + t_off)
float64 t_offset_imu_cam
