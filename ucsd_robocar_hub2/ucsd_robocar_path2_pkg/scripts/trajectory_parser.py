from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import pandas as pd

rosbag_name_and_path = "/home/projects/ros2_ws/ebu2_courtyard/ebu2_courtyard_0.mcap"
output_csv_name_and_path = "/home/projects/ros2_ws/ebu2_courtyard/ebu2_courtyard_man_2.csv"

# create reader instance and open for reading
with Reader(rosbag_name_and_path) as reader:
    msg = None
    # iterate over messages
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/trajectory':
            msg = deserialize_cdr(rawdata, connection.msgtype)
    df = pd.DataFrame(columns = ['px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw'])
    for pose_msg in msg.poses:
        df = df.append({'px': pose_msg.pose.position.x, 'py': pose_msg.pose.position.y, 'pz': pose_msg.pose.position.z, 'qx': pose_msg.pose.orientation.x, 'qy': pose_msg.pose.orientation.y, 'qz': pose_msg.pose.orientation.z, 'qw': pose_msg.pose.orientation.w}, ignore_index=True)
    df.to_csv(output_csv_name_and_path, index = False)
