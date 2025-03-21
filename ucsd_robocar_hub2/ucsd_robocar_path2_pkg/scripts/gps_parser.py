from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import pandas as pd
from mcap_ros2.reader import read_ros2_messages


rosbag_name_and_path = "/home/projects/ros2_ws/ebu2_courtyard/ebu2_courtyard_0.mcap"
output_csv_name_and_path = "/home/projects/ros2_ws/ebu2_courtyard/ebu2_courtyard_man_2.csv"
gps_topic_name = "/fix"

df_list = []
for msg in read_ros2_messages(rosbag_name_and_path):
    if msg.channel.topic == gps_topic_name:
        df_list.append({
            'lat': msg.ros_msg.latitude,
            'lon': msg.ros_msg.longitude,
            'alt': msg.ros_msg.altitude
        })

df = pd.concat([pd.DataFrame([d]) for d in df_list], ignore_index=True)
df.to_csv(output_csv_name_and_path, index = False, float_format='%.20f')

# read CSV file into a pandas dataframe
df = pd.read_csv(output_csv_name_and_path)

# user-defined percentage decrease
percent_decrease = 10 # reduce to 2%

# calculate the number of rows to skip based on the percentage decrease
skip_rows = int(100 / percent_decrease)

# create a new dataframe with the downsampled data
df_downsampled = df.iloc[::skip_rows]

# write the downsampled data to a new CSV file
df_downsampled.to_csv(output_csv_name_and_path, index=False, float_format='%.20f')