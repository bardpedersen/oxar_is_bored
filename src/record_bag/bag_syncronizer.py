#!/usr/bin.env python3
import pandas as pd
import rosbag
import rospy

def sync_bag(bag_path, clock_topic='/clock_sync'):

    # Read bag
    bag = rosbag.Bag(bag_path)
    messages = bag.read_messages(topics=[clock_topic])

    # Get time difference
    data_dict = {'index_time': [], 'clock_time': []}
    for topic, msg, t in messages:
        data_dict['index_time'].append(pd.Timestamp(t.to_sec(), unit='s'))
        data_dict['clock_time'].append(pd.Timestamp(msg.clock.secs + msg.clock.nsecs * 1e-9, unit='s'))

    # Calculate time difference
    df = pd.DataFrame(data_dict)
    time_difference = (df['index_time'] - df['clock_time']).dt.total_seconds().mean()
    print(f"Time difference: {time_difference} seconds")

    # Write new bag
    new_bag_path = bag_path.replace('.bag', '_sync.bag')
    with rosbag.Bag(new_bag_path, 'w') as new_bag:
        for topic, msg, timestamp in bag:
            modified_timestamp = timestamp - rospy.Duration.from_sec(time_difference)
            new_bag.write(topic, msg, modified_timestamp)
    print(f"New bag written to {new_bag_path}")


if __name__ == '__main__':
    bag_path = "/home/stein/bagfiles/clock_always_on/test_right.bag"
    sync_bag(bag_path)