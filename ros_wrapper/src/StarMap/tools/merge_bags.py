import rosbag
import sys
import rospy

if __name__ == '__main__':
    header_timestamps = False
    align_timestamps = True
    input_bags = sys.argv[1:-1]
    output_bag = sys.argv[-1]
    bag2first_timestamp = dict()
    renaming = {"/kitti/camera_gray_right/image_rect": "/kitti/camera_gray_right/image" }
    # http://docs.ros.org/en/latest/api/rostime/html/src_2time_8cpp_source.html
    TIME_MIN = rospy.Time(0, 1)
    with rosbag.Bag(output_bag, 'w') as outbag:
        for input_bag in input_bags:
            if align_timestamps:
                for topic, msg, t in rosbag.Bag(input_bag).read_messages():
                    dur = t - TIME_MIN
                    if input_bag not in bag2first_timestamp:
                        bag2first_timestamp[input_bag] = dur
                    elif dur < bag2first_timestamp[input_bag]:
                        bag2first_timestamp[input_bag] = dur
            for topic, msg, t in rosbag.Bag(input_bag).read_messages():
                if align_timestamps:
                    t  = t - bag2first_timestamp[input_bag]

                # This also replaces tf timestamps under the assumption 
                # that all transforms in the message share the same timestamp
                ren_topic  = renaming.get(topic, topic)
                if not header_timestamps:
                    outbag.write(ren_topic, msg, t)
                else:
                    if topic == "/tf" and msg.transforms:
                        outbag.write(ren_topic, msg, msg.transforms[0].header.stamp)
                    else:
                        outbag.write(ren_topic, msg, msg.header.stamp if msg._has_header else t)
