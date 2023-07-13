#!/usr/bin/env python3

import rospy
import tf


if __name__ == '__main__':
    rospy.init_node('TF_publisher', anonymous=True)

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        if (rospy.has_param('/tf_params')):
            tf_params = rospy.get_param('/tf_params')
            existing_tfs = listener.getFrameStrings()
            for tf_param in tf_params:
                if tf_param['frame'] in existing_tfs:
                    br.sendTransform(tf_param['position'], tf_param['quaternion'], rospy.Time.now(), tf_param['name'], tf_param['frame'])
                elif tf_param['frame'] == 'world':
                    br.sendTransform(tf_param['position'], tf_param['quaternion'], rospy.Time.now(), tf_param['name'], tf_param['frame'])
        r.sleep()
