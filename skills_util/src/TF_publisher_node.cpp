#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <rosparam_utilities/rosparam_utilities.h>

#include <geometry_msgs/Pose.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_publisher");
    ros::NodeHandle nh("tf_publisher");

    XmlRpc::XmlRpcValue TF_param;
    ros::Rate rate(10.0);
    static tf::TransformBroadcaster br;
    tf::TransformListener listener;


    while(ros::ok())
    {
        if (!nh.getParam("/tf_params",TF_param))
        {
            ROS_ERROR("Unable to find /tf_params");
            return false;
        }
        else
        {
            if (TF_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
                ROS_ERROR("The param is not an array" );
                continue;
            }
            ROS_DEBUG("There are %d TF",TF_param.size());

            tf::Transform transform;
            std::string TF_name;
            std::string reference_TF;

            for (std::size_t i = 0; i < TF_param.size(); i++)
            {
                XmlRpc::XmlRpcValue single_TF = TF_param[i];
                if( single_TF.getType() != XmlRpc::XmlRpcValue::TypeStruct)
                {
                    ROS_WARN("The element is not a struct");
                    continue;
                }

                if( !single_TF.hasMember("name") )
                {
                    ROS_WARN("The element has not the field 'name'");
                    continue;
                }
                TF_name = rosparam_utilities::toString(single_TF["name"]);
                ROS_DEBUG_STREAM("TF_name: "<<TF_name);

                if( !single_TF.hasMember("frame") )
                {
                    ROS_WARN("The element %s has not the field 'frame'", TF_name.c_str());
                    continue;
                }
                reference_TF = rosparam_utilities::toString(single_TF["frame"]);
                ROS_DEBUG_STREAM("TF_reference_frame: "<<reference_TF);

                std::string what;
                std::vector<double> pos;
                if( !rosparam_utilities::getParam(single_TF,"position",pos,what) )
                {
                    ROS_WARN("TF %s has not the field 'position'",TF_name.c_str());
                    continue;
                }
                assert(pos.size()==3);
                ROS_DEBUG_STREAM("TF_position: ["<<pos.at(0)<<","<<pos.at(1)<<","<<pos.at(2)<<"]");

                std::vector<double> quat;
                if( !rosparam_utilities::getParam(single_TF,"quaternion",quat,what) )
                {
                    ROS_WARN("TF %s has not the field 'quaternion'",TF_name.c_str());
                    continue;
                }
                assert(quat.size()==4);
                ROS_DEBUG_STREAM("TF_quaternion: ["<<quat.at(0)<<","<<quat.at(1)<<","<<quat.at(2)<<","<<quat.at(3)<<"]");

                transform.setOrigin( tf::Vector3( pos.at(0),pos.at(1),pos.at(2) ) );
                transform.setRotation( tf::Quaternion( quat.at(0),quat.at(1),quat.at(2),quat.at(3) ) );
                br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), reference_TF, TF_name) );
                ros::spinOnce();
            }
        }

        std::vector<std::string> tf_names;
        listener.getFrameStrings(tf_names);

        if ( !tf_names.empty() )
        {
            for ( const std::string tf_name: tf_names)
            {
                std::size_t index = tf_name.find("_model__link");
                if ( index != std::string::npos )
                {
                    std::string new_tf_name = tf_name;
                    new_tf_name.erase(new_tf_name.begin()+index,new_tf_name.end());
                    std::string str = "_misure";
                    new_tf_name.append(str);
                    tf::Transform transform;
                    transform.setOrigin( tf::Vector3( (((rand() % 101)/50.0)-1) * 0.005,(((rand() % 101)/50.0)-1) * 0.005, (((rand() % 101)/50.0)-1) * 0.005 ) );
                    transform.setRotation( tf::Quaternion( 0.0,0.0,0.0,1.0 ) );
                    br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), tf_name, new_tf_name) );
                }
            }
        }
        rate.sleep();
    }

    return 0;
}
