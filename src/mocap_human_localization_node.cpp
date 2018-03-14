//
// Created by gbuisan on 13/03/18.
//

#include <ros/ros.h>
#include "../include/optitrack/or_pose_estimator_state.h"
#include <tf/tf.h>
#include <std_msgs/Time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <hanp_msgs/TrackedHumans.h>
#include <hanp_msgs/TrackedSegmentType.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define HUMAN_PUB_TOPIC "/tracked_humans"
#define HUMAN_MARKER_PUB_TOPIC "/tracked_humans_markers"

using namespace std;

class MocapHumanLocalization{
private:
    ros::NodeHandle node_;
    tf::TransformListener tfListener_;
    tf::TransformBroadcaster tfBroadcaster_;
    ros::Publisher hanpPublisher_;
    ros::Publisher hanpMarkerPublisher_;
    ros::Subscriber personSub_;
    ros::Timer locaTimerTf_;
    ros::Timer hanpPublishTimer_;

    tf::Transform tfOptitrack2Human_;

    geometry_msgs::PoseStamped currentPose, lastPose;
    geometry_msgs::TwistStamped currentSpeed, lastSpeed;
    bool isCurrentPose, isLastPose, isCurrentSpeed, isLastSpeed;

    public:
    MocapHumanLocalization(ros::NodeHandle node): tfListener_(), tfBroadcaster_(), node_(node), isLastPose(false), isLastSpeed(
            false){
        personSub_ = node_.subscribe("/optitrack/bodies/person_01_torso", 10, &MocapHumanLocalization::updateMocapPersonPose, this);
        locaTimerTf_ = node_.createTimer(ros::Duration(0.2), &MocapHumanLocalization::updateTf,this);
        tfOptitrack2Human_.setIdentity();

        hanpPublisher_ = node_.advertise<hanp_msgs::TrackedHumans>(HUMAN_PUB_TOPIC, 10);
        hanpMarkerPublisher_ = node_.advertise<visualization_msgs::MarkerArray>(HUMAN_MARKER_PUB_TOPIC, 10);
        hanpPublishTimer_ = node_.createTimer(ros::Duration(0.2), &MocapHumanLocalization::updateHanp, this);
    }

    private:
    void updateTf(const ros::TimerEvent&){
        try {
            tfBroadcaster_.sendTransform(
                    tf::StampedTransform(tfOptitrack2Human_, ros::Time::now(), "optitrack", "human1"));
        }catch (tf::TransformException &ex){
            ROS_ERROR("[mocap_localization] Error occur : %s\n", ex.what());
        }
    };

    void updateHanp(const ros::TimerEvent&){
        if (isLastPose){
            fillAndSendHanp();
        }
    }

    void fillHanpSpeed(hanp_msgs::TrackedSegment* segment){
        double dt = (currentPose.header.stamp - lastPose.header.stamp).toSec();
        tf::Vector3 positionDiff(currentPose.pose.position.x - lastPose.pose.position.x,
                                currentPose.pose.position.y - lastPose.pose.position.y,
                                currentPose.pose.position.z - lastPose.pose.position.z);
        double roll_diff, pitch_diff, yaw_diff;
        tf::Matrix3x3(tf::Quaternion(currentPose.pose.orientation.x, currentPose.pose.orientation.y,
                                     currentPose.pose.orientation.z, currentPose.pose.orientation.w)
                        * tf::Quaternion(lastPose.pose.orientation.x, lastPose.pose.orientation.y,
                                         lastPose.pose.orientation.z, lastPose.pose.orientation.w).inverse()).getRPY(
                roll_diff, pitch_diff, yaw_diff);
        lastSpeed =currentSpeed;
        isLastSpeed = isCurrentSpeed;
        currentSpeed.header = currentPose.header;
        currentSpeed.twist.linear.x = positionDiff.x() / dt;
        currentSpeed.twist.linear.y = positionDiff.y() / dt;
        currentSpeed.twist.linear.z = positionDiff.z() / dt;
        currentSpeed.twist.angular.x = roll_diff / dt;
        currentSpeed.twist.angular.y = pitch_diff / dt;
        currentSpeed.twist.angular.z = yaw_diff / dt;
        isCurrentSpeed = true;

        segment->twist.twist = currentSpeed.twist;
    }

    void fillHanpAcceleration(hanp_msgs::TrackedSegment* segment){
        double dt = (currentSpeed.header.stamp - lastSpeed.header.stamp).toSec();
        tf::Vector3 lspeedDiff(currentSpeed.twist.linear.x - lastSpeed.twist.linear.x,
                                currentSpeed.twist.linear.y - lastSpeed.twist.linear.y,
                                currentSpeed.twist.linear.z - lastSpeed.twist.linear.z);
        tf::Vector3 aspeedDiff(currentSpeed.twist.angular.x - lastSpeed.twist.angular.x,
                                currentSpeed.twist.angular.y - lastSpeed.twist.angular.y,
                                currentSpeed.twist.angular.z - lastSpeed.twist.angular.z);
        segment->accel.accel.linear.x = lspeedDiff.x() / dt;
        segment->accel.accel.linear.y = lspeedDiff.y() / dt;
        segment->accel.accel.linear.z = lspeedDiff.z() / dt;
        segment->accel.accel.angular.x = aspeedDiff.x() / dt;
        segment->accel.accel.angular.y = aspeedDiff.y() / dt;
        segment->accel.accel.angular.z = aspeedDiff.z() / dt;
    }

    void fillHanpMarkers(visualization_msgs::MarkerArray* markers){
        visualization_msgs::Marker human_arrow, human_cylinder;

        human_arrow.header.stamp = ros::Time::now();
        human_arrow.header.frame_id = "optitrack";
        human_arrow.type = visualization_msgs::Marker::ARROW;
        human_arrow.action = visualization_msgs::Marker::MODIFY;
        human_arrow.id = 101;
        human_arrow.pose.position.x = currentPose.pose.position.x;
        human_arrow.pose.position.y = currentPose.pose.position.y;
        human_arrow.pose.orientation = tf::createQuaternionMsgFromYaw(
                tf::getYaw(currentPose.pose.orientation));
        human_arrow.scale.x = 0.25 * 2.0;
        human_arrow.scale.y = 0.1;
        human_arrow.scale.z = 0.1;
        human_arrow.color.a = 1.0;
        human_arrow.color.r = 0;
        human_arrow.color.g = 150;
        human_arrow.color.b = 200;
        human_arrow.lifetime = ros::Duration(4.0);

        human_cylinder.header.stamp = ros::Time::now();;
        human_cylinder.header.frame_id = "optitrack";
        human_cylinder.type = visualization_msgs::Marker::CYLINDER;
        human_cylinder.action = visualization_msgs::Marker::MODIFY;
        human_cylinder.id = 1;
        human_cylinder.pose.position.x = currentPose.pose.position.x;
        human_cylinder.pose.position.y = currentPose.pose.position.y;
        human_cylinder.pose.position.z += (1.5 / 2);
// human_cylinder.pose.orientation = body_segment.pose.orientation;
        human_cylinder.scale.x = 0.25 * 2;
        human_cylinder.scale.y = 0.25 * 2;
        human_cylinder.scale.z = 1.5;
        human_cylinder.color.a = 1.0;
        human_cylinder.color.r = 0;
        human_cylinder.color.g = 150;
        human_cylinder.color.b = 200;
        human_cylinder.lifetime = ros::Duration(4.0);

        markers->markers.push_back(human_arrow);
        markers->markers.push_back(human_cylinder);
    }

    void fillAndSendHanp(){
        ros::Time now = ros::Time::now();
        hanp_msgs::TrackedHumans trackedHumans;
        hanp_msgs::TrackedHuman trackedHuman;
        hanp_msgs::TrackedSegment trackedSegment;
        visualization_msgs::MarkerArray humansMarkers;
        trackedSegment.type = hanp_msgs::TrackedSegmentType::TORSO;
        trackedSegment.pose.pose = currentPose.pose;
        fillHanpSpeed(&trackedSegment);
        if (isLastSpeed) {
            fillHanpAcceleration(&trackedSegment);
            trackedHuman.track_id = 1;
            trackedHuman.segments.push_back(trackedSegment);
            trackedHumans.humans.push_back(trackedHuman);
            trackedHumans.header.frame_id = "optitrack";
            trackedHumans.header.stamp = now;

            fillHanpMarkers(&humansMarkers);

            hanpPublisher_.publish(trackedHumans);
            hanpMarkerPublisher_.publish(humansMarkers);
        }
    }


    void updateMocapPersonPose(const optitrack::or_pose_estimator_state::ConstPtr& msg){
        geometry_msgs::Pose pose_received;
        tf::Transform tf_received;
        if(!msg->pos.empty()){
            //Setting position
            //Setting position
            pose_received.position.x=msg->pos[0].x;
            pose_received.position.y=msg->pos[0].y;
            pose_received.position.z=msg->pos[0].z;
            //Setting orientation
            pose_received.orientation.x=msg->pos[0].qx;
            pose_received.orientation.y=msg->pos[0].qy;
            pose_received.orientation.z=msg->pos[0].qz;
            pose_received.orientation.w=msg->pos[0].qw;

            tf::poseMsgToTF(pose_received, tf_received);

            tfOptitrack2Human_.setOrigin(tf_received.getOrigin());
            tfOptitrack2Human_.setRotation(tf_received.getRotation());

            lastPose = currentPose;
            isLastPose = isCurrentPose;
            currentPose = geometry_msgs::PoseStamped();
            currentPose.header.frame_id ="optitrack";
            currentPose.header.stamp = ros::Time::now();
            currentPose.pose = pose_received;
            isCurrentPose = true;
        }
    };

};

int main(int argc, char** argv){
    ros::init(argc, argv, "mocap_human_localization");
    ros::NodeHandle node;
    MocapHumanLocalization mchl(node);
    ros::spin();
    return 0;
};
