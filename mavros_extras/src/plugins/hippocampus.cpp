#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <hippocampus_msgs/AttitudeTargetExt.h>
#include <hippocampus_msgs/MixerFeedthrough.h>

#include <Eigen/Core>

namespace mavros
{
namespace extra_plugins
{
class HippoCampusPlugin : public plugin::PluginBase
{
public:
    HippoCampusPlugin() : PluginBase(),
                          nh("~hippocampus"){};

    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);
        attitude_target_sub = nh.subscribe(
            "attitude_control_ext",
            1,
            &HippoCampusPlugin::attitude_control_ext_cb,
            this
         );
        
        mixer_feedthrough_sub = nh.subscribe(
            "mixer_feedthrough",
            1,
            &HippoCampusPlugin::mixer_feedthrough_cb,
            this
        );
    };

    Subscriptions get_subscriptions()
    {
        return {};
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber attitude_target_sub;
    ros::Subscriber mixer_feedthrough_sub;

    void attitude_control_ext_cb(
        const hippocampus_msgs::AttitudeTargetExt::ConstPtr &req)
    {
        mavlink::common::msg::ATTITUDE_CONTROL_EXT target{};
        target.thrust = req->thrust;
        target.roll = req->roll;
        target.pitch = -req->pitch;
        target.yaw = -req->yaw;

        UAS_FCU(m_uas)->send_message_ignore_drop(target);
    }

    void mixer_feedthrough_cb(const hippocampus_msgs::MixerFeedthrough::ConstPtr &req)
    {
        mavlink::common::msg::MIXER_FEEDTHROUGH target{};
        target.motor_ul = req->motor_ul;
        target.motor_ur = req->motor_ur;
        target.motor_ll = req->motor_ll;
        target.motor_lr = req->motor_lr;

        UAS_FCU(m_uas)->send_message_ignore_drop(target);
    }
};
} // namespace extra_plugins
} // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::HippoCampusPlugin,
                       mavros::plugin::PluginBase)
