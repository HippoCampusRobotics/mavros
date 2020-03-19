#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <hippocampus_msgs/AttitudeTargetExt.h>

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
            "attitude_target/external",
            1,
            &HippoCampusPlugin::attitude_target_cb,
            this);
    };

    Subscriptions get_subscriptions()
    {
        return {};
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber attitude_target_sub;
    void attitude_target_cb(
        const hippocampus_msgs::AttitudeTargetExt::ConstPtr &req)
    {
        mavlink::common::msg::HIPPOCAMPUS_ATTITUDE_TARGET_EXTERNAL target{};
        target.thrust = req->thrust;
        target.roll = req->roll;
        target.pitch = req->pitch;
        target.yaw = req->yaw;

        UAS_FCU(m_uas)->send_message_ignore_drop(target);
    }
}
} // namespace extra_plugins
} // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::HippoCampusPlugin,
                       mavros::plugin::PluginBase)