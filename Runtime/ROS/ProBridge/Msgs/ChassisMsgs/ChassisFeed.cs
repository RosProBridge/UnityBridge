using ProBridge;

namespace ProBridge
{
    public class ChassisFeed : ProBridgeMsgStamped<ROS.Msgs.Chassis.ChassisFeed>
    {
        protected override string GetMsgType()
        {
            return "chassis_msgs.msg.ChassisFeed";
        }
    }
}