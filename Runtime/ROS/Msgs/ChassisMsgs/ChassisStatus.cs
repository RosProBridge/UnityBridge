using ProBridge;

namespace ProBridge
{
    public class ChassisStatus : ProBridgeMsgStamped<ROS.Msgs.Chassis.ChassisStatus>
    {
        protected override string GetMsgType()
        {
            return "chassis_msgs.msg.ChassisStatus";
        }
    }
}