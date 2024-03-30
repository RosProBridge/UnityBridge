using ProBridge;

namespace ProBridge
{
    public class ChassisSignals : ProBridgeMsgStamped<ROS.Msgs.Chassis.ChassisSignals>
    {
        protected override string GetMsgType()
        {
            return "chassis_msgs.msg.ChassisSignals";
        }
    }
}