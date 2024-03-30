using ProBridge;

namespace ProBridge
{
    public class AckermannDrive : ProBridgeMsgStamped<ROS.Msgs.Ackermann.AckermannDrive>
    {
        protected override string GetMsgType()
        {
            return "ackermann_msgs.msg.AckermannDrive";
        }
    }
}