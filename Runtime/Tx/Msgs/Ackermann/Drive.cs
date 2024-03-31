using UnityEngine;

namespace ProBridge.Tx.Ackermann
{
    [AddComponentMenu("ProBridge/Tx/Ackermann/Drive")]
    public class Drive : ProBridgeTxMsgStamped<ROS.Msgs.Ackermann.AckermannDrive>
    {
        protected override string GetMsgType()
        {
            return "ackermann_msgs.msg.AckermannDrive";
        }
    }
}