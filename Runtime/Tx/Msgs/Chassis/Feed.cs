using UnityEngine;

namespace ProBridge.Tx.Chassis
{
    [AddComponentMenu("ProBridge/Tx/Chassis/Feed")]
    public class Feed : ProBridgeTxMsgStamped<ROS.Msgs.Chassis.ChassisFeed>
    {
        protected override string GetMsgType()
        {
            return "chassis_msgs.msg.ChassisFeed";
        }
    }
}