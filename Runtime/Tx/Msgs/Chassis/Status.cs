using UnityEngine;

namespace ProBridge.Tx.Chassis
{
    [AddComponentMenu("ProBridge/Tx/Chassis/Status")]
    public class Status : ProBridgeTxMsgStamped<ROS.Msgs.Chassis.ChassisStatus>
    {
        protected override string GetMsgType()
        {
            return "chassis_msgs.msg.ChassisStatus";
        }
    }
}