using UnityEngine;

namespace ProBridge.Tx.Chassis
{
    [AddComponentMenu("ProBridge/Tx/Chassis/Signals")]
    public class Signals : ProBridgeTxMsgStamped<ROS.Msgs.Chassis.ChassisSignals>
    {
        protected override string GetMsgType()
        {
            return "chassis_msgs.msg.ChassisSignals";
        }
    }
}