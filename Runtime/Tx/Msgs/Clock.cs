using System;
using UnityEngine;

namespace ProBridge.Tx
{
    [AddComponentMenu("ProBridge/Tx/Clock")]
    public class Clock : ProBridgeTxMsg<ROS.Msgs.Rosgraph.Clock>
    {
        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data = new ROS.Msgs.Rosgraph.Clock(ts);
            return base.GetMsg(ts);
        }

        protected override string GetMsgType()
        {
            return "rosgraph_msgs.msg.Clock";
        }
    }
}
