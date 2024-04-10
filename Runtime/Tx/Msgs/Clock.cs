using System;
using UnityEngine;

namespace ProBridge.Tx
{
    [AddComponentMenu("ProBridge/Tx/Clock")]
    public class Clock : ProBridgeTx<ROS.Msgs.Rosgraph.Clock>
    {
        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data.clock = ts;
            //data = new ROS.Msgs.Rosgraph.Clock(ts);
            return base.GetMsg(ts);
        }
    }
}
