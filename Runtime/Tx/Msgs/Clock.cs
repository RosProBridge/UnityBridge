using System;
using UnityEngine;

namespace ProBridge.Tx
{
    [AddComponentMenu("ProBridge/Tx/Clock")]
    public class Clock : ProBridgeTx<rosgraph_msgs.msg.Clock>
    {
        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data.clock = ts;
            return base.GetMsg(ts);
        }
    }
}
