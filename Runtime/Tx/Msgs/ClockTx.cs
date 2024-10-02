using System;
using rosgraph_msgs.msg;
using UnityEngine;

namespace ProBridge.Tx
{
    [AddComponentMenu("ProBridge/Tx/Clock")]
    public class ClockTx : ProBridgeTx<Clock>
    {
        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data.clock = ts;
            return base.GetMsg(ts);
        }
    }
}