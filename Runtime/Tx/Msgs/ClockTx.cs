using System;
using rosgraph_msgs.msg;
using UnityEngine;

namespace ProBridge.Tx
{
    [AddComponentMenu("ProBridge/Tx/rosgraph_msgs/Clock")]
    public class ClockTx : ProBridgeTx<Clock>
    {
        protected override void AfterEnable()
        {
            sendRate = Time.fixedDeltaTime;
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data.clock = ts;
            return base.GetMsg(ts);
        }
    }
}