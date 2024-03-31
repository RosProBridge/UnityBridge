using System;

namespace ProBridge.Tx
{
    public abstract class ProBridgeTxMsgStamped<T> : ProBridgeTxMsg<T> where T : ROS.Msgs.Std.IStamped, new()
    {
        public string frame_id = "";

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data.header.frame_id = frame_id;
            data.header.stamp = ts;

            return base.GetMsg(ts);
        }
    }
}