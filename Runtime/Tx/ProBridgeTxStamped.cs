using System;

namespace ProBridge.Tx
{
    public abstract class ProBridgeTxStamped<T> : ProBridgeTx<T> where T : std_msgs.IRosMsg, std_msgs.msg.IStamped, new()
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