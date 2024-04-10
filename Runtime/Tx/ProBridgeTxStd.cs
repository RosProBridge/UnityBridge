using System;
using ProBridge.ROS.Msgs;
using ProBridge.ROS.Msgs.Std;

namespace ProBridge.Tx.Std
{
    public abstract class ProBridgeTxStd<T, U> : ProBridgeTx<T> where T : StdMsg<U>, IRosMsg, new() where U : IConvertible
    {
        public U value;

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data.data = value;
            return base.GetMsg(ts);
        }
    }
}