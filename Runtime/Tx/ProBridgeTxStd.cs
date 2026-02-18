using System;
using std_msgs;
using std_msgs.msg;
using UnityEngine;

namespace ProBridge.Tx.Std
{
    public abstract class ProBridgeTxStd<T, U> : ProBridgeTx<T> where T : StdMsg<U>, IRosMsg, new() where U : IConvertible
    {
        [Space(20)]
        public U value;

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data.data = value;
            return base.GetMsg(ts);
        }
    }
}