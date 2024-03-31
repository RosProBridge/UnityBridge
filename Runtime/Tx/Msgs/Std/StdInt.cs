using UnityEngine;

namespace ProBridge.Tx
{
    [AddComponentMenu("ProBridge/Tx/Std/Int")]
    public class StdInt : ProBridgeTxMsgStd<int>
    {
        protected override string GetMsgType()
        {
            return "std_msgs.msg.Int32";
        }
    }
}