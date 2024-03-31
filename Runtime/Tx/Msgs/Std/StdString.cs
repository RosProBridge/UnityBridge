using UnityEngine;

namespace ProBridge.Tx.Std
{
    [AddComponentMenu("ProBridge/Tx/Std/String")]
    public class StdString : ProBridgeTxMsgStd<string>
    {
        protected override string GetMsgType()
        {
            return "std_msgs.msg.String";
        }
    }
}