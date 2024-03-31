using UnityEngine;

namespace ProBridge.Tx.Std
{
    [AddComponentMenu("ProBridge/Tx/Std/Bool")]
    public class StdBool : ProBridgeTxMsgStd<bool>
    {
        protected override string GetMsgType()
        {
            return "std_msgs.msg.Bool";
        }
    }
}
