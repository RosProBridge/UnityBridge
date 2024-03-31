using UnityEngine;

namespace ProBridge.Tx.Std
{
    [AddComponentMenu("ProBridge/Tx/Std/Float")]
    public class StdFloat : ProBridgeTxMsgStd<float>
    {
        protected override string GetMsgType()
        {
            return "std_msgs.msg.Float32";
        }
    }
}