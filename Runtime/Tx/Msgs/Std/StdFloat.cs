using UnityEngine;

namespace ProBridge.Tx
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