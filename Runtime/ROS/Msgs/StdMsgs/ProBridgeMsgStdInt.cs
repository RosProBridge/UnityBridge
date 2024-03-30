using System;

namespace ProBridge
{
    public class ProBridgeMsgStdInt : ProBridgeMsgStd<int>
    {
        protected override string GetMsgType()
        {
            return "std_msgs.msg.Int32";
        }
    }
}