using System;

namespace ProBridge
{
    public class ProBridgeMsgStdString : ProBridgeMsgStd<string>
    {
        protected override string GetMsgType()
        {
            return "std_msgs.msg.String";
        }
    }
}