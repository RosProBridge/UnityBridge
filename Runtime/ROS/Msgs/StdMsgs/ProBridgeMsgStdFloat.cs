namespace ProBridge
{
    public class ProBridgeMsgStdFloat : ProBridgeMsgStd<float>
    {
        protected override string GetMsgType()
        {
            return "std_msgs.msg.Float32";
        }
    }
}