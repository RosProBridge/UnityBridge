namespace ProBridge
{
    public class ProBridgeMsgStdBool : ProBridgeMsgStd<bool>
    {
        protected override string GetMsgType()
        {
            return "std_msgs.msg.Bool";
        }
    }
}
