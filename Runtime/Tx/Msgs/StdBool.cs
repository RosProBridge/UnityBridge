namespace ProBridge.Tx
{
    public class StdBool : ProBridgeTxMsgStd<bool>
    {
        protected override string GetMsgType()
        {
            return "std_msgs.msg.Bool";
        }
    }
}
