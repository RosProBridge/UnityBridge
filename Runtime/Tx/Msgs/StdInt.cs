namespace ProBridge.Tx
{
    public class StdInt : ProBridgeTxMsgStd<int>
    {
        protected override string GetMsgType()
        {
            return "std_msgs.msg.Int32";
        }
    }
}