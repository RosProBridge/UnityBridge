namespace ProBridge.Tx
{
    public class StdString : ProBridgeTxMsgStd<string>
    {
        protected override string GetMsgType()
        {
            return "std_msgs.msg.String";
        }
    }
}