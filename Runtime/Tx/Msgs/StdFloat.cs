namespace ProBridge.Tx
{
    public class StdFloat : ProBridgeTxMsgStd<float>
    {
        protected override string GetMsgType()
        {
            return "std_msgs.msg.Float32";
        }
    }
}