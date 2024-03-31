using System;

namespace ProBridge.Tx
{
    public abstract class ProBridgeTxMsg<T> : ProBridgeTx<ProBridgeTxMsg<T>> where T : new()
    {
        public T data = new T();

        protected abstract string GetMsgType();

        protected override void OnStart()
        {
        }

        protected override void OnStop()
        {
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            return new ProBridge.Msg()
            {
                n = topic,
                t = GetMsgType(),
                q = qos,
                d = data
            };
        }
    }
}