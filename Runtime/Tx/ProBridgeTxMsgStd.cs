using System;

namespace ProBridge.Tx
{
    public abstract class ProBridgeTxMsgStd<T> : ProBridgeTx<ProBridgeTxMsgStd<T>> where T : IConvertible
    {
        private struct Data
        {
            public T data;

            public Data(T value)
            {
                data = value;
            }
        }

        public T value;

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
                d = new Data(value)
            };
        }
    }
}