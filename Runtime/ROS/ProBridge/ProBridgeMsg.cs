using System;

namespace ProBridge
{
    public abstract class ProBridgeMsg<T> : ProBridgeBehaviour<ProBridgeMsg<T>> where T : new()
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