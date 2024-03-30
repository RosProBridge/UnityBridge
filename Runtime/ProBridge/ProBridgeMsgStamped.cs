using System;

namespace ProBridge
{
    public abstract class ProBridgeMsgStamped<T> : ProBridgeBehaviour<ProBridgeMsgStamped<T>> where T : ROS.Msgs.Std.IStamped, new()
    {
        public string frame_id = "";
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
            data.header.frame_id = frame_id;
            data.header.stamp = ts;

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