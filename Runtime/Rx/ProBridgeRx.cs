using ProBridge.Utils;
using UnityEngine;

namespace ProBridge.Rx
{
    public abstract class ProBridgeRx<T> : MonoBehaviour where T : ROS.Msgs.IRosMsg, new()
    {
        public string topic = "";

        protected abstract void OnMessage(T msg);

        private T __msg = new();

        private void GetMsg(ProBridge.Msg msg)
        {
            if (!isActiveAndEnabled || topic == "")
                return;

            if (msg.t != __msg.GetRosType())
                return;

            // TO DO переделать на регулярное выражение 
            if (msg.n != topic)
                return;

            OnMessage(CDRSerializer.Deserialize<T>((byte[])msg.d));
        }

        private void Awake()
        {
            var srv = FindObjectOfType<ProBridgeServer>();
            if (srv == null)
                return;

            srv.MessageEvent.AddListener(GetMsg);
        }

        private void OnDestroy()
        {
            var srv = FindObjectOfType<ProBridgeServer>();
            if (srv == null)
                return;

            srv.MessageEvent.RemoveListener(GetMsg);
        }
    }
}