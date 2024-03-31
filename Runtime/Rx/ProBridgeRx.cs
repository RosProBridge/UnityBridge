using UnityEngine;

namespace ProBridge.Rx
{
    public abstract class ProBridgeRx : MonoBehaviour
    {
        public string topic = "";

        [Header("Debug")]
        public bool LogMessage = false;

        public abstract string GetMsgType();
        protected abstract void OnMessage(ProBridge.Msg msg);

        private void GetMsg(ProBridge.Msg msg)
        {
            if (!isActiveAndEnabled || topic == "")
                return;

            if (msg.t != GetMsgType())
                return;

            // TO DO переделать на регулярное выражение 
            if (msg.n != topic)
                return;

            if (LogMessage)
                Debug.Log(msg);

            OnMessage(msg);
        }

        private void Awake()
        {
            var srv = FindObjectOfType<ProBridgeServer>();
            if (srv == null)
                return;

            srv.MessageEvent.AddListener(GetMsg);
        }
    }
}