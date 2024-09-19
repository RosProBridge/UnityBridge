using System;
using ProBridge.Utils;
using UnityEngine;

namespace ProBridge.Rx
{
    public abstract class ProBridgeRx<T> : MonoBehaviour where T : std_msgs.IRosMsg, new()
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
            
            if (msg.n != topic)
                return;

            try
            {
                OnMessage(CDRSerializer.Deserialize<T>((byte[])msg.d));
            }
            catch(Exception e)
            {
                Debug.LogError($"Failed to deserialize message for {msg.n} of type {msg.t} : {e}");
            }
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