using System;
using UnityEngine;

namespace ProBridge
{
    public abstract class ProBridgeCallback : MonoBehaviour
    {
        public string topicName = "*";
        public float timeOut = 1f;

        public abstract string GetMsgType();
        protected abstract void OnMessage(ProBridge.Msg msg);

        private void GetMsg(ProBridge.Msg msg)
        {
            if (!isActiveAndEnabled)
                return;

            if (msg.t != GetMsgType())
                return;

            // TO DO переделать на регулярное выражение 
            if (msg.n != topicName)
                return;

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