using System;
using UnityEngine;

namespace ProBridge.Tx
{
    public abstract class ProBridgeTx<T> : MonoBehaviour where T : MonoBehaviour
    {
        #region Inspector
        public ProBridgeHost host;
        public float sendRate = 0.025f;
        public string topic = "";
        public string qos = ROS.QoS.DEFAULT;
        #endregion

        public bool Active { get; set; } = true;

        public EventHandler<ProBridge.Msg> OnSendMessage { get; set; } = delegate { };

        private ProBridge Bridge { get { return ProBridgeServer.Instance?.Bridge; } }

        private long _lastSimTime = 0;

        private void OnEnable()
        {
            if (Bridge == null)
            {
                enabled = false;
                Debug.LogWarning("Don't inited ROS bridge server.");
                return;
            }

            OnStart();
            InvokeRepeating("SendMsg", 0, sendRate);
        }

        private void OnDisable()
        {
            CancelInvoke("SendMsg");
            OnStop();
        }

        protected void SendMsg()
        {
            var st = ProBridgeServer.SimTime.Ticks;
            if (_lastSimTime >= st)
            {
                Debug.LogWarning("Can't send message before update SimTime.");
                return;
            }
            _lastSimTime = st;

            if (Active)
            {
                var msg = GetMsg(ProBridgeServer.SimTime);
                OnSendMessage?.Invoke(this, msg);
                if (Bridge != null)
                    Bridge.SendMsg(host, msg);
            }
        }

        protected abstract void OnStart();
        protected abstract void OnStop();

        protected abstract ProBridge.Msg GetMsg(TimeSpan ts);
    }
}