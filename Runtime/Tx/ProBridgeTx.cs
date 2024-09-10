using System;
using ProBridge.Utils;
using UnityEngine;

namespace ProBridge.Tx
{
    public abstract class ProBridgeTx<T> : MonoBehaviour where T : ROS.Msgs.IRosMsg, new()
    {
        #region Inspector
        public ProBridgeHost host;
        public float sendRate = 0.025f;
        public string topic = "";
        [Range(0, 9)]
        public int compressionLevel = 0;


#if ROS_V2
        [Header("QOS")]
        public Qos qos;
#endif
        #endregion

        public bool Active { get; set; } = true;

        public T data { get; } = new T();

        public EventHandler<ProBridge.Msg> OnSendMessage { get; set; } = delegate { };

        private ProBridge Bridge { get { return ProBridgeServer.Instance?.Bridge; } }

        private long _lastSimTime = 0;

        private void Start()
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

            if (Active && topic != "")
            {
                var msg = GetMsg(ProBridgeServer.SimTime);
                OnSendMessage?.Invoke(this, msg);
                if (Bridge != null)
                    Bridge.SendMsg(host.publisher, msg);
            }
        }

        protected virtual ProBridge.Msg GetMsg(TimeSpan ts)
        {
            return new ProBridge.Msg()
            {
#if ROS_V2
                v = 2,
#else
                v = 1,
#endif
                n = topic,
                t = data.GetRosType(),
                c = compressionLevel,
#if ROS_V2
                q = qos.GetValue(),
#endif
                d = data
            };
        }

        protected virtual void OnStart() { }
        protected virtual void OnStop() { }
    }
}
