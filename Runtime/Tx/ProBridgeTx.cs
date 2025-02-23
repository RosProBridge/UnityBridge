﻿using System;
using ProBridge.Utils;
using UnityEngine;

namespace ProBridge.Tx
{
    public abstract class ProBridgeTx<T> : MonoBehaviour where T : std_msgs.IRosMsg, new()
    {
        #region Inspector
        public ProBridgeHost host;
        public bool manualSend;
        public float sendRate = 0.025f;
        public string topic = "";
        [Range(0, 2)]
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

        private bool initialized;

        private bool sentHostMissingMsg;

        private void Start()
        {
            if (Bridge == null)
            {
                enabled = false;
                Debug.LogWarning("Don't inited ROS bridge server.");
                return;
            }

            OnStart();
            initialized = true;
            
            if(sendRate>0f && !manualSend) InvokeRepeating(nameof(SendMsg), 0, sendRate);
            
            
        }

        private void OnEnable()
        {
            if(sendRate>0f && !manualSend && initialized) InvokeRepeating(nameof(SendMsg), 0, sendRate);

        }

        private void OnDisable()
        {
            CancelInvoke("SendMsg");
        }

        private void OnDestroy()
        {
            CancelInvoke("SendMsg");
            OnStop();
        }

        protected void SendMsg()
        {
            if (!host)
            {
                if(!sentHostMissingMsg) Debug.LogWarning($"No host assigned for topic {topic}.");
                sentHostMissingMsg = true;
                return;
            }

            sentHostMissingMsg = false;
            
            var st = ProBridgeServer.SimTime.Ticks;
            if (_lastSimTime >= st)
            {
                Debug.LogWarning("Can't send message before update SimTime.");
                return;
            }
            _lastSimTime = st;

            if (!Active || topic == "") return;
            ProBridge.Msg msg;
            try
            {
                msg = GetMsg(ProBridgeServer.SimTime);
            }
            catch (Exception e)
            {
                Debug.LogWarning($"Failed to get message for {topic}, PC might be running slower than requested frequency." + e);
                return;
            }
            OnSendMessage?.Invoke(this, msg);
            if (Bridge != null)
                Bridge.SendMsg(host.pushSocket, msg);
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
                q = qos,
#endif
                d = data
            };
        }

        protected virtual void OnStart() { }
        protected virtual void OnStop() { }
    }
}
