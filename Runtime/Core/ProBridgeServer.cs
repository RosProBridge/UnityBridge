using System;
using System.Collections.Generic;

using UnityEngine;
using UnityEngine.Events;

namespace ProBridge
{
    [AddComponentMenu("ProBridge/Server")]
    [RequireComponent(typeof(InitializationManager))]
    public class ProBridgeServer : ProBridgeSingletone<ProBridgeServer>
    {
        [Serializable]
        public class MsgEvent : UnityEvent<ProBridge.Msg> { }

        #region Inspector
        public string ip = "127.0.0.1";
        public int port = 47777;
        public int queueBuffer = 100;
        #endregion

        public MsgEvent MessageEvent { get; } = new MsgEvent();

        public static TimeSpan SimTime { get; private set; }

        public ProBridge Bridge;

        private Queue<ProBridge.Msg> _queue = new Queue<ProBridge.Msg>();
        [HideInInspector] public long _initTime;

        public void OnDestroy()
        {
            if (Bridge != null)
            {
                Bridge.onMessageHandler -= OnMsg;
                Bridge.onDebugHandler -= OnLogMessage;
                Bridge.Dispose();
            }
        }
        private void FixedUpdate()
        {
            SimTime = new TimeSpan(_initTime + (long)(Time.time * TimeSpan.TicksPerSecond));
            
            Bridge.TryReceive();

            while (_queue.Count > 0)
                MessageEvent.Invoke(_queue.Dequeue());
        }

        public void OnMsg(ProBridge.Msg msg)
        {
            if (_queue.Count < queueBuffer)
                _queue.Enqueue(msg);
        }

        public void OnLogMessage(string message, ProBridge.MessageType messageType)
        {
            if (messageType == ProBridge.MessageType.Log)
                Debug.Log(message);
            else if (messageType == ProBridge.MessageType.Error)
                Debug.LogError(message);
            else if (messageType == ProBridge.MessageType.Warning)
                Debug.LogWarning(message);
        }
    }
}