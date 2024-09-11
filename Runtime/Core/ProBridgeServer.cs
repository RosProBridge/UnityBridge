using System;
using System.Collections.Generic;

using UnityEngine;
using UnityEngine.Events;

namespace ProBridge
{
    [AddComponentMenu("ProBridge/Server")]
    [DefaultExecutionOrder(-10000)]
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

        public ProBridge Bridge { get; private set; }

        private Queue<ProBridge.Msg> _queue = new Queue<ProBridge.Msg>();
        private long _initTime;

        public void Start()
        {
            try
            {
                _initTime = DateTime.Now.Ticks;
                Bridge = new ProBridge(port, ip);
                Bridge.onMessageHandler += OnMsg;
            }
            catch (Exception ex)
            {
                Bridge = null;
                Debug.Log(ex);
            }
        }

        public void OnDestroy()
        {
            if (Bridge != null)
            {
                Bridge.onMessageHandler -= OnMsg;
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

        private void OnMsg(ProBridge.Msg msg)
        {
            if (_queue.Count < queueBuffer)
                _queue.Enqueue(msg);
        }
    }
}