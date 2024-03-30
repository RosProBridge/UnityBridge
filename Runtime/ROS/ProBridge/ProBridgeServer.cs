using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

using System;

namespace ProBridge
{
    [DefaultExecutionOrder(-10000)]
    public class ProBridgeServer : Singleton<ProBridgeServer>
    {
        [Serializable]
        public class MsgEvent : UnityEvent<ProBridge.Msg> { }

        #region Inspector
        public int port = 47777;
        public int queueBuffer = 100;
        #endregion

        public MsgEvent MessageEvent { get; } = new MsgEvent();

        public static TimeSpan SimTime { get; private set; }

        public ProBridge Bridge { get; private set; }

        private Queue<ProBridge.Msg> _queue = new Queue<ProBridge.Msg>();
        private long _initTime;


        public void OnEnable()
        {
            try
            {
                _initTime = DateTime.Now.Ticks;
                Bridge = new ProBridge(port);
                Bridge.onMessageHandler += OnMsg;
            }
            catch (Exception ex)
            {
                Bridge = null;
                Debug.Log(ex);
            }
        }

        public void OnDisable()
        {
            if (Bridge != null)
            {
                Bridge.onMessageHandler -= OnMsg;
                Bridge.Dispose();
            }
        }

        private void FixedUpdate()
        {
            SimTime = new TimeSpan((long)(_initTime + Time.time * TimeSpan.TicksPerSecond));
        }

        public void Update()
        {
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