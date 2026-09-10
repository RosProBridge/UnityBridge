using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

namespace ProBridge
{
    [AddComponentMenu("ProBridge/Server")]
    [RequireComponent(typeof(InitializationManager))]
    public class ProBridgeServer : ProBridgeSingletone<ProBridgeServer>, IDisposable
    {
        [Serializable]
        public class MsgEvent : UnityEvent<ProBridge.Msg>
        {
        }

        [Serializable]
        public class StatusEvent : UnityEvent<bool>
        {
        }

        #region Inspector

        public string ip = "127.0.0.1";
        public int port = 47777;
        public int queueBuffer = 100;

        #endregion

        public MsgEvent MessageEvent { get; } = new MsgEvent();

        /// <summary>
        /// Raised on the Unity main thread when a remote PUSH client connects to or disconnects from this PULL server.
        /// Check <see cref="IsConnected"/> for the current snapshot after subscribing.
        /// </summary>
        public event Action<bool> ConnectionStatusChanged;

        /// <summary>
        /// Same as <see cref="ConnectionStatusChanged"/>, for <c>AddListener</c> style subscriptions.
        /// Does not replay the current status; read <see cref="IsConnected"/> after subscribing if needed.
        /// </summary>
        public StatusEvent ConnectionStatusEvent { get; } = new StatusEvent();

        public bool IsConnected => Bridge != null && Bridge.IsConnected;

        public static TimeSpan SimTime { get; private set; } = new TimeSpan(DateTime.UtcNow.Ticks);

        public ProBridge Bridge;

        private Queue<ProBridge.Msg> _queue = new Queue<ProBridge.Msg>();
        [HideInInspector] public long _initTime;

        public void Dispose()
        {
            if (Bridge != null)
            {
                Bridge.onMessageHandler -= OnMsg;
                Bridge.onDebugHandler -= OnLogMessage;
                Bridge.Dispose();
                Bridge.PumpConnectionEvents(OnConnectionStatusChanged);
            }
        }

        private bool _isFirstFrame = true;

        private void OnEnable()
        {
            _isFirstFrame = true;
        }

        private void Update()
        {
            Bridge?.PumpConnectionEvents(OnConnectionStatusChanged);
        }

        private void OnConnectionStatusChanged(bool connected)
        {
            ConnectionStatusChanged?.Invoke(connected);
            ConnectionStatusEvent.Invoke(connected);
        }

        private void FixedUpdate()
        {
            if (_isFirstFrame)
            {
                _isFirstFrame = false;
                _initTime = DateTime.UtcNow.Ticks;
            }

            SimTime = new TimeSpan(_initTime + (long)(Time.fixedTime * TimeSpan.TicksPerSecond));

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