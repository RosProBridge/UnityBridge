using System;
using NetMQ;
using UnityEngine;
using NetMQ.Sockets;
using UnityEngine.Events;

namespace ProBridge
{
    [AddComponentMenu("ProBridge/Host")]
    public class ProBridgeHost : MonoBehaviour, IDisposable
    {
        [Serializable]
        public class StatusEvent : UnityEvent<bool>
        {
        }

        public string addr = "127.0.0.1";
        public int port = 47778;

        [HideInInspector] public PushSocket pushSocket;

        /// <summary>
        /// Fired for every successful PUSH peer connection. Used internally to resend static TF.
        /// May be raised off the Unity main thread.
        /// </summary>
        public event EventHandler onSubscriberConnect;

        /// <summary>
        /// Raised on the Unity main thread when the host connects to or disconnects from the remote PULL server.
        /// Check <see cref="IsConnected"/> for the current snapshot after subscribing.
        /// </summary>
        public event Action<bool> ConnectionStatusChanged;

        /// <summary>
        /// Same as <see cref="ConnectionStatusChanged"/>, for <c>AddListener</c> style subscriptions.
        /// Does not replay the current status; read <see cref="IsConnected"/> after subscribing if needed.
        /// </summary>
        public StatusEvent ConnectionStatusEvent { get; } = new StatusEvent();

        public bool IsConnected => _connectionMonitor != null && _connectionMonitor.IsConnected;

        private ProBridgeConnectionMonitor _connectionMonitor;

        public void SetupMonitor()
        {
            if (pushSocket == null || _connectionMonitor != null)
                return;

            _connectionMonitor = new ProBridgeConnectionMonitor(
                pushSocket,
                $"inproc://monitor-{addr}:{port}",
                ProBridgeConnectionMonitor.Mode.Connect);

            _connectionMonitor.PeerConnected += (s, e) => onSubscriberConnect?.Invoke(this, e);
        }

        private void Update()
        {
            PumpConnectionEvents();
        }

        public void Dispose()
        {
            _connectionMonitor?.Dispose();
            PumpConnectionEvents();
            _connectionMonitor = null;

            pushSocket?.Close();
            pushSocket?.Dispose();
            pushSocket = null;
        }

        private void PumpConnectionEvents()
        {
            _connectionMonitor?.Pump(OnConnectionStatusChanged);
        }

        private void OnConnectionStatusChanged(bool connected)
        {
            ConnectionStatusChanged?.Invoke(connected);
            ConnectionStatusEvent.Invoke(connected);
        }
    }
}
