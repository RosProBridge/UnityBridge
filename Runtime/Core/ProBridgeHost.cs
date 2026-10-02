using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using NetMQ;
using UnityEngine;
using UnityEngine.Profiling;
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

        // Sender thread: builds frames (header, compression) and owns the socket sends,
        // so the main thread only serializes. Frames are sent in order; the oldest is dropped on overflow.
        private const int MaxQueuedFrames = 64;
        private const int StopTimeoutMs = 2000;

        private struct OutgoingFrame
        {
            public Dictionary<string, object> header;
            public MemoryStream payload;
            public int compressionLevel;
        }

        private readonly Queue<OutgoingFrame> _sendQueue = new Queue<OutgoingFrame>();
        private readonly object _sendLock = new object();
        private Thread _sendThread;
        private bool _sendStopping;
        private int _droppedFrames;

        public void SetupMonitor()
        {
            if (pushSocket == null || _connectionMonitor != null)
                return;

            _connectionMonitor = new ProBridgeConnectionMonitor(
                pushSocket,
                // Unique per instance: the previous scene's monitor may still hold its endpoint.
                $"inproc://monitor-{addr}:{port}-{Guid.NewGuid():N}",
                ProBridgeConnectionMonitor.Mode.Connect);

            _connectionMonitor.PeerConnected += (s, e) => onSubscriberConnect?.Invoke(this, e);
        }

        private void Update()
        {
            PumpConnectionEvents();
        }

        /// <summary>
        /// Queues a serialized message for sending. Takes ownership of <paramref name="payload"/> (a pooled stream).
        /// </summary>
        internal void EnqueueFrame(Dictionary<string, object> header, MemoryStream payload, int compressionLevel)
        {
            lock (_sendLock)
            {
                if (_sendStopping || pushSocket == null)
                {
                    ProBridge.ReturnStream(payload);
                    return;
                }

                if (_sendQueue.Count >= MaxQueuedFrames)
                {
                    ProBridge.ReturnStream(_sendQueue.Dequeue().payload);
                    _droppedFrames++;
                }

                _sendQueue.Enqueue(new OutgoingFrame { header = header, payload = payload, compressionLevel = compressionLevel });

                if (_sendThread == null)
                {
                    _sendThread = new Thread(SendLoop) { IsBackground = true, Name = $"ProBridge sender {addr}:{port}" };
                    _sendThread.Start();
                }

                Monitor.Pulse(_sendLock);
            }
        }

        private void SendLoop()
        {
            Profiler.BeginThreadProfiling("ProBridge", $"Sender {addr}:{port}");
            byte[] frame = null;
            try
            {
                while (true)
                {
                    OutgoingFrame item;
                    int dropped;
                    lock (_sendLock)
                    {
                        while (_sendQueue.Count == 0 && !_sendStopping)
                            Monitor.Wait(_sendLock);
                        if (_sendQueue.Count == 0)
                            return; // stopping and drained
                        item = _sendQueue.Dequeue();
                        dropped = _droppedFrames;
                        _droppedFrames = 0;
                    }

                    if (dropped > 0)
                        Debug.LogWarning($"[ProBridgeHost {addr}:{port}] Send queue overflow: {dropped} message(s) dropped.");

                    try
                    {
                        Profiler.BeginSample("ProBridge.BuildFrame");
                        int length = ProBridge.BuildFrame(item.header, item.payload, item.compressionLevel, ref frame);
                        Profiler.EndSample();

                        Profiler.BeginSample("ProBridge.SocketSend");
                        pushSocket?.TrySendFrame(frame, length);
                        Profiler.EndSample();
                    }
                    catch (Exception e)
                    {
                        Debug.LogWarning($"[ProBridgeHost {addr}:{port}] Failed to send a message: {e.Message}");
                    }
                    finally
                    {
                        ProBridge.ReturnStream(item.payload);
                    }
                }
            }
            finally
            {
                Profiler.EndThreadProfiling();
            }
        }

        // Sends what is queued (e.g. a service response right before a scene reload), then stops the thread.
        private void StopSender()
        {
            Thread thread;
            lock (_sendLock)
            {
                _sendStopping = true;
                thread = _sendThread;
                Monitor.Pulse(_sendLock);
            }

            if (thread != null && !thread.Join(StopTimeoutMs))
                Debug.LogWarning($"[ProBridgeHost {addr}:{port}] Sender thread did not stop in time.");
        }

        public void Dispose()
        {
            StopSender();
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
