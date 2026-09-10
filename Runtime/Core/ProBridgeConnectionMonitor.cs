using System;
using System.Collections.Concurrent;
using System.Threading;
using NetMQ;
using NetMQ.Monitoring;

namespace ProBridge
{
    /// <summary>
    /// Tracks ZeroMQ peer connections via <see cref="NetMQMonitor"/> and queues status changes
    /// so they can be raised on the Unity main thread.
    /// </summary>
    internal sealed class ProBridgeConnectionMonitor : IDisposable
    {
        public enum Mode
        {
            /// <summary>Outgoing socket that calls Connect (Push). Uses Connected/Disconnected.</summary>
            Connect,

            /// <summary>Incoming socket that calls Bind (Pull). Uses Accepted/Disconnected.</summary>
            Bind
        }

        private readonly NetMQMonitor _monitor;
        private int _connectionCount;
        private readonly ConcurrentQueue<bool> _pending = new ConcurrentQueue<bool>();
        private bool _disposed;

        public bool IsConnected => Volatile.Read(ref _connectionCount) > 0;

        public int ConnectionCount
        {
            get
            {
                int count = Volatile.Read(ref _connectionCount);
                return count < 0 ? 0 : count;
            }
        }

        /// <summary>
        /// Raised for every peer attach (including additional peers). May fire off the Unity main thread.
        /// </summary>
        public event EventHandler PeerConnected;

        public ProBridgeConnectionMonitor(NetMQSocket socket, string endpoint, Mode mode)
        {
            var events = mode == Mode.Bind
                ? SocketEvents.Accepted | SocketEvents.Disconnected
                : SocketEvents.Connected | SocketEvents.Disconnected;

            _monitor = new NetMQMonitor(socket, endpoint, events);

            if (mode == Mode.Bind)
                _monitor.Accepted += (s, e) => OnPeerAttached(s, e);
            else
                _monitor.Connected += (s, e) => OnPeerAttached(s, e);

            _monitor.Disconnected += (s, e) => OnPeerDetached();
            _monitor.StartAsync();
        }

        private void OnPeerAttached(object sender, EventArgs e)
        {
            PeerConnected?.Invoke(sender, e);
            ApplyDelta(+1);
        }

        private void OnPeerDetached()
        {
            ApplyDelta(-1);
        }

        private void ApplyDelta(int delta)
        {
            int current;
            int updated;
            do
            {
                current = Volatile.Read(ref _connectionCount);
                updated = current + delta;
                if (updated < 0)
                    updated = 0;
            } while (Interlocked.CompareExchange(ref _connectionCount, updated, current) != current);

            bool wasConnected = current > 0;
            bool isConnected = updated > 0;
            if (wasConnected != isConnected)
                _pending.Enqueue(isConnected);
        }

        public void Pump(Action<bool> onStatusChanged)
        {
            if (onStatusChanged == null)
                return;

            while (_pending.TryDequeue(out bool connected))
                onStatusChanged(connected);
        }

        public void Dispose()
        {
            if (_disposed)
                return;
            _disposed = true;

            try
            {
                _monitor.Stop();
            }
            catch
            {
                // Monitor may already be stopped when the socket is tearing down.
            }

            try
            {
                _monitor.Dispose();
            }
            catch
            {
                // ignored
            }

            int current = Interlocked.Exchange(ref _connectionCount, 0);
            if (current > 0)
                _pending.Enqueue(false);
        }
    }
}
