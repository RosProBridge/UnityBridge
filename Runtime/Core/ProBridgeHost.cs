using System;
using System.Threading;
using NetMQ;
using UnityEngine;
using NetMQ.Sockets;
using NetMQ.Monitoring;

namespace ProBridge
{
    [AddComponentMenu("ProBridge/Host")]
    public class ProBridgeHost : MonoBehaviour
    {
        public string addr = "127.0.0.1";
        public int port = 47778;

        [HideInInspector] public PublisherSocket publisher;

        public event EventHandler onSubscriberConnect;
        private NetMQMonitor monitor;

        private Thread monitoringThread;
        private bool shouldStopMonitoring = false;

        private void OnEnable()
        {
            AsyncIO.ForceDotNet.Force();
            publisher = new PublisherSocket();
            publisher.Bind($"tcp://{addr}:{port}");
            publisher.Options.Linger = new TimeSpan(0, 0, 1);

            shouldStopMonitoring = false;
            monitoringThread = new Thread(StartMonitoring);
            monitoringThread.Start();
        }

        private void StartMonitoring()
        {
            monitor = new NetMQMonitor(publisher, $"inproc://monitor-{addr}:{port}", SocketEvents.All);
            monitor.Accepted += (s, e) => onSubscriberConnect?.Invoke(this, EventArgs.Empty);

            monitor.Start();
        }

        private void OnDisable()
        {
            publisher.Close();
            publisher?.Dispose();

            monitor.Stop();
            monitor?.Dispose();
            if (monitoringThread != null && monitoringThread.Join(1000))
                monitoringThread?.Abort();
        }

        private void OnDestroy()
        {
            NetMQConfig.Cleanup(false); // Must be here to work more than once, and false to not block when there are unprocessed messages.
        }
    }
}