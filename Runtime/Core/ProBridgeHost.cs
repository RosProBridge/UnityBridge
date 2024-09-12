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

        [HideInInspector] public PushSocket pushSocket;

        public event EventHandler onSubscriberConnect;
        private NetMQMonitor monitor;

        private Thread monitoringThread;
        private bool shouldStopMonitoring = false;
        
        public void SetupMonitor()
        {
            monitor = new NetMQMonitor(pushSocket, $"inproc://monitor-{addr}:{port}", SocketEvents.All);
            monitor.Connected += (s, e) => onSubscriberConnect?.Invoke(this, e);
            

            monitor.StartAsync();
        }

        private void OnDisable()
        {
            pushSocket.Close();
            pushSocket?.Dispose();

            monitor.Stop();
            monitor?.Dispose();
        }

        private void OnDestroy()
        {
            NetMQConfig.Cleanup(false); // Must be here to work more than once, and false to not block when there are unprocessed messages.
        }
    }
}