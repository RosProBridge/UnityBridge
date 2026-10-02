using System;
using NetMQ;
using NetMQ.Sockets;
using ProBridge.Tx.Tf;
using UnityEngine;


namespace ProBridge
{

    [DefaultExecutionOrder(-10000)]
    public class InitializationManager : ProBridgeSingletone<InitializationManager>
    {
        private ProBridgeServer _server;
        private ProBridgeHost[] _hosts;
        private TfSender _tfSender;

        private void Awake()
        {
            _hosts = FindObjectsOfType<ProBridgeHost>(true);
            _server = FindObjectOfType<ProBridgeServer>();
            _tfSender = FindObjectOfType<TfSender>();

            try
            {
                // Subscribe before connecting: the first peer connection may be reported right away
                if (_tfSender != null)
                    _tfSender.host.onSubscriberConnect += _tfSender.SendStaticMsg;

                // Init hosts sockets. The monitor must be attached before Connect, otherwise a fast
                // connection (e.g. ROS already running when the scene is reloaded) is never reported.
                AsyncIO.ForceDotNet.Force();
                ProBridgeBufferPool.Install();
                foreach (var host in _hosts)
                {
                    host.pushSocket = new PushSocket();
                    host.pushSocket.Options.Linger = new TimeSpan(0, 0, 1);
                    host.SetupMonitor();
                    host.pushSocket.Connect($"tcp://{host.addr}:{host.port}");
                }

                // Init server
                try
                {
                    _server.Bridge = new ProBridge(_server.port, _server.ip);
                    _server.Bridge.onMessageHandler += _server.OnMsg;
                    _server.Bridge.onDebugHandler += _server.OnLogMessage;
                }
                catch (Exception ex)
                {
                    _server.Bridge = null;
                    Debug.LogError(ex);
                    return;
                }

                // Init tf sender
                if (_tfSender != null)
                {
                    _tfSender.Bridge = _server.Bridge;
                    _tfSender.CallRepeatingMethods();
                }
                else
                {
                    Debug.LogWarning("No TFSender found in scene.");
                }
            }
            catch (Exception e)
            {
                Debug.Log("Failed to setup ProBridge: " + e);
                OnDestroy();
            }
        }

        private void OnDestroy()
        {
            // De-init hosts sockets
            foreach (var host in _hosts)
            {
                host?.Dispose();
            }

            // De-init server
            _server?.Dispose();

            // NetMQ cleanup
            NetMQConfig.Cleanup(false);
        }
    }
}