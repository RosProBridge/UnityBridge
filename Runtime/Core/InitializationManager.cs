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
            _server = FindObjectOfType<ProBridgeServer>();
            _hosts = FindObjectsOfType<ProBridgeHost>();
            _tfSender = FindObjectOfType<TfSender>();
            
            
            // Init hosts sockets
            AsyncIO.ForceDotNet.Force();
            foreach (var host in _hosts)
            {
                host.pushSocket = new PushSocket();
                host.pushSocket.Connect($"tcp://{host.addr}:{host.port}");
                host.pushSocket.Options.Linger = new TimeSpan(0, 0, 1);
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
                Debug.Log(ex);
            }
            
            
            // Init tf sender
            if(_tfSender != null)
            {
                _tfSender.Bridge = _server.Bridge;
                if (_tfSender.Bridge == null)
                {
                    enabled = false;
                    Debug.LogWarning("ROS bridge server not initialized.");
                    return;
                }

                _tfSender.UpdateTree();
                _tfSender.host.onSubscriberConnect += _tfSender.SendStaticMsg;
                _tfSender.CallRepeatingMethods();
            }
            else
            {
                Debug.LogWarning("No TFSender found in scene.");
            }
            
            // Init host monitors
            foreach (var host in _hosts)
            {
                host.SetupMonitor();
            }
        }

        private void OnDestroy()
        {
            // De-init hosts sockets
            foreach (var host in _hosts)
            {
                host.Dispose();
            }
            
            // De-init server
            _server.Dispose();
            
            // NetMQ cleanup
            NetMQConfig.Cleanup(false);
        }
    }
}