using System;
using NetMQ.Sockets;
using ProBridge.Tx.Tf;
using UnityEngine;


namespace ProBridge
{
    [DefaultExecutionOrder(-10000)]
    public class InitializationManager : ProBridgeSingletone<InitializationManager>
    {
        [SerializeField] private bool _useConfigFile = false;

        private ProBridgeServer _server;
        private ProBridgeHost[] _hosts;
        private TfSender _tfSender;

        private void OnEnable()
        {
            _server = FindObjectOfType<ProBridgeServer>();
            _hosts = FindObjectsOfType<ProBridgeHost>();
            _tfSender = FindObjectOfType<TfSender>();

            if(_useConfigFile)

            {
                ParamsReader.Load();

                var config = ParamsReader.Value;

                // Get config for hosts
                foreach (var host in _hosts)
                {
                    var proBridgeHost = config.GetObjectByTypeAndName("ProBridgeHost", host.gameObject.name);

                    if (proBridgeHost == null)
                    {
                        Debug.LogWarning("ProBridgeHost configuration not found. Using Default values.");
                        continue;
                    }

                    var port = proBridgeHost.GetField("port");
                    var addr = proBridgeHost.GetField("addr");
                    host.addr = addr;
                    host.port = Convert.ToInt32(port);
                }
                
                // Get config for server
                var proBridgeServer = config.GetObjectByType("ProBridgeServer");

                if (proBridgeServer != null)
                {
                    var port = proBridgeServer.GetField("port");
                    var addr = proBridgeServer.GetField("addr");

                    _server.port = Convert.ToInt32(port);
                    _server.ip = addr;
                }
                else
                {
                    Debug.LogWarning("ProBridgeServer configuration not found. Using Default values.");
                }
            }


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
                _server._initTime = DateTime.Now.Ticks;
                _server.Bridge = new ProBridge(_server.port, _server.ip);
                _server.Bridge.onMessageHandler += _server.OnMsg;
            }
            catch (Exception ex)
            {
                _server.Bridge = null;
                Debug.Log(ex);
            }


            // Init tf sender
            if (_tfSender != null)
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
    }
}