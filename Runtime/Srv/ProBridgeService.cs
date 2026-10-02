using System;
using ProBridge.Utils;
using UnityEngine;

namespace ProBridge.Srv
{
    /// <summary>
    /// ROS service served by Unity. The service is advertised to the ROS bridge through <see cref="host"/>
    /// (on enable and on every connection), the bridge creates it in ROS and forwards the calls here.
    /// </summary>
    /// <typeparam name="TRequest">Request type, named "&lt;pkg&gt;.srv.&lt;Service&gt;_Request".</typeparam>
    /// <typeparam name="TResponse">Response type, named "&lt;pkg&gt;.srv.&lt;Service&gt;_Response".</typeparam>
    public abstract class ProBridgeService<TRequest, TResponse> : MonoBehaviour
        where TRequest : std_msgs.IRosMsg, new()
        where TResponse : std_msgs.IRosMsg, new()
    {
        [Tooltip("Host connected to the ROS bridge: the service is advertised and answered through it.")]
        public ProBridgeHost host;

        [Tooltip("Service name, e.g. /sim/reload.")]
        public string service = "";

        /// <summary>Service type, e.g. "std_srvs.srv.Trigger".</summary>
        public string ServiceType { get; private set; }

        private ProBridgeServer _server;
        private ProBridgeHost _subscribedHost;

        /// <summary>
        /// Handles a call on the main thread (in FixedUpdate). An exception is logged and answered with a default response.
        /// </summary>
        protected abstract TResponse OnRequest(TRequest request);

        protected virtual void Awake()
        {
            ServiceType = ServiceMsg.ServiceTypeOf<TRequest>();

            _server = FindObjectOfType<ProBridgeServer>();
            if (_server != null)
                _server.MessageEvent.AddListener(OnBridgeMessage);
        }

        protected virtual void OnEnable()
        {
            if (host)
            {
                _subscribedHost = host;
                _subscribedHost.ConnectionStatusChanged += OnHostConnectionChanged;
            }

            Advertise();
        }

        protected virtual void OnDisable()
        {
            if (_subscribedHost)
                _subscribedHost.ConnectionStatusChanged -= OnHostConnectionChanged;
            _subscribedHost = null;
        }

        protected virtual void OnDestroy()
        {
            if (_server != null)
                _server.MessageEvent.RemoveListener(OnBridgeMessage);
        }

        // The bridge (re)started: it doesn't know the service yet.
        private void OnHostConnectionChanged(bool connected)
        {
            if (connected)
                Advertise();
        }

        private void Advertise()
        {
            if (service == "")
                return;

            // The bridge sends the calls to our server: our IP from the connection, the port from here.
            int replyPort = _server != null ? _server.port : 0;
            Send(ServiceMsg.Create(service, ServiceType, ProBridge.Msg.KindAdvertise, 0, null, replyPort), "advertisement");
        }

        private void OnBridgeMessage(ProBridge.Msg msg)
        {
            if (!isActiveAndEnabled || msg.k != ProBridge.Msg.KindRequest || msg.n != service || msg.t != ServiceType)
                return;

            TRequest request;
            try
            {
                request = CDRSerializer.Deserialize<TRequest>((byte[])msg.d);
            }
            catch (Exception e)
            {
                Debug.LogError($"[{service}] Failed to deserialize request of type {msg.t}: {e}", this);
                return;
            }

            TResponse response;
            try
            {
                response = OnRequest(request);
            }
            catch (Exception e)
            {
                Debug.LogException(e, this);
                response = new TResponse();
            }

            Send(ServiceMsg.Create(service, ServiceType, ProBridge.Msg.KindResponse, msg.id, response), "response");
        }

        private void Send(ProBridge.Msg msg, string what)
        {
            var bridge = _server != null ? _server.Bridge : null;
            if (!host || bridge == null)
            {
                Debug.LogWarning($"[{service}] Can't send the {what}: no host assigned or bridge is not initialized.", this);
                return;
            }

            bridge.SendMsg(host.pushSocket, msg);
        }
    }
}
