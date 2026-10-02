using System;
using System.Collections.Generic;
using ProBridge.Utils;
using UnityEngine;

namespace ProBridge.Srv
{
    /// <summary>
    /// Client of a ROS service. The service must be listed in "services" of the ROS bridge config,
    /// in the group whose hosts include this Unity instance (responses are sent there).
    /// </summary>
    public class ProBridgeServiceClient<TRequest, TResponse> : MonoBehaviour
        where TRequest : std_msgs.IRosMsg, new()
        where TResponse : std_msgs.IRosMsg, new()
    {
        [Tooltip("Host connected to the ROS bridge: requests are sent through it.")]
        public ProBridgeHost host;

        [Tooltip("Service name, e.g. /map/save.")]
        public string service = "";

        [Tooltip("Seconds to wait for the response (real time).")]
        [Min(0.1f)] public float timeout = 10f;

        /// <summary>Service type, e.g. "std_srvs.srv.Trigger".</summary>
        public string ServiceType { get; private set; }

        private class PendingCall
        {
            public Action<TResponse> onResponse;
            public Action<string> onError;
            public float deadline;
        }

        private readonly Dictionary<long, PendingCall> _pending = new Dictionary<long, PendingCall>();
        private readonly List<long> _expired = new List<long>();
        private long _nextId = 1;
        private ProBridgeServer _server;

        protected virtual void Awake()
        {
            ServiceType = ServiceMsg.ServiceTypeOf<TRequest>();

            _server = FindObjectOfType<ProBridgeServer>();
            if (_server != null)
                _server.MessageEvent.AddListener(OnBridgeMessage);
        }

        protected virtual void OnDestroy()
        {
            if (_server != null)
                _server.MessageEvent.RemoveListener(OnBridgeMessage);
            FailAll("Client destroyed");
        }

        /// <summary>
        /// Calls the service. Exactly one of the callbacks is invoked later on the main thread:
        /// <paramref name="onResponse"/> with the response, or <paramref name="onError"/> with the reason
        /// (not connected, timeout). A failure reported by the ROS bridge comes as a response
        /// (for std_srvs: success = false, message = reason).
        /// </summary>
        public void Call(TRequest request, Action<TResponse> onResponse, Action<string> onError = null)
        {
            var bridge = _server != null ? _server.Bridge : null;
            if (!host || bridge == null || !host.IsConnected)
            {
                Fail(onError, "ROS bridge is not connected");
                return;
            }

            long id = _nextId++;
            _pending[id] = new PendingCall
            {
                onResponse = onResponse,
                onError = onError,
                deadline = Time.realtimeSinceStartup + timeout
            };
            bridge.SendMsg(host, ServiceMsg.Create(service, ServiceType, ProBridge.Msg.KindRequest, id, request, _server.port));
        }

        protected virtual void Update()
        {
            if (_pending.Count == 0)
                return;

            float now = Time.realtimeSinceStartup;
            foreach (var pair in _pending)
                if (now >= pair.Value.deadline)
                    _expired.Add(pair.Key);

            foreach (var id in _expired)
            {
                var call = _pending[id];
                _pending.Remove(id);
                Fail(call.onError, $"No response within {timeout:0.#} s");
            }
            _expired.Clear();
        }

        private void OnBridgeMessage(ProBridge.Msg msg)
        {
            if (msg.k != ProBridge.Msg.KindResponse || msg.n != service || msg.t != ServiceType)
                return;
            if (!_pending.TryGetValue(msg.id, out var call))
                return; // timed out, or a call of another client of the same service
            _pending.Remove(msg.id);

            TResponse response;
            try
            {
                response = CDRSerializer.Deserialize<TResponse>((byte[])msg.d);
            }
            catch (Exception e)
            {
                Fail(call.onError, $"Failed to deserialize response: {e.Message}");
                return;
            }

            call.onResponse?.Invoke(response);
        }

        private void FailAll(string reason)
        {
            foreach (var call in _pending.Values)
                Fail(call.onError, reason);
            _pending.Clear();
        }

        private void Fail(Action<string> onError, string reason)
        {
            if (onError != null)
                onError(reason);
            else
                Debug.LogWarning($"[{service}] {reason}", this);
        }
    }
}
