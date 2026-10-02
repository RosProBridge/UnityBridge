using System;
using System.Threading.Tasks;
using ProBridge.Utils;
using UnityEngine;

namespace ProBridge.Rx
{
    public abstract class ProBridgeRx<T> : MonoBehaviour where T : std_msgs.IRosMsg, new()
    {
        public string topic = "";

        protected abstract void OnMessage(T msg);

        private T __msg = new();

        private async void GetMsg(ProBridge.Msg msg)
        {
            if (!isActiveAndEnabled || topic == "")
                return;

            if (msg.k != null) // service call
                return;

            if (msg.t != __msg.GetRosType())
                return;

            if (msg.n != topic)
                return;

            var data = (byte[])msg.d;
            try
            {
                var deserialized = await Task.Run(() => CDRSerializer.Deserialize<T>(data));

                if (this == null || !isActiveAndEnabled)
                    return;

                OnMessage(deserialized);
            }
            catch (Exception ex)
            {
                Debug.LogError($"Failed to deserialize message for {msg.n} of type {msg.t}: {ex}");
            }
        }

        private void Awake()
        {
            var srv = FindObjectOfType<ProBridgeServer>();
            if (srv == null)
                return;

            srv.MessageEvent.AddListener(GetMsg);
        }

        private void OnDestroy()
        {
            var srv = FindObjectOfType<ProBridgeServer>();
            if (srv == null)
                return;

            srv.MessageEvent.RemoveListener(GetMsg);
        }
    }
}
