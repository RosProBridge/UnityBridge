using System;
using NetMQ;
using UnityEngine;
using NetMQ.Sockets;

namespace ProBridge
{
    [AddComponentMenu("ProBridge/Host")]
    public class ProBridgeHost : MonoBehaviour
    {
        public string addr = "127.0.0.1";
        public int port = 47778;

        [HideInInspector] public PublisherSocket publisher;

        private void OnEnable()
        {
            AsyncIO.ForceDotNet.Force();
            publisher = new PublisherSocket();
            publisher.Bind($"tcp://{addr}:{port}");
            publisher.Options.Linger = new TimeSpan(0, 0, 1);
        }

        private void OnDisable()
        {
            publisher?.Dispose();
            NetMQConfig.Cleanup(false); // SECRET INGREDIENT, DO NOT TOUCH
        }
    }
}