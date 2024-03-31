using UnityEngine;

namespace ProBridge
{
    [AddComponentMenu("ProBridge/Host")]
    public class ProBridgeHost : MonoBehaviour
    {
        public string addr = "127.0.0.1";
        public int port = 47778;
    }
}