using UnityEngine;

namespace ProBridge.Tx.Tf
{
    [AddComponentMenu("ProBridge/Tf/Link")]
    public class TfLink : MonoBehaviour
    {
        public string frame_id;
        public TfLink[] children;
    }
}