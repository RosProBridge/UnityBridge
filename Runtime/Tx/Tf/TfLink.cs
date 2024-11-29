using UnityEngine;

namespace ProBridge.Tx.Tf
{
    [AddComponentMenu("ProBridge/Tx/tf2_msgs/Link")]
    public class TfLink : MonoBehaviour
    {
        public string frame_id = "";
        public bool is_static = false;
        public TfLink[] children = new TfLink[0];

        private TfSender __sender;

        private void Awake()
        {
            __sender = FindObjectOfType<TfSender>();
        }

        private void OnEnable()
        {
            __sender?.LinkAdd(this);
        }

        private void OnDisable()
        {
            __sender?.LinkRemove(this);
        }
    }
}