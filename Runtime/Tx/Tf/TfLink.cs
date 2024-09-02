using UnityEngine;

namespace ProBridge.Tx.Tf
{
    [AddComponentMenu("ProBridge/Tf/Link")]
    public class TfLink : MonoBehaviour
    {
        public string frame_id = "";
        public bool fixed_frame;
        public bool is_static = false;
        public TfLink[] children = new TfLink[0];

        public void Start()
        {
            if (fixed_frame)
                transform.parent = null;
        }
    }
}