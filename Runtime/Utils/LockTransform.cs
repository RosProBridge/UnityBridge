using UnityEngine;

namespace ProBridge.Utils
{
    [AddComponentMenu("ProBridge/Utils/LockTransform")]
    public class LockTransform : MonoBehaviour
    {
        public bool lockPosition = false;
        public bool lockRotation = false;

        private Vector3 initialPosition;
        private Quaternion initialRotation;

        private void Start()
        {
            initialPosition = transform.position;
            initialRotation = transform.rotation;
        }

        private void FixedUpdate()
        {
            if (lockPosition)
                transform.position = initialPosition;

            if (lockRotation)
                transform.rotation = initialRotation;
        }
    }
}