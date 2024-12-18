using UnityEngine;

namespace ProBridge
{
    public class ProBridgeSingletone<T> : MonoBehaviour where T : MonoBehaviour
    {
        private static __instance;
        public static T Instance
        {
            get
            {
                if (__instance == null)
                   __instance = FindObjectOfType<T>();
                return __instance;
            }
        }

        protected void OnValidate()
        {
            if (FindObjectsOfType<T>().Length > 1)

            {
                Debug.LogError("Singleton<" + this.GetType() + "> already has an instance on scene. Component will be destroyed.");
#if UNITY_EDITOR
                UnityEditor.EditorApplication.delayCall -= DestroySelf;
                UnityEditor.EditorApplication.delayCall += DestroySelf;
#endif
            }
        }

        private void DestroySelf()
        {
            if (Application.isPlaying)
                Destroy(this);
            else
                DestroyImmediate(this);
        }
    }
}
