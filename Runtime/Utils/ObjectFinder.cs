using UnityEngine;

namespace ProBridge.Utils
{
    /// <summary>
    /// Scene object search without deprecated APIs across Unity versions:
    /// FindObjectOfType / FindObjectsOfType are obsolete since 2022.2+ (replaced by *ByType),
    /// and the FindObjectsSortMode parameter is obsolete since 6.5.
    /// </summary>
    public static class ObjectFinder
    {
        /// <summary>Any active object of type <typeparamref name="T"/>, or null.</summary>
        public static T FindAny<T>() where T : Object
        {
#if UNITY_2022_2_OR_NEWER
            return Object.FindAnyObjectByType<T>();
#else
            return Object.FindObjectOfType<T>();
#endif
        }

        /// <summary>All objects of type <typeparamref name="T"/>, unsorted.</summary>
        public static T[] FindAll<T>(bool includeInactive = false) where T : Object
        {
#if UNITY_6000_5_OR_NEWER
            return Object.FindObjectsByType<T>(includeInactive ? FindObjectsInactive.Include : FindObjectsInactive.Exclude);
#elif UNITY_2022_2_OR_NEWER
            return Object.FindObjectsByType<T>(includeInactive ? FindObjectsInactive.Include : FindObjectsInactive.Exclude,
                FindObjectsSortMode.None);
#else
            return Object.FindObjectsOfType<T>(includeInactive);
#endif
        }
    }
}
