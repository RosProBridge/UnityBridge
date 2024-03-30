using System;
using UnityEngine;

namespace ProBridge.ROS
{
    public static class QoS
    {
        public static readonly string DEFAULT = "qos_profile_system_default";
        public static readonly string BEST_EFFORT = "qos_profile_sensor_data";
    }

    /// <summary>
    /// Convert coordinate system from/to ROS
    /// <para>[in]  ROS:    X-Forward(Easting)  Y-Left(Northing)   Z-Up                  </para>
    /// <para>[out] UNITY:  X-Right(Easting)    Y-Up               Z-Forward(Northing)   </para>
    /// </summary>
    public static class RosConvertor
    {
        public static Msgs.Geometry.Vector3 ToRos(this Vector3 v)
        {
            return new Msgs.Geometry.Vector3() { x = v.z, y = -v.x, z = v.y };
        }

        /// <summary>
        /// Convert coordinate system from/to ROS
        /// <para>[in]  ROS:    X-Roll(left-to-right +)     Y-Pith(down-to-up -)    Z-Yaw(counterclockwise)   </para>
        /// <para>[out] UNITY:  X-Pith(down-to-up -)        Y-Yaw(clockwise)        Z-Roll(left-to-right -)   </para>
        /// </summary>
        public static Msgs.Geometry.Vector3 ToRosAngular(this Vector3 v)
        {
            return new Msgs.Geometry.Vector3() { x = -v.z, y = v.x, z = -v.y };
        }

        public static Quaternion FromRos(this Quaternion q)
        {
            return Quaternion.AngleAxis(90, new Vector3(0, 1, 0)) * new Quaternion(q.y, -q.z, -q.x, q.w);
        }

        public static Msgs.Geometry.Quaternion ToRos(this Quaternion q)
        {
            // q = Quaternion.AngleAxis(-90, Vector3.up) * q;
            var rq = new Quaternion(-q.z, q.x, -q.y, q.w);
            return new Msgs.Geometry.Quaternion() { x = rq.x, y = rq.y, z = rq.z, w = rq.w };
        }
    }
}