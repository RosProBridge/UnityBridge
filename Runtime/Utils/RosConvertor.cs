using UnityEngine;

namespace ProBridge.Utils
{
    /// <summary>
    /// Convert coordinate system from/to ROS
    /// <para>[in]  ROS:    X-Forward(Easting)  Y-Left(Northing)   Z-Up                  </para>
    /// <para>[out] UNITY:  X-Right(Easting)    Y-Up               Z-Forward(Northing)   </para>
    /// </summary>
    public static class RosConvertor
    {
        public static ROS.Msgs.Geometry.Vector3 ToRos(this Vector3 v)
        {
            return new ROS.Msgs.Geometry.Vector3() { x = v.z, y = -v.x, z = v.y };
        }

        /// <summary>
        /// Convert coordinate system from/to ROS
        /// <para>[in]  ROS:    X-Roll(left-to-right +)     Y-Pith(down-to-up -)    Z-Yaw(counterclockwise)   </para>
        /// <para>[out] UNITY:  X-Pith(down-to-up -)        Y-Yaw(clockwise)        Z-Roll(left-to-right -)   </para>
        /// </summary>
        public static ROS.Msgs.Geometry.Vector3 ToRosAngular(this Vector3 v)
        {
            return new ROS.Msgs.Geometry.Vector3() { x = -v.z, y = v.x, z = -v.y };
        }

        public static Quaternion FromRos(this Quaternion q)
        {
            return Quaternion.AngleAxis(90, new Vector3(0, 1, 0)) * new Quaternion(q.y, -q.z, -q.x, q.w);
        }

        public static ROS.Msgs.Geometry.Quaternion ToRos(this Quaternion q)
        {
            // q = Quaternion.AngleAxis(-90, Vector3.up) * q;
            var rq = new Quaternion(-q.z, q.x, -q.y, q.w);
            return new ROS.Msgs.Geometry.Quaternion() { x = rq.x, y = rq.y, z = rq.z, w = rq.w };
        }
    }
}