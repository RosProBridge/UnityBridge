using UnityEngine;

namespace ProBridge.Utils
{
    public static class RosConvertor
    {
        /// <summary>
        /// Convert coordinate system to ROS
        /// <para>[out] UNITY:  X-Right(Easting)    Y-Up               Z-Forward(Northing)   </para>
        /// <para>[in]  ROS:    X-Forward(Easting)  Y-Left(Northing)   Z-Up                  </para>
        /// </summary>
        public static geometry_msgs.msg.Vector3 ToRos(this Vector3 v)
        {
            return new geometry_msgs.msg.Vector3() { x = v.z, y = -v.x, z = v.y };
        }

        /// <summary>
        /// Convert coordinate system from ROS
        /// <para>[in]  ROS:    X-Forward(Easting)  Y-Left(Northing)   Z-Up                  </para>
        /// <para>[out] UNITY:  X-Right(Easting)    Y-Up               Z-Forward(Northing)   </para>
        /// </summary>
        public static Vector3 FromRos(this geometry_msgs.msg.Vector3 v)
        {
            return new Vector3((float)-v.y, (float)v.z, (float)v.x);
        }

        public static Vector3 FromRos(this geometry_msgs.msg.Point v)
        {
            return new Vector3((float)-v.y, (float)v.z, (float)v.x);
        }

        /// <summary>
        /// Convert coordinate system from/to ROS
        /// <para>[in]  ROS:    X-Roll(left-to-right +)     Y-Pith(down-to-up -)    Z-Yaw(counterclockwise)   </para>
        /// <para>[out] UNITY:  X-Pith(down-to-up -)        Y-Yaw(clockwise)        Z-Roll(left-to-right -)   </para>
        /// </summary>
        public static geometry_msgs.msg.Vector3 ToRosAngular(this Vector3 v)
        {
            return new geometry_msgs.msg.Vector3() { x = -v.z, y = v.x, z = -v.y };
        }

        public static Vector3 FromRosAngular(this geometry_msgs.msg.Vector3 v)
        {
            return new Vector3((float)v.y, (float)-v.z, (float)-v.x);
        }

        public static Quaternion FromRos(this geometry_msgs.msg.Quaternion q)
        {
            return new Quaternion((float)q.y, (float)-q.z, (float)-q.x, (float)q.w);
        }

        public static geometry_msgs.msg.Quaternion ToRos(this Quaternion q)
        {
            return new geometry_msgs.msg.Quaternion() { x = -q.z, y = q.x, z = -q.y, w = q.w };
        }

        public static Color ToUnityColor(this std_msgs.msg.ColorRGBA c)
        {
            return new Color(c.r, c.g, c.b, c.a);
        }
    }
}