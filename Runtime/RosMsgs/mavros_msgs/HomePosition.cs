using std_msgs;
using std_msgs.msg;

namespace mavros_msgs
{
    namespace msg
    {
        /// <summary>
        /// MAVLink message: HOME_POSITION
        /// https://mavlink.io/en/messages/common.html#HOME_POSITION
        /// </summary>
        [System.Serializable]
        public class HomePosition: IRosMsg, IStamped
        {
            public string GetRosType()
            {
                return "mavros_msgs.msg.HomePosition";
            }

            public std_msgs.msg.Header header {get;set;} = new std_msgs.msg.Header();

            /// <summary>
            /// geodetic coordinates in WGS-84 datum
            /// </summary>
            public geographic_msgs.msg.GeoPoint geo {get;set;} = new geographic_msgs.msg.GeoPoint();

            /// <summary>
            /// local position
            /// </summary>
            public geometry_msgs.msg.Point position = new geometry_msgs.msg.Point();

            /// <summary>
            /// XXX: verify field name (q[4])
            /// </summary>
            public geometry_msgs.msg.Quaternion orientation = new geometry_msgs.msg.Quaternion();

            /// <summary>
            /// position of the end of approach vector
            /// </summary>
            public geometry_msgs.msg.Vector3 approach = new geometry_msgs.msg.Vector3();
        }
    }
}