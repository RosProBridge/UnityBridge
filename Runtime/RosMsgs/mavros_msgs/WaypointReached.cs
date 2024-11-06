using System;
using std_msgs;
using std_msgs.msg;


namespace mavros_msgs
{
    namespace msg
    {
        [System.Serializable]
        /// <summary>
        /// That message represent MISSION_ITEM_REACHED
        /// </summary>
        public class WaypointReached : IRosMsg, IStamped
        {
            public string GetRosType()
            {
                return "mavros_msgs.msg.WaypointReached";
            }

            public std_msgs.msg.Header header {get;set;} = new std_msgs.msg.Header();

            /// <summary>
            /// index number of reached waypoint
            /// </summary>
            public System.UInt16 wp_seq;
        }
    }
}