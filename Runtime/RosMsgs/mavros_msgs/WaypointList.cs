using System;
using std_msgs;


namespace mavros_msgs
{
    namespace msg
    {
        [System.Serializable]
        public class WaypointList : IRosMsg
        {
            public string GetRosType()
            {
                return "mavros_msgs.msg.WaypointList";
            }

            /// <summary>
            /// seq nr of currently active waypoint
            /// waypoints[current_seq].is_current == True
            /// </summary>
            public UInt16 current_seq;

            /// <summary>
            /// list of waypoints
            /// </summary>
            public mavros_msgs.msg.Waypoint[] waypoints;
        }
    }
}