using System;
using std_msgs;


namespace mavros_msgs
{
    namespace msg
    {
        /// <summary>
        /// ROS representation of MAVLink MISSION_ITEM
        /// See mavlink documentation
        /// </summary>
        [System.Serializable]
        public class Waypoint : IRosMsg
        {
            public string GetRosType()
            {
                return "mavros_msgs.msg.Waypoint";
            }

            public const byte FRAME_GLOBAL = 0;
            public const byte FRAME_LOCAL_NED = 1;
            public const byte FRAME_MISSION = 2;
            public const byte FRAME_GLOBAL_REL_ALT = 3;
            public const byte FRAME_LOCAL_ENU = 4;
            public const byte FRAME_GLOBAL_INT = 5;
            public const byte FRAME_GLOBAL_RELATIVE_ALT_INT = 6;
            public const byte FRAME_LOCAL_OFFSET_NED = 7;
            public const byte FRAME_BODY_NED = 8;
            public const byte FRAME_BODY_OFFSET_NED = 9;
            public const byte FRAME_GLOBAL_TERRAIN_ALT = 10;
            public const byte FRAME_GLOBAL_TERRAIN_ALT_INT = 11;
            public const byte FRAME_BODY_FRD = 12;
            public const byte FRAME_RESERVED_13 = 13;
            public const byte FRAME_RESERVED_14 = 14;
            public const byte FRAME_RESERVED_15 = 15;
            public const byte FRAME_RESERVED_16 = 16;
            public const byte FRAME_RESERVED_17 = 17;
            public const byte FRAME_RESERVED_18 = 18;
            public const byte FRAME_RESERVED_19 = 19;
            public const byte FRAME_LOCAL_FRD = 20;
            public const byte FRAME_LOCAL_FLU = 21;

            public byte frame;

            /// <summary>
            /// see enum MAV_CMD and CommandCode.msg
            /// </summary>
            public UInt16 command;

            public bool is_current;
            public bool autocontinue;

            public float param1;
            public float param2;
            public float param3;
            public float param4;

            public double x_lat;
            public double y_long;
            public double z_alt;
        }
    }
}