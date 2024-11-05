using UnityEngine;
using std_msgs.msg;
using std_msgs;
using System;
using UInt16 = System.UInt16;


namespace mavros_msgs
{
    namespace msg
    {
        public class VehicleInfo : IRosMsg, IStamped
        {
            public string GetRosType()
            {
                return "mavros_msgs.msg.VehicleInfo";
            }

            public const byte HAVE_INFO_HEARTBEAT = 1;
            public const byte HAVE_INFO_AUTOPILOT_VERSION = 2;

            public std_msgs.msg.Header header { get; set; } = new std_msgs.msg.Header();

            /// <summary>
            /// Bitmap shows what info is available
            /// </summary>
            public byte available_info;

            /// <summary>
            /// SYSTEM ID
            /// </summary>
            public byte sysid;

            /// <summary>
            /// COMPONENT ID
            /// </summary>
            public byte compid;

            /// <summary>
            /// MAV_AUTOPILOT
            /// </summary>
            public byte autopilot;

            /// <summary>
            /// MAV_TYPE
            /// </summary>
            public byte type;

            /// <summary>
            /// MAV_STATE
            /// </summary>
            public byte system_status;

            public byte base_mode;
            public UInt32 custom_mode;

            /// <summary>
            /// MAV_MODE string
            /// </summary>
            public string mode = "";

            /// <summary>
            /// MAV_MODE number
            /// </summary>
            public UInt32 mode_id;

            /// <summary>
            /// MAV_PROTOCOL_CAPABILITY
            /// </summary>
            public UInt64 capabilities;

            /// <summary>
            /// Firmware version number
            /// </summary>
            public UInt32 flight_sw_version;

            /// <summary>
            /// Middleware version number
            /// </summary>
            public UInt32 middleware_sw_version;

            /// <summary>
            /// Middleware version number
            /// </summary>
            public UInt32 os_sw_version;

            /// <summary>
            /// HW / board version (last 8 bytes should be silicon ID, if any)
            /// </summary>
            public UInt32 board_version;

            /// <summary>
            /// Custom version field, commonly from the first 8 bytes of the git hash
            /// </summary>
            public string flight_custom_version = "";

            /// <summary>
            /// ID of the board vendor
            /// </summary>
            public UInt16 vendor_id;

            /// <summary>
            /// ID of the product
            /// </summary>
            public UInt16 product_id;

            /// <summary>
            /// UID if provided by hardware
            /// </summary>
            public UInt64 uid;
        }

    }
}
