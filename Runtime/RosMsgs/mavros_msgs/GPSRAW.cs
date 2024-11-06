using System;
using std_msgs;
using std_msgs.msg;
using UInt16 = System.UInt16;

namespace mavros_msgs
{
    namespace msg
    {
        /// <summary>
        /// FCU GPS RAW message for the gps_status plugin
        /// A merge of mavlink GPS_RAW_INT and
        /// mavlink GPS2_RAW messages.
        /// </summary>
        public class GPSRAW : IRosMsg, IStamped
        {
            public string GetRosType()
            {
                return "mavros_msgs.msg.GPSRAW";
            }

            /// <summary>
            /// No GPS connected
            /// </summary>
            public const byte GPS_FIX_TYPE_NO_GPS = 0;

            /// <summary>
            /// No position information, GPS is connected
            /// </summary>
            public const byte GPS_FIX_TYPE_NO_FIX = 1;

            /// <summary>
            /// 2D position
            /// </summary>
            public const byte GPS_FIX_TYPE_2D_FIX = 2;

            /// <summary>
            /// 3D position
            /// </summary>
            public const byte GPS_FIX_TYPE_3D_FIX = 3;

            /// <summary>
            /// DGPS/SBAS aided 3D position
            /// </summary>
            public const byte GPS_FIX_TYPE_DGPS = 4;

            /// <summary>
            /// TK float, 3D position
            /// </summary>
            public const byte GPS_FIX_TYPE_RTK_FLOATR = 5;

            /// <summary>
            /// TK float, 3D position
            /// </summary>
            public const byte GPS_FIX_TYPE_RTK_FIXEDR = 6;

            /// <summary>
            /// Static fixed, typically used for base stations
            /// </summary>
            public const byte GPS_FIX_TYPE_STATIC = 7;

            /// <summary>
            /// PPP, 3D position
            /// </summary>
            public const byte GPS_FIX_TYPE_PPP = 8;

            public std_msgs.msg.Header header { get; set; } = new std_msgs.msg.Header();

            /// <summary>
            /// [GPS_FIX_TYPE] GPS fix type
            /// </summary>
            public byte fix_type;

            /// <summary>
            /// [degE7] Latitude (WGS84, EGM96 ellipsoid)
            /// </summary>
            public int lat;

            /// <summary>
            /// [degE7] Longitude (WGS84, EGM96 ellipsoid)
            /// </summary>
            public int lon;

            /// <summary>
            /// [mm] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
            /// </summary>
            public int alt;

            /// <summary>
            /// GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
            /// </summary>
            public UInt16 eph = UInt16.MaxValue;

            /// <summary>
            /// GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
            /// </summary>
            public UInt16 epv = UInt16.MaxValue;

            /// <summary>
            /// [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX
            /// </summary>
            public UInt16 vel = UInt16.MaxValue;

            /// <summary>
            /// [cdeg] Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
            /// </summary>
            public UInt16 cog = UInt16.MaxValue;

            /// <summary>
            /// Number of satellites visible. If unknown, set to 255
            /// </summary>
            public byte satellites_visible = 255;

            /// <summary>
            /// [mm] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
            /// </summary>
            public int alt_ellipsoid;

            /// <summary>
            /// [mm] Position uncertainty. Positive for up.
            /// </summary>
            public UInt32 h_acc;

            /// <summary>
            /// [mm] Altitude uncertainty. Positive for up.
            /// </summary>
            public UInt32 v_acc;

            /// <summary>
            /// [mm] Speed uncertainty. Positive for up.
            /// </summary>
            public UInt32 vel_acc;

            /// <summary>
            /// [degE5] Heading / track uncertainty
            /// </summary>
            public int hdg_acc;

            /// <summary>
            /// Number of DGPS satellites
            /// </summary>
            public byte dgps_numch;

            /// <summary>
            /// [ms] Age of DGPS info
            /// </summary>
            public UInt32 dgps_age;

        }
    }
}