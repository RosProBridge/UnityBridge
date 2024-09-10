using std_msgs;
using std_msgs.msg;

namespace radar_msgs
{
    namespace msg
    {
        public class RadarReturn : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "radar_msgs.msg.RadarReturn";
            }

            /// <summary>
            /// Distance (m) from the sensor to the detected return.
            /// </summary>
            public float range;

            /// <summary>
            /// Angle (in radians) in the azimuth plane between the sensor and the detected return.
            /// Positive angles are anticlockwise from the sensor and negative angles clockwise from the sensor as per REP-0103.
            /// </summary>
            public float azimuth;

            /// <summary>
            /// Angle (in radians) in the elevation plane between the sensor and the detected return.
            /// Negative angles are below the sensor. For 2D radar, this will be 0.
            /// </summary>
            public float elevation;

            /// <summary>
            /// The doppler speed (m/s) of the return.
            /// </summary>
            public float doppler_velocity;

            /// <summary>
            /// The amplitude of the return (dB).
            /// </summary>
            public float amplitude;
        }

        public class RadarScan : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "radar_msgs.msg.RadarScan";
            }

            public Header header { get; set; } = new Header();
            public RadarReturn[] returns;
        }
    }
}