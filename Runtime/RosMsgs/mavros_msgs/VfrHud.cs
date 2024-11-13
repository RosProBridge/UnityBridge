using std_msgs;
using std_msgs.msg;

namespace mavros_msgs
{
    namespace msg
    {
        /// <summary>
        /// Metrics typically displayed on a HUD for fixed wing aircraft
        /// </summary>
        [System.Serializable]

#if ROS_V2
        public class VfrHud : IRosMsg, IStamped
        {
            public string GetRosType()
            {
                return "mavros_msgs.msg.VfrHud";
            }
#else
        public class VFR_HUD : IRosMsg, IStamped
        {
            public string GetRosType()
            {
                return "mavros_msgs.msg.VFR_HUD";
            }
#endif
            public std_msgs.msg.Header header { get; set; } = new std_msgs.msg.Header();

            /// <summary>
            /// m/s
            /// </summary>
            public float airspeed;

            /// <summary>
            /// m/s
            /// </summary>
            public float groundspeed;

            /// <summary>
            /// degrees 0..360
            /// </summary>
            public short heading;

            /// <summary>
            /// normalized to 0.0..1.0
            /// </summary>
            public float throttle;

            /// <summary>
            /// MSL
            /// </summary>
            public float altitude;

            /// <summary>
            /// current climb rate m/s
            /// </summary>
            public float climb;

        }
    }
}