#if ROS_V2
using System;
using std_msgs;
using std_msgs.msg;

namespace mavros_msgs
{
    namespace msg
    {
        /// <summary>
        /// Known modes listed here:
        /// http://wiki.ros.org/mavros/CustomModes
        /// For system_status values
        /// see https://mavlink.io/en/messages/common.html#MAV_STATE
        /// </summary>
        [System.Serializable]
        public class State: IRosMsg, IStamped
        {
            public string GetRosType()
            {
                return "mavros_msgs.msg.State";
            }

            public string MODE_APM_PLANE_MANUAL = "MANUAL";
            public string MODE_APM_PLANE_CIRCLE = "CIRCLE";
            public string MODE_APM_PLANE_STABILIZE = "STABILIZE";
            public string MODE_APM_PLANE_TRAINING = "TRAINING";
            public string MODE_APM_PLANE_ACRO = "ACRO";
            public string MODE_APM_PLANE_FBWA = "FBWA";
            public string MODE_APM_PLANE_FBWB = "FBWB";
            public string MODE_APM_PLANE_CRUISE = "CRUISE";
            public string MODE_APM_PLANE_AUTOTUNE = "AUTOTUNE";
            public string MODE_APM_PLANE_AUTO = "AUTO";
            public string MODE_APM_PLANE_RTL = "RTL";
            public string MODE_APM_PLANE_LOITER = "LOITER";
            public string MODE_APM_PLANE_LAND = "LAND";
            public string MODE_APM_PLANE_GUIDED = "GUIDED";
            public string MODE_APM_PLANE_INITIALISING = "INITIALISING";
            public string MODE_APM_PLANE_QSTABILIZE = "QSTABILIZE";
            public string MODE_APM_PLANE_QHOVER = "QHOVER";
            public string MODE_APM_PLANE_QLOITER = "QLOITER";
            public string MODE_APM_PLANE_QLAND = "QLAND";
            public string MODE_APM_PLANE_QRTL = "QRTL";
            public string MODE_APM_COPTER_STABILIZE = "STABILIZE";
            public string MODE_APM_COPTER_ACRO = "ACRO";
            public string MODE_APM_COPTER_ALT_HOLD = "ALT_HOLD";
            public string MODE_APM_COPTER_AUTO = "AUTO";
            public string MODE_APM_COPTER_GUIDED = "GUIDED";
            public string MODE_APM_COPTER_LOITER = "LOITER";
            public string MODE_APM_COPTER_RTL = "RTL";
            public string MODE_APM_COPTER_CIRCLE = "CIRCLE";
            public string MODE_APM_COPTER_POSITION = "POSITION";
            public string MODE_APM_COPTER_LAND = "LAND";
            public string MODE_APM_COPTER_OF_LOITER = "OF_LOITER";
            public string MODE_APM_COPTER_DRIFT = "DRIFT";
            public string MODE_APM_COPTER_SPORT = "SPORT";
            public string MODE_APM_COPTER_FLIP = "FLIP";
            public string MODE_APM_COPTER_AUTOTUNE = "AUTOTUNE";
            public string MODE_APM_COPTER_POSHOLD = "POSHOLD";
            public string MODE_APM_COPTER_BRAKE = "BRAKE";
            public string MODE_APM_COPTER_THROW = "THROW";
            public string MODE_APM_COPTER_AVOID_ADSB = "AVOID_ADSB";
            public string MODE_APM_COPTER_GUIDED_NOGPS = "GUIDED_NOGPS";
            public string MODE_APM_ROVER_MANUAL = "MANUAL";
            public string MODE_APM_ROVER_LEARNING = "LEARNING";
            public string MODE_APM_ROVER_STEERING = "STEERING";
            public string MODE_APM_ROVER_HOLD = "HOLD";
            public string MODE_APM_ROVER_AUTO = "AUTO";
            public string MODE_APM_ROVER_RTL = "RTL";
            public string MODE_APM_ROVER_GUIDED = "GUIDED";
            public string MODE_APM_ROVER_INITIALISING = "INITIALISING";
            public string MODE_PX4_MANUAL = "MANUAL";
            public string MODE_PX4_ACRO = "ACRO";
            public string MODE_PX4_ALTITUDE = "ALTCTL";
            public string MODE_PX4_POSITION = "POSCTL";
            public string MODE_PX4_OFFBOARD = "OFFBOARD";
            public string MODE_PX4_STABILIZED = "STABILIZED";
            public string MODE_PX4_RATTITUDE = "RATTITUDE";
            public string MODE_PX4_MISSION = "AUTO.MISSION";
            public string MODE_PX4_LOITER = "AUTO.LOITER";
            public string MODE_PX4_RTL = "AUTO.RTL";
            public string MODE_PX4_LAND = "AUTO.LAND";
            public string MODE_PX4_RTGS = "AUTO.RTGS";
            public string MODE_PX4_READY = "AUTO.READY";
            public string MODE_PX4_TAKEOFF = "AUTO.TAKEOFF";

            public std_msgs.msg.Header header { get; set; } = new std_msgs.msg.Header();
            public bool connected;
            public bool armed;
            public bool guided;
            public bool manual_input;
            public string mode = "";
            public byte system_status;
        }
    }
}
#endif