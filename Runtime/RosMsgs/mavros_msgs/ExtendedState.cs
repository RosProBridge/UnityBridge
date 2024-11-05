using std_msgs;
using std_msgs.msg;

namespace mavros_msgs
{
    namespace msg
    {
        /// <summary>
        /// Extended autopilot state
        /// https://mavlink.io/en/messages/common.html#EXTENDED_SYS_STATE
        /// </summary>
        public class ExtendedState: IRosMsg, IStamped
        {
            public string GetRosType()
            {
                return "mavros_msgs.msg.ExtendedState";
            }

            public const byte VTOL_STATE_UNDEFINED = 0;
            public const byte VTOL_STATE_TRANSITION_TO_FW = 1;
            public const byte VTOL_STATE_TRANSITION_TO_MC = 2;
            public const byte VTOL_STATE_MC = 3;
            public const byte VTOL_STATE_FW = 4;
            public const byte LANDED_STATE_UNDEFINED = 0;
            public const byte LANDED_STATE_ON_GROUND = 1;
            public const byte LANDED_STATE_IN_AIR = 2;
            public const byte LANDED_STATE_TAKEOFF = 3;
            public const byte LANDED_STATE_LANDING = 4;

            public std_msgs.msg.Header header { get; set; } = new std_msgs.msg.Header();
            public byte vtol_state;
            public byte landed_state;

        }
    }
}