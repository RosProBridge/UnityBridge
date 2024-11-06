using System;
using std_msgs;
using std_msgs.msg;


namespace mavros_msgs
{
    namespace msg
    {
        /// <summary>
        /// Altitude information
        /// https://mavlink.io/en/messages/common.html#ALTITUDE
        /// </summary>
        [System.Serializable]
        public class Altitude: IRosMsg, IStamped
        {
            public string GetRosType()
            {
                return "mavros_msgs.msg.Altitude";
            }
            public std_msgs.msg.Header header { get; set; } = new std_msgs.msg.Header();

            public float monotonic;
            public float amsl;
            public float local;
            public float relative;
            public float terrain;
            public float bottom_clearance;
        }
    }
}
