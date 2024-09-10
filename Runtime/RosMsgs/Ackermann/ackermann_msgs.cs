using std_msgs;

namespace ackermann_msgs
{
    namespace msg
    {
        public class AckermannDrive : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "ackermann_msgs.msg.AckermannDrive";
            }


            /// <summary>
            /// Desired virtual angle (radians)
            /// </summary>
            public float steering_angle;

            /// <summary>
            /// Desired rate of change (radians/s)
            /// </summary>
            public float steering_angle_velocity;

            /// <summary>
            /// Desired forward speed (m/s)
            /// </summary>
            public float speed;

            /// <summary>
            /// Desired acceleration (m/s^2)
            /// </summary>
            public float acceleration;

            /// <summary>
            /// Desired jerk (m/s^3)
            /// </summary>
            public float jerk;
        }
    }
}