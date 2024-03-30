using System;

namespace ProBridge.ROS.Msgs.Ackermann
{
    public class AckermannDrive : Std.IStamped
    {
        public Std.Header header { get; } = new Std.Header();

        public float steering_angle;                  // desired virtual angle (radians)
        public float steering_angle_velocity;         // desired rate of change (radians/s)
        public float speed;                           // desired forward speed (m/s)
        public sbyte acceleration;                    // desired acceleration (m/s^2)
        public sbyte jerk;                            // desired jerk (m/s^3)
    }

}