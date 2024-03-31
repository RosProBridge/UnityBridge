using ProBridge.ROS.Msgs.Std;
using System;

namespace ProBridge.ROS
{
    public static class QoS
    {
        public static readonly string DEFAULT = "qos_profile_system_default";
        public static readonly string BEST_EFFORT = "qos_profile_sensor_data";
    }
}

namespace ProBridge.ROS.Msgs
{
#if ROS_V2
    public class Time
    {
        public uint sec;
        public uint nanosec;

        public static implicit operator Time(TimeSpan value)
        {
            return new Time()
            {
                sec = (uint)(value.TotalSeconds - 62135596800L), // convert seconds into unix time
                nanosec = (uint)((value.Ticks - ((long)value.TotalSeconds * TimeSpan.TicksPerSecond)) * (1e9d / TimeSpan.TicksPerSecond))
            };
        }

        public static implicit operator TimeSpan(Time value)
        {
            return new TimeSpan((long)((value.sec + 62135596800L) * TimeSpan.TicksPerSecond + (value.nanosec / 1e6 * TimeSpan.TicksPerMillisecond)));
        }
    }
#else
    public class Time
    {
        public uint secs;
        public uint nsecs;

        public static implicit operator Time(TimeSpan value)
        {
            return new Time()
            {
                secs = (uint)(value.TotalSeconds - 62135596800L), // convert seconds into unix time
                nsecs = (uint)((value.Ticks - ((long)value.TotalSeconds * TimeSpan.TicksPerSecond)) * (1e9d / TimeSpan.TicksPerSecond))
            };
        }

        public static implicit operator TimeSpan(Time value)
        {
            return new TimeSpan((long)((value.secs + 62135596800L) * TimeSpan.TicksPerSecond + (value.nsecs / 1e6 * TimeSpan.TicksPerMillisecond)));
        }
    }
#endif
}


namespace ProBridge.ROS.Msgs.Std
{
    public class Time
    {
        public Msgs.Time data = new Msgs.Time();

        public Time(TimeSpan time)
        {
            data = time;
        }
    }

    public class Native<T> where T : IConvertible
    {
        public T data;
    }

    public class Header
    {
        public Msgs.Time stamp;
#if ROS_V2
#else
        public UInt32 seq;
#endif
        public string frame_id;
    }

    public interface IStamped
    {
        Header header { get; }
    }
}

namespace ProBridge.ROS.Msgs.Rosgraph
{
    public class Clock
    {
        public Time clock = new Time();

        public Clock() { }

        public Clock(TimeSpan time)
        {
            clock = time;
        }
    }
}

namespace ProBridge.ROS.Msgs.Geometry
{
    public class Point
    {
        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }

        public static implicit operator Point(Vector3 value) { return new Point() { x = value.x, y = value.y, z = value.z }; }
        public static implicit operator Vector3(Point value) { return new Vector3() { x = value.x, y = value.y, z = value.z }; }
    }

    public class Vector3
    {
        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }
    }

    public class PointStamped : IStamped
    {
        public Header header { get; set; } = new Header();
        public Point point = new Point();
    }

    public class Quaternion
    {
        public double x;
        public double y;
        public double z;
        public double w;
    }

    public class Pose
    {
        public Point position = new Point();
        public Quaternion orientation = new Quaternion() { w = 1 };
    }

    public class PoseStamped : IStamped
    {
        public Header header { get; set; } = new Header();
        public Pose pose = new Pose();
    }

    public class PoseWithCovariance
    {
        public Pose pose = new Pose();
        public float[] covariance = new float[36];
    }

    public class Twist
    {
        public Point linear = new Point();
        public Point angular = new Point();
    }

    public class TwistStamped : IStamped
    {
        public Header header { get; set; } = new Header();
        public Twist twist = new Twist();
    }

    public class TwistithCovariance
    {
        public Twist twist = new Twist();
        public float[] covariance = new float[36];
    }
}

namespace ProBridge.ROS.Msgs.Sensors
{
    public class Imu : IStamped
    {
        public Header header { get; set; } = new Header();
        public Geometry.Quaternion orientation = new Geometry.Quaternion() { w = 1 };
        public float[] orientation_covariance = new float[9];
        public Geometry.Vector3 angular_velocity = new Geometry.Vector3();
        public float[] angular_velocity_covariance = new float[9];
        public Geometry.Vector3 linear_acceleration = new Geometry.Vector3();
        public float[] linear_acceleration_covariance = new float[9];
    }

    public class NavSatFix : IStamped
    {
        public const byte COVARIANCE_TYPE_UNKNOWN = 0;
        public const byte COVARIANCE_TYPE_APPROXIMATED = 1;
        public const byte COVARIANCE_TYPE_DIAGONAL_KNOWN = 2;
        public const byte COVARIANCE_TYPE_KNOWN = 3;

        public Header header { get; set; } = new Header();

        /// <summary>
        /// Latitude [degrees]. Positive is north of equator; negative is south
        /// </summary>
        public double latitude;

        /// <summary>
        /// Longitude [degrees]. Positive is east of prime meridian; negative is west.
        /// </summary>
        public double longitude;

        /// <summary>
        /// Altitude [m]. Positive is above the WGS 84 ellipsoid 
        /// (quiet NaN if no altitude is available).
        /// </summary>
        public double altitude;

        /// <summary>
        /// // Position covariance [m^2] defined relative to a tangential plane
        /// through the reported position. The components are East, North, and
        /// Up (ENU), in row-major order.
        /// Beware: this coordinate system exhibits singularities at the poles.
        /// </summary>
        public double[] position_covariance = new double[9];

        /// <summary>
        /// If the covariance of the fix is known, fill it in completely. If the
        /// GPS receiver provides the variance of each measurement, put them
        /// along the diagonal. If only Dilution of Precision is available,
        /// estimate an approximate covariance from that.
        /// </summary>
        public byte position_covariance_type;
    }
}

namespace ProBridge.ROS.Msgs.Nav
{
    public class Odometry : IStamped
    {
        public Header header { get; set; } = new Header();
        public string child_frame_id;
        public Geometry.PoseWithCovariance pose = new Geometry.PoseWithCovariance();
        public Geometry.TwistithCovariance twist = new Geometry.TwistithCovariance();
    }
    public class Path : IStamped
    {
        public Header header { get; set; } = new Header();
        public Geometry.PoseStamped[] poses;
    }
}
