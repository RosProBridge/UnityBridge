using System;
using ProBridge.ROS.Msgs.Std;

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
    public interface IRosMsg
    {
        public string GetRosType();
    }

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
    public abstract class StdMsg<T> : IRosMsg
    {
        public T data;

        public abstract string GetRosType();
    }

    public class StdTime : StdMsg<Time>
    {
        public override string GetRosType() { return "std_msgs.msg.Time"; }

        public StdTime(TimeSpan time)
        {
            data = time;
        }
    }

    public class StdBool : StdMsg<bool>
    {
        public override string GetRosType() { return "std_msgs.msg.Bool"; }
    }

    public class StdFloat : StdMsg<float>
    {
        public override string GetRosType() { return "std_msgs.msg.Float32"; }
    }

    public class StdInt : StdMsg<int>
    {
        public override string GetRosType() { return "std_msgs.msg.Int32"; }
    }

    public class StdString : StdMsg<string>
    {
        public override string GetRosType() { return "std_msgs.msg.String"; }
    }

    public class ColorRGBA : IRosMsg
    {
        string IRosMsg.GetRosType() { return "std_msgs.msg.ColorRGBA"; }

        public float r;
        public float g;
        public float b;
        public float a;
    }

    public class Header : IRosMsg
    {
        string IRosMsg.GetRosType() { return "std_msgs.msg.Header"; }

        public Time stamp = new Time();
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

namespace ProBridge.ROS.Msgs.Tf
{
    public class tfMessage : IRosMsg
    {
        string IRosMsg.GetRosType() { return "tf.msg.tfMessage"; }

        public Geometry.TransformStamped[] transforms;
    }
}

namespace ProBridge.ROS.Msgs.Rosgraph
{
    public class Clock : IRosMsg
    {
        string IRosMsg.GetRosType() { return "rosgraph_msgs.msg.Clock"; }

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
    public class Point : IRosMsg
    {
        string IRosMsg.GetRosType() { return "geometry_msgs.msg.Point"; }

        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }

        public static implicit operator Point(Vector3 value) { return new Point() { x = value.x, y = value.y, z = value.z }; }
        public static implicit operator Vector3(Point value) { return new Vector3() { x = value.x, y = value.y, z = value.z }; }
    }

    public class Vector3 : IRosMsg
    {
        string IRosMsg.GetRosType() { return "geometry_msgs.msg.Vector3"; }

        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }
    }

    public class PointStamped : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "geometry_msgs.msg.PointStamped"; }

        public Header header { get; set; } = new Header();
        public Point point = new Point();
    }

    public class Quaternion : IRosMsg
    {
        string IRosMsg.GetRosType() { return "geometry_msgs.msg.Quaternion"; }

        public double x;
        public double y;
        public double z;
        public double w;
    }

    public class Pose : IRosMsg
    {
        string IRosMsg.GetRosType() { return "geometry_msgs.msg.Pose"; }

        public Point position = new Point();
        public Quaternion orientation = new Quaternion() { w = 1 };
    }

    public class PoseStamped : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "geometry_msgs.msg.PoseStamped"; }

        public Header header { get; set; } = new Header();
        public Pose pose = new Pose();
    }

    public class PoseWithCovariance : IRosMsg
    {
        string IRosMsg.GetRosType() { return "geometry_msgs.msg.PoseWithCovariance"; }

        public Pose pose = new Pose();
        public float[] covariance = new float[36];
    }

    public class Twist : IRosMsg
    {
        string IRosMsg.GetRosType() { return "geometry_msgs.msg.Twist"; }

        public Point linear = new Point();
        public Point angular = new Point();
    }

    public class TwistStamped : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "geometry_msgs.msg.TwistStamped"; }

        public Header header { get; set; } = new Header();
        public Twist twist = new Twist();
    }

    public class TwistithCovariance : IRosMsg
    {
        string IRosMsg.GetRosType() { return "geometry_msgs.msg.TwistithCovariance"; }

        public Twist twist = new Twist();
        public float[] covariance = new float[36];
    }

    public class Transform : IRosMsg
    {
        string IRosMsg.GetRosType() { return "geometry_msgs.msg.Transform"; }
        public Vector3 translation = new Vector3();
        public Quaternion rotation = new Quaternion();
    }

    public class TransformStamped : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "geometry_msgs.msg.TransformStamped"; }

        public Header header { get; set; } = new Header();
        public string child_frame_id;
        public Transform transform = new Transform();
    }
}

namespace ProBridge.ROS.Msgs.Sensors
{
    public class Imu : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "sensor_msgs.msg.Imu"; }

        public Header header { get; set; } = new Header();
        public Geometry.Quaternion orientation = new Geometry.Quaternion() { w = 1 };
        public float[] orientation_covariance = new float[9];
        public Geometry.Vector3 angular_velocity = new Geometry.Vector3();
        public float[] angular_velocity_covariance = new float[9];
        public Geometry.Vector3 linear_acceleration = new Geometry.Vector3();
        public float[] linear_acceleration_covariance = new float[9];
    }

    public class NavSatFix : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "sensor_msgs.msg.NavSatFix"; }

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
    public class Odometry : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "nav_msgs.msg.Odometry"; }

        public Header header { get; set; } = new Header();
        public string child_frame_id;
        public Geometry.PoseWithCovariance pose = new Geometry.PoseWithCovariance();
        public Geometry.TwistithCovariance twist = new Geometry.TwistithCovariance();
    }
    public class Path : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "nav_msgs.msg.Path"; }

        public Header header { get; set; } = new Header();
        public Geometry.PoseStamped[] poses;
    }
}

namespace ProBridge.ROS.Msgs.Visualization
{
    public class Marker : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "visualization_msgs.msg.Marker"; }

        public Header header { get; set; } = new Header();
        public string ns;
        public Int32 id;
        public Int32 type;
        public Int32 action;
        public Geometry.Pose pose;
        public Geometry.Vector3 scale;
        public ColorRGBA color;
        public Time lifetime;
        public bool frame_locked;
        public Geometry.Point[] points;
        public ColorRGBA[] colors;
        public string text;
        public string mesh_resource;
        public bool mesh_use_embedded_materials;
    }

    public class MarkerArray : IRosMsg
    {
        string IRosMsg.GetRosType() { return "visualization_msgs.msg.MarkerArray"; }

        Marker[] markers;
    }
}

namespace ProBridge.ROS.Msgs.Chassis
{
    [Serializable]
    public class ChassisStatus : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "chassis_msgs.msg.ChassisStatus"; }

        public Header header { get; set; } = new Header();

        public float battery;                           // Напряжение АКБ Вольт
        public float fuel_available;                    // Запас топлива л
        public float fuel_consumption;                  // Текущий (усредненный за мин) расход топлива л/ч
        public sbyte engine_state;                      // Статус ДВС
        public sbyte engine_temp;                       // Температура ДВС градус
        public UInt16 engine_value;                     // Обороты ДВС об/мин
        public sbyte transmission_state;                // Статус АКПП
        public sbyte transmission_value;                // Значение передачи АКПП
        public sbyte transmission_temp;                 // Температура АКПП градус
        public sbyte transfer_value;                    // Значение раздаточной коробки
        public sbyte main_brake_state;                  // Статус тормозной системы
        public sbyte parking_brake_state;               // Статус парковочного тормоза
        public sbyte rail_state;                        // Статус рулевой рейки
        public sbyte[] parts_temp = new sbyte[0];       // Температура составных частей РТС градус
        public sbyte[] general_state = new sbyte[0];    // Статусы датчиков, приборов освещения, индикаторы, и т.д.
        public bool sto;                                // Разрешение движения
    }

    [Serializable]
    public class ChassisFeed : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "chassis_msgs.msg.ChassisFeed"; }

        public Header header { get; set; } = new Std.Header();
        public float speed;                         // Показания датчика скорости [м/сек]
        public Int16[] engine_value = new Int16[0]; // Обороты двигателя [об/мин]
        public float[] rail_value = new float[0];   // Угол поворота рулевой рейки [радиан]
        public float[] rail_speed = new float[0];   // Уголовая скорость рулевой рейки в [рад/сек]
        public float[] rail_target = new float[0];  // Задание на рулевую рейку [усл. ед]
        public float[] accel_value = new float[0];  // Положение педали газа [усл. ед]
        public float[] accel_target = new float[0]; // Задание на педаль газа [усл. ед]
        public float[] brake_value = new float[0];  // Значение датчика обратной связи по тормозной системе (отрицательное значение - код неисправности) [усл. ед]
        public float[] brake_target = new float[0]; // Задание для тормозной системы [усл. ед]
    }

    [Serializable]
    public class ChassisSignals : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "chassis_msgs.msg.ChassisSignals"; }

        public Header header { get; set; } = new Std.Header();
        public bool lights_side;            // состояние габаритных огней
        public bool lights_head;            // состояние фонарей головного света
        public bool lights_left_turn;       // состояние левого указателя поворота
        public bool lights_right_turn;      // состояние правого указателя поворота
        public bool sound_signal;           // состояние звукового сигнала
        public byte[] aux = new byte[0];     // состояние дополнительного сигнального оборудования
    }

    [Serializable]
    public class ChassisControl : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "chassis_msgs.msg.ChassisControl"; }

        public Header header { get; set; } = new Header();

        public float throttle;          //  0. <= throttle <= 1.
        public float steer;             //  -1. <= steer <= 1.
        public float brake;             //  0. <= brake <= 1
        public bool hand_brake;         //  0. <= throttle <= 1.
        public bool reverse;            //  reverse 0 or 1
        public Int32 gear;              //  gear
        public bool manual_gear_shift;  //  manual_gear_shift
    }
}

namespace ProBridge.ROS.Msgs.Ackermann
{
    [Serializable]
    public class AckermannDrive : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "ackermann_msgs.msg.AckermannDrive"; }

        public Header header { get; set; } = new Header();

        public float steering_angle;                  // desired virtual angle (radians)
        public float steering_angle_velocity;         // desired rate of change (radians/s)
        public float speed;                           // desired forward speed (m/s)
        public sbyte acceleration;                    // desired acceleration (m/s^2)
        public sbyte jerk;                            // desired jerk (m/s^3)
    }
}