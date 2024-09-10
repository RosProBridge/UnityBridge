using System;
using ProBridge.ROS.Msgs.Std;

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
    public class Duration
    {
        public uint sec;
        public uint nanosec;

        public static implicit operator Duration(TimeSpan value)
        {
            return new Duration()
            {
                sec = (uint)(value.TotalSeconds - 62135596800L), // convert seconds into unix time
                nanosec = (uint)((value.Ticks - ((long)value.TotalSeconds * TimeSpan.TicksPerSecond)) * (1e9d / TimeSpan.TicksPerSecond))
            };
        }

        public static implicit operator TimeSpan(Duration value)
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
      public class Duration
    {
        public uint secs;
        public uint nsecs;

        public static implicit operator Duration(TimeSpan value)
        {
            return new Duration()
            {
                secs = (uint)(value.TotalSeconds - 62135596800L), // convert seconds into unix time
                nsecs = (uint)((value.Ticks - ((long)value.TotalSeconds * TimeSpan.TicksPerSecond)) * (1e9d / TimeSpan.TicksPerSecond))
            };
        }

        public static implicit operator TimeSpan(Duration value)
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
    public class StdDuration : StdMsg<Duration>
    {
        public override string GetRosType() { return "std_msgs.msg.Duration"; }

        public StdDuration(TimeSpan duration)
        {
            data = duration;
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
    public class StdFloat64 : StdMsg<double>
    {
        public override string GetRosType() { return "std_msgs.msg.Float64"; }
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

#if ROS_V2
#else
        public UInt32 seq;
#endif
        public Time stamp = new Time();
        public string frame_id;
    }

    public interface IStamped
    {
        Header header { get; }
    }
    public class MultiArrayDimension : IRosMsg
    {
        string IRosMsg.GetRosType() { return "std_msgs.msg.MultiArrayDimension"; }

        /// <summary>
        /// label of given dimension
        /// </summary>
        public string label;

        /// <summary>
        /// size of given dimension (in type units)
        /// </summary>
        public UInt32 size;

        /// <summary>
        /// stride of given dimension
        /// </summary>
        public UInt32 stride;

    }
    public class MultiArrayLayout : IRosMsg
    {
        string IRosMsg.GetRosType() { return "std_msgs.msg.MultiArrayLayout"; }

        /// <summary>
        /// Array of dimension properties
        /// </summary>
        public MultiArrayDimension[] dim;

        /// <summary>
        /// padding elements at front of data
        /// </summary>
        public UInt32 data_offset;

    }

    public class StdFloat64MultiArray : IRosMsg
    {

        string IRosMsg.GetRosType() { return "std_msgs.msg.Float64MultiArray"; }

        /// <summary>
        /// specification of data layout
        /// </summary>
        public MultiArrayLayout layout;

        /// <summary>
        /// array of data
        /// </summary>
        public double[] data;

    }

}

namespace ProBridge.ROS.Msgs.TF2
{
    public class TFMessage : IRosMsg
    {
        string IRosMsg.GetRosType() { return "tf2_msgs.msg.TFMessage"; }

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
    public class PoseArray : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "geometry_msgs.msg.PoseArray"; }

        public Header header { get; set; } = new Header();
        public Pose[] poses;
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
        public double[] covariance = new double[36];
    }

    public class Twist : IRosMsg
    {
        string IRosMsg.GetRosType() { return "geometry_msgs.msg.Twist"; }

        public Vector3 linear = new Vector3();
        public Vector3 angular = new Vector3();
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
        public double[] covariance = new double[36];
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
    public class PointField : IRosMsg
    {
        string IRosMsg.GetRosType() { return "sensor_msgs.msg.PointField"; }

        // Constants for data types
        public const byte INT8 = 1;
        public const byte UINT8 = 2;
        public const byte INT16 = 3;
        public const byte UINT16 = 4;
        public const byte INT32 = 5;
        public const byte UINT32 = 6;
        public const byte FLOAT32 = 7;
        public const byte FLOAT64 = 8;

        public string name;        // Name of the field
        public uint offset;        // Offset from the start of the point struct
        public byte datatype;      // Datatype of the field
        public uint count;         // Number of elements in the field
        
    }
    public class PointCloud2 : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "sensor_msgs.msg.PointCloud2"; }

        public Header header { get; set; } = new Header();

        public uint height;               // Height of the point cloud data
        public uint width;                // Width of the point cloud data

        public PointField[] fields; // Array of PointField messages that describe the layout of the data
        
        public bool is_bigendian;         // Is this data bigendian?
        public uint point_step;           // Length of a point in bytes
        public uint row_step;             // Length of a row in bytes

        public byte[] data; // Actual point cloud data, serialized as a byte array

        public bool is_dense;             // Is this point cloud data dense?
        
        
    }
    public class Imu : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "sensor_msgs.msg.Imu"; }

        public Header header { get; set; } = new Header();
        public Geometry.Quaternion orientation = new Geometry.Quaternion() { w = 1 };
        public double[] orientation_covariance = new double[9];
        public Geometry.Vector3 angular_velocity = new Geometry.Vector3();
        public double[] angular_velocity_covariance = new double[9];
        public Geometry.Vector3 linear_acceleration = new Geometry.Vector3();
        public double[] linear_acceleration_covariance = new double[9];
    }
    
    public class NavSatStatus : IRosMsg
    {
        // ROS Message Type
        string IRosMsg.GetRosType() { return "sensor_msgs.msg.NavSatStatus"; }

        // Constants for Status
        public const sbyte STATUS_NO_FIX = -1;
        public const sbyte STATUS_FIX = 0;
        public const sbyte STATUS_SBAS_FIX = 1;
        public const sbyte STATUS_GBAS_FIX = 2;

        // Constants for Service
        public const ushort SERVICE_GPS = 1;
        public const ushort SERVICE_GLONASS = 2;
        public const ushort SERVICE_COMPASS = 4;
        public const ushort SERVICE_GALILEO = 8;

        /// <summary>
        /// Status of the GPS fix.
        /// </summary>
        public sbyte status;

        /// <summary>
        /// Which Global Navigation Satellite System (GNSS) is being used.
        /// </summary>
        public ushort service;

        public NavSatStatus()
        {
            status = STATUS_NO_FIX;
            service = SERVICE_GPS;
        }
    }

    public class NavSatFix : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "sensor_msgs.msg.NavSatFix"; }

        public const byte COVARIANCE_TYPE_UNKNOWN = 0;
        public const byte COVARIANCE_TYPE_APPROXIMATED = 1;
        public const byte COVARIANCE_TYPE_DIAGONAL_KNOWN = 2;
        public const byte COVARIANCE_TYPE_KNOWN = 3;

        public Header header { get; set; } = new Header();
        
        public NavSatStatus status = new NavSatStatus();
        
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

    public class CompressedImage : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "sensor_msgs.msg.CompressedImage"; }
        public Header header { get; set; } = new Header();

        /// <summary>
        /// Specifies the format of the data
        /// Acceptable values:
        /// jpeg, png
        /// </summary>
        public string format;

        /// <summary>
        /// Compressed image buffer
        /// </summary>
        public byte[] data;
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

    public class UVCoordinate : IRosMsg
    {
        string IRosMsg.GetRosType() { return "visualization_msgs.msg.UVCoordinate"; }

        /// <summary>
        /// Location of the pixel as a ratio of the width of a 2D texture.
        /// Values should be in range: [0.0-1.0].
        /// </summary>
        public float u;

        /// <summary>
        /// Location of the pixel as a ratio of the width of a 2D texture.
        /// Values should be in range: [0.0-1.0].
        /// </summary>
        public float v;

    }
    public class MeshFile : IRosMsg
    {
        string IRosMsg.GetRosType() { return "visualization_msgs.msg.MeshFile"; }

        /// <summary>
        /// The filename is used for both debug purposes and to provide a file extension
        /// for whatever parser is used.
        /// </summary>
        public string filename;

        /// <summary>
        /// This stores the raw text of the mesh file.
        /// </summary>
        public byte[] data;
    }
#if ROS_V2
    public class Marker : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "visualization_msgs.msg.Marker"; }

        public const Int32 ARROW = 0;
        public const Int32 CUBE = 1;
        public const Int32 SPHERE = 2;
        public const Int32 CYLINDER = 3;
        public const Int32 LINE_STRIP = 4;
        public const Int32 LINE_LIST = 5;
        public const Int32 CUBE_LIST = 6;
        public const Int32 SPHERE_LIST = 7;
        public const Int32 POINTS = 8;
        public const Int32 TEXT_VIEW_FACING = 9;
        public const Int32 MESH_RESOURCE = 10;
        public const Int32 TRIANGLE_LIST = 11;
        public const Int32 ADD = 0;
        public const Int32 MODIFY = 0;
        public const Int32 DELETE = 2;
        public const Int32 DELETEALL = 3;

        /// <summary>
        ///  Header for timestamp and frame id.
        /// </summary>
        public Header header { get; set; } = new Header();

        /// <summary>
        /// Namespace in which to place the object.
        /// Used in conjunction with id to create a unique name for the object.
        /// </summary>
        public string ns;

        /// <summary>
        /// Object ID used in conjunction with the namespace for manipulating and deleting the object later.
        /// </summary>
        public Int32 id;

        /// <summary>
        /// Type of object.
        /// </summary>
        public Int32 type;

        /// <summary>
        /// Action to take; one of:
        /// - 0 add/modify an object
        /// - 1 (deprecated)
        /// - 2 deletes an object (with the given ns and id)
        /// - 3 deletes all objects (or those with the given ns if any)
        /// </summary>
        public Int32 action;

        /// <summary>
        /// Pose of the object with respect the frame_id specified in the header.
        /// </summary>
        public Geometry.Pose pose;

        /// <summary>
        /// Scale of the object; 1,1,1 means default (usually 1 meter square).
        /// </summary>
        public Geometry.Vector3 scale;

        /// <summary>
        /// Color of the object; in the range: [0.0-1.0]
        /// </summary>
        public ColorRGBA color;

        /// <summary>
        /// How long the object should last before being automatically deleted.
        /// 0 indicates forever.
        /// </summary>
        public Duration lifetime;

        /// <summary>
        /// If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
        /// </summary>
        public bool frame_locked;

        /// <summary>
        /// Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ARROW_STRIP, etc.)
        /// </summary>
        public Geometry.Point[] points;

        /// <summary>
        /// Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
        /// The number of colors provided must either be 0 or equal to the number of points provided.
        /// NOTE: alpha is not yet used
        /// </summary>
        public ColorRGBA[] colors;

        /// <summary>
        /// Texture resource is a special URI that can either reference a texture file in
        /// a format acceptable to (resource retriever)[https://docs.ros.org/en/rolling/p/resource_retriever/]
        /// or an embedded texture via a string matching the format:
        /// "embedded://texture_name"
        /// </summary>
        public string texture_resource;

        /// <summary>
        /// An image to be loaded into the rendering engine as the texture for this marker.
        /// This will be used iff texture_resource is set to embedded.
        /// </summary>
        public Sensors.CompressedImage texture;

        /// <summary>
        /// Location of each vertex within the texture; in the range: [0.0-1.0]
        /// </summary>
        public UVCoordinate[] uv_coordinates;

        /// <summary>
        /// Only used for text markers
        /// </summary>
        public string text;

        /// <summary>
        /// Only used for MESH_RESOURCE markers.
        /// Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
        /// Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
        /// use the following format for mesh_resource:
        ///   "embedded://mesh_name"
        /// </summary>
        public string mesh_resource;
        public MeshFile mesh_file;
        public bool mesh_use_embedded_materials;
    }
#else
    public class Marker : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "visualization_msgs.msg.Marker"; }
     
        public const byte ARROW = 0;
        public const byte CUBE = 1;
        public const byte SPHERE = 2;
        public const byte CYLINDER = 3;
        public const byte LINE_STRIP = 4;
        public const byte LINE_LIST = 5;
        public const byte CUBE_LIST = 6;
        public const byte SPHERE_LIST = 7;
        public const byte POINTS = 8;
        public const byte TEXT_VIEW_FACING = 9;
        public const byte MESH_RESOURCE = 10;
        public const byte TRIANGLE_LIST = 11;
        public const byte ADD = 0;
        public const byte MODIFY = 0;
        public const byte DELETE = 2;
        public const byte DELETEALL = 3;

        /// <summary>
        ///  Header for timestamp and frame id.
        /// </summary>
        public Header header { get; set; } = new Header();

        /// <summary>
        /// Namespace in which to place the object.
        /// Used in conjunction with id to create a unique name for the object.
        /// </summary>
        public string ns;

        /// <summary>
        /// Object ID used in conjunction with the namespace for manipulating and deleting the object later.
        /// </summary>
        public Int32 id;

        /// <summary>
        /// Type of object.
        /// </summary>
        public Int32 type;

        /// <summary>
        /// Action to take; one of:
        /// - 0 add/modify an object
        /// - 1 (deprecated)
        /// - 2 deletes an object (with the given ns and id)
        /// - 3 deletes all objects (or those with the given ns if any)
        /// </summary>
        public Int32 action;

        /// <summary>
        /// Pose of the object with respect the frame_id specified in the header.
        /// </summary>
        public Geometry.Pose pose;

        /// <summary>
        /// Scale of the object; 1,1,1 means default (usually 1 meter square).
        /// </summary>
        public Geometry.Vector3 scale;

        /// <summary>
        /// Color of the object; in the range: [0.0-1.0]
        /// </summary>
        public ColorRGBA color;

        /// <summary>
        /// How long the object should last before being automatically deleted.
        /// 0 indicates forever.
        /// </summary>
        public Duration lifetime;

        /// <summary>
        /// If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
        /// </summary>
        public bool frame_locked;

        /// <summary>
        /// Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ARROW_STRIP, etc.)
        /// </summary>
        public Geometry.Point[] points;

        /// <summary>
        /// Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
        /// The number of colors provided must either be 0 or equal to the number of points provided.
        /// NOTE: alpha is not yet used
        /// </summary>
        public ColorRGBA[] colors;


        /// <summary>
        /// Only used for text markers
        /// </summary>
        public string text;

        /// <summary>
        /// only used for MESH_RESOURCE markers
        /// </summary>
        public string mesh_resource;
        public bool mesh_use_embedded_materials;
    }
#endif
    public class MarkerArray : IRosMsg
    {
        string IRosMsg.GetRosType() { return "visualization_msgs.msg.MarkerArray"; }

        public Marker[] markers;
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
    public class AckermannDrive : IRosMsg
    {
        string IRosMsg.GetRosType() { return "ackermann_msgs.msg.AckermannDrive"; }


        public float steering_angle;                  // desired virtual angle (radians)
        public float steering_angle_velocity;         // desired rate of change (radians/s)
        public float speed;                           // desired forward speed (m/s)
        public float acceleration;                    // desired acceleration (m/s^2)
        public float jerk;                            // desired jerk (m/s^3)
    }
}
namespace ProBridge.ROS.Msgs.Radar
{
    public class RadarReturn : IRosMsg
    {
        string IRosMsg.GetRosType() { return "radar_msgs.msg.RadarReturn"; }
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
        string IRosMsg.GetRosType() { return "radar_msgs.msg.RadarScan"; }

        public Header header { get; set; } = new Std.Header();
        public RadarReturn[] returns;
    }

}