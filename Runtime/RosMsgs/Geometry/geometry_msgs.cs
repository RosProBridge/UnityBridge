using std_msgs;
using std_msgs.msg;

namespace geometry_msgs
{
    namespace msg
    {
        public class Point : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "geometry_msgs.msg.Point";
            }

            public double x { get; set; }
            public double y { get; set; }
            public double z { get; set; }

            public static implicit operator Point(Vector3 value)
            {
                return new Point() { x = value.x, y = value.y, z = value.z };
            }

            public static implicit operator Vector3(Point value)
            {
                return new Vector3() { x = value.x, y = value.y, z = value.z };
            }
        }

        public class Vector3 : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "geometry_msgs.msg.Vector3";
            }

            public double x { get; set; }
            public double y { get; set; }
            public double z { get; set; }
        }

        public class PointStamped : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "geometry_msgs.msg.PointStamped";
            }

            public Header header { get; set; } = new Header();
            public Point point = new Point();
        }

        public class Quaternion : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "geometry_msgs.msg.Quaternion";
            }

            public double x;
            public double y;
            public double z;
            public double w;
        }

        public class Pose : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "geometry_msgs.msg.Pose";
            }

            public Point position = new Point();
            public Quaternion orientation = new Quaternion() { w = 1 };
        }

        public class PoseArray : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "geometry_msgs.msg.PoseArray";
            }

            public Header header { get; set; } = new Header();
            public Pose[] poses;
        }

        public class PoseStamped : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "geometry_msgs.msg.PoseStamped";
            }

            public Header header { get; set; } = new Header();
            public Pose pose = new Pose();
        }

        public class PoseWithCovariance : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "geometry_msgs.msg.PoseWithCovariance";
            }

            public Pose pose = new Pose();
            public double[] covariance = new double[36];
        }

        public class Twist : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "geometry_msgs.msg.Twist";
            }

            public Vector3 linear = new Vector3();
            public Vector3 angular = new Vector3();
        }

        public class TwistStamped : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "geometry_msgs.msg.TwistStamped";
            }

            public Header header { get; set; } = new Header();
            public Twist twist = new Twist();
        }

        public class TwistithCovariance : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "geometry_msgs.msg.TwistithCovariance";
            }

            public Twist twist = new Twist();
            public double[] covariance = new double[36];
        }

        public class Transform : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "geometry_msgs.msg.Transform";
            }

            public Vector3 translation = new Vector3();
            public Quaternion rotation = new Quaternion();
        }

        public class TransformStamped : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "geometry_msgs.msg.TransformStamped";
            }

            public Header header { get; set; } = new Header();
            public string child_frame_id;
            public Transform transform = new Transform();
        }
    }
}