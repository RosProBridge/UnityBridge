using std_msgs;
using std_msgs.msg;
using geometry_msgs.msg;
using sensor_msgs.msg;

namespace vision_msgs
{
    namespace msg
    {
        public class BoundingBox3D : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "vision_msgs.msg.BoundingBox3D";
            }

            public Pose center;
            public Vector3 size;
        }

        public class ObjectHypothesisWithPose : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "vision_msgs.msg.ObjectHypothesisWithPose";
            }

            public long id;
            public double score;
            public PoseWithCovariance pose;

        }

        public class Detection3D : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "vision_msgs.msg.Detection3D";
            }

            public Header header { get; set; } = new Header();

            public ObjectHypothesisWithPose[] results;
            public BoundingBox3D bbox;
            public PointCloud2 source_cloud;

        }

        public class Detection3DArray : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "vision_msgs.msg.Detection3DArray";
            }

            public Header header { get; set; } = new Header();

            public Detection3D[] detections;
        }
    }

}