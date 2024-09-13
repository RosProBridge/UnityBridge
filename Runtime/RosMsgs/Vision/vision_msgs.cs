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

            /// <summary>
            /// The 3D position and orientation of the bounding box center.
            /// </summary>
            public Pose center;

            /// <summary>
            /// The size of the bounding box, in meters, surrounding the object's center pose.
            /// </summary>
            public Vector3 size;
        }

        public class ObjectHypothesis : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "vision_msgs.msg.ObjectHypothesis";
            }

            /// <summary>
            /// The unique string ID of the object class.
            /// </summary>
            public string class_id;

            /// <summary>
            /// The probability or confidence value of the detected object.
            /// </summary>
            public double score;
        }


        public class ObjectHypothesisWithPose : ObjectHypothesis
        {
            string IRosMsg.GetRosType()
            {
                return "vision_msgs.msg.ObjectHypothesisWithPose";
            }

            /// <summary>
            /// The 6D pose of the object hypothesis. This pose should be
            /// defined as the pose of some fixed reference point on the object,
            /// such as the geometric center of the bounding box or the center of mass.
            /// </summary>
            public PoseWithCovariance pose;
        }


        public class Detection3D : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "vision_msgs.msg.Detection3D";
            }

            public Header header { get; set; } = new Header();

            /// <summary>
            /// Class probabilities. Does not have to include hypotheses for all possible
            /// object ids, the scores for any ids not listed are assumed to be 0.
            /// </summary>
            public ObjectHypothesisWithPose[] results;

            /// <summary>
            /// 3D bounding box surrounding the object.
            /// </summary>
            public BoundingBox3D bbox;

            /// <summary>
            /// ID used for consistency across multiple detection messages. Detections
            /// of the same object in different detection messages should have the same id.
            /// This field may be empty.
            /// </summary>
            public string id;

            /// <summary>
            ///Source data that generated this classification are not a part of the message.
            ///If you need to access them, use an exact or approximate time synchronizer in
            ///your code, as this message's header should match the header of the source
            ///data.
            /// </summary>
            public PointCloud2 source_cloud;

        }

        public class Detection3DArray : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "vision_msgs.msg.Detection3DArray";
            }

            public Header header { get; set; } = new Header();

            /// <summary>
            /// A list of the detected proposals. A multi-proposal detector might generate
            /// this list with many candidate detections generated from a single input.
            /// </summary>
            public Detection3D[] detections;
        }
    }

}
