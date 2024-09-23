using geometry_msgs.msg;
using std_msgs;
using std_msgs.msg;

namespace nav_msgs
{
    namespace msg
    {
        public class Odometry : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "nav_msgs.msg.Odometry";
            }

            public Header header { get; set; } = new Header();
            public string child_frame_id;
            public PoseWithCovariance pose = new PoseWithCovariance();
            public TwistWithCovariance twist = new TwistWithCovariance();
        }

        public class Path : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "nav_msgs.msg.Path";
            }

            public Header header { get; set; } = new Header();
            public PoseStamped[] poses;
        }
    }
}