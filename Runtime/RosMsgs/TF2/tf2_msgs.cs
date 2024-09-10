using std_msgs;
using geometry_msgs.msg;

namespace tf2_msgs
{
    namespace msg
    {
        public class TFMessage : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "tf2_msgs.msg.TFMessage";
            }

            public TransformStamped[] transforms;
        }
    }
}