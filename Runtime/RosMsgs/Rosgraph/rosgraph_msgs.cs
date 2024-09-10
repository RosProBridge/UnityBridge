using System;
using std_msgs;

namespace rosgraph_msgs
{
    namespace msg
    {
        public class Clock : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "rosgraph_msgs.msg.Clock";
            }

            public Time clock = new Time();

            public Clock()
            {
            }

            public Clock(TimeSpan time)
            {
                clock = time;
            }
        }
    }
}