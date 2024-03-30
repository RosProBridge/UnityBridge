using System;

namespace ProBridge
{
    public class SimTime : ProBridgeBehaviour<SimTime>
    {
        protected override void OnStart()
        {
        }

        protected override void OnStop()
        {
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            return new ProBridge.Msg()
            {
                n = topic,
                t = "rosgraph_msgs.msg.Clock",
                q = qos,
                d = new ROS.Msgs.Rosgraph.Clock(ts)
            };
        }
    }
}
