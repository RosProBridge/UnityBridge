using UnityEngine;

using System;

using ProBridge.ROS;

namespace ProBridge
{
    public class PoseStampedCustom : ProBridgeBehaviour<PoseStamped>
    {
        public string frameId;
        public PoseFromLine p;

        protected override void OnStart()
        {
        }

        protected override void OnStop()
        {
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            var data = new ROS.Msgs.Geometry.PoseStamped();
            data.header.frame_id = frameId;
            data.header.stamp = ts;

            data.pose.position.x = p.distance_front_;
            data.pose.position.y = p.distance_back_;
            data.pose.orientation.z = p.angle_;

            return new ProBridge.Msg()
            {
                n = topic,
                t = "geometry_msgs.msg.PoseStamped",
                q = qos,
                d = data
            };
        }
    }
}