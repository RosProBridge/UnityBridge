using UnityEngine;

using System;

using ProBridge.ROS;

namespace ProBridge
{
    public class PoseStamped : ProBridgeBehaviour<PoseStamped>
    {
        public string frameId;
        public bool globalPose;

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

            if (globalPose)
            {
                data.pose.position = transform.position.ToRos();
                data.pose.orientation = transform.rotation.ToRos();
            }
            else
            {
                data.pose.position = transform.localPosition.ToRos();
                data.pose.orientation = transform.localRotation.ToRos();
            }

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