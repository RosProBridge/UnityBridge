using System;
using UnityEngine;
using ProBridge.Utils;

namespace ProBridge.Tx.Geometry
{
    [AddComponentMenu("ProBridge/Tx/Geometry/PoseStamped")]
    public class PoseStamped : ProBridgeTxMsgStamped<ROS.Msgs.Geometry.PoseStamped>
    {
        public bool globalPose;

        protected override string GetMsgType()
        {
            return "geometry_msgs.msg.PoseStamped";
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
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

            return base.GetMsg(ts);
        }
    }
}