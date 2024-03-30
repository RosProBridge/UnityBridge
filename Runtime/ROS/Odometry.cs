using UnityEngine;

using System;

using ProBridge.ROS;

namespace ProBridge
{
    [RequireComponent(typeof(Rigidbody))]
    public class Odometry : ProBridgeBehaviour<Odometry>
    {
        public string frameId;
        public string childFrameId;
        public bool originInStartPos;

        public float[] covariancePose = new float[6] { 0.001f, 0.001f, 0.001f, 0.001f, 0.001f, 0.001f };
        public float[] covarianceTwist = new float[6] { 0.001f, 0.001f, 0.001f, 0.001f, 0.001f, 0.001f };

        public Rigidbody Body { get; private set; }
        private Vector3 _startPos;

        protected override void OnStart()
        {
            Body = GetComponent<Rigidbody>();
            _startPos = originInStartPos ? transform.position : Vector3.zero;
        }

        protected override void OnStop()
        {
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            var data = new ROS.Msgs.Nav.Odometry()
            {
                header = new ROS.Msgs.Std.Header()
                {
                    frame_id = frameId,
                    stamp = ts
                },
                child_frame_id = childFrameId
            };

            var p = Body.position - _startPos;
            //data.pose.pose.position = new Vector3(-p.z, p.y, p.x).ToRos();
            data.pose.pose.position = p.ToRos();
            data.pose.pose.orientation = Body.rotation.ToRos();
            data.pose.covariance[0] = covariancePose[0];
            data.pose.covariance[7] = covariancePose[1];
            data.pose.covariance[14] = covariancePose[2];
            data.pose.covariance[21] = covariancePose[3];
            data.pose.covariance[28] = covariancePose[4];
            data.pose.covariance[35] = covariancePose[5];
            data.twist.twist.linear = (Quaternion.Inverse(Body.rotation) * Body.velocity).ToRos();
            data.twist.twist.angular = (Quaternion.Inverse(Body.rotation) * Body.angularVelocity).ToRosAngular();
            data.twist.covariance[0] = covarianceTwist[0];
            data.twist.covariance[7] = covarianceTwist[1];
            data.twist.covariance[14] = covarianceTwist[2];
            data.twist.covariance[21] = covarianceTwist[3];
            data.twist.covariance[28] = covarianceTwist[4];
            data.twist.covariance[35] = covarianceTwist[5];

            return new ProBridge.Msg()
            {
                n = topic,
                t = "nav_msgs.msg.Odometry",
                q = qos,
                d = data
            };
        }
    }
}