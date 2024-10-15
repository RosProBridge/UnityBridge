using System;
using nav_msgs.msg;
using UnityEngine;
using ProBridge.Utils;

namespace ProBridge.Tx.Nav
{
    [AddComponentMenu("ProBridge/Tx/nav_msgs/Odometry")]
    [RequireComponent(typeof(Rigidbody))]
    public class OdometryTx : ProBridgeTxStamped<Odometry>
    {
        public string childFrameId;
        public bool localOrigin = false;
        public Transform startPose;
        public float[] covariancePose = new float[6] { 0.001f, 0.001f, 0.001f, 0.001f, 0.001f, 0.001f };
        public float[] covarianceTwist = new float[6] { 0.001f, 0.001f, 0.001f, 0.001f, 0.001f, 0.001f };
        public Rigidbody Body { get; private set; }
        private Vector3 _startPose;
        private Quaternion _startRotation;
        protected override void OnStart()
        {
            Body = GetComponent<Rigidbody>();

            if (localOrigin)
            {
                _startPose = transform.position;
                _startRotation = transform.rotation;
            }
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data.child_frame_id = childFrameId;
            if (localOrigin)
            {
                data.pose.pose.position = (Quaternion.Inverse(_startRotation) * (Body.position - _startPose)).ToRos();
                data.pose.pose.orientation = (Quaternion.Inverse(_startRotation) * Body.rotation).ToRos();
            }
            else
            {
                data.pose.pose.position = startPose.InverseTransformPoint(Body.position).ToRos();
                data.pose.pose.orientation = (Quaternion.Inverse(startPose.rotation) * Body.rotation).ToRos();
            }

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
            return base.GetMsg(ts);
        }
    }
}