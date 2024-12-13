using System;
using nav_msgs.msg;
using UnityEngine;
using ProBridge.Utils;

namespace ProBridge.Tx.Nav
{
    [AddComponentMenu("ProBridge/Tx/nav_msgs/Odometry")]
    public class OdometryTx : ProBridgeTxStamped<Odometry>
    {
        public string childFrameId;
        public bool localOrigin = false;
        public Transform startPos;
        public float[] covariancePose = new float[6] { 0.001f, 0.001f, 0.001f, 0.001f, 0.001f, 0.001f };
        public float[] covarianceTwist = new float[6] { 0.001f, 0.001f, 0.001f, 0.001f, 0.001f, 0.001f };
        
        [Header("Noise Parameters")]
        public bool applyNoise = false;
        public float linearVelocityNoiseStdDev = 0.0f;
        public float angularVelocityNoiseStdDev = 0.0f;
        public float positionNoiseStdDev = 0.0f;
        public float rotationNoiseStdDev = 0.0f;
        
        private Vector3 _startPose;
        private Quaternion _startRotation;

        private Vector3 _previousPosition;
        private Quaternion _previousRotation;

        public Vector3 Velocity = new Vector3(0, 0, 0);
        public Vector3 AngularVelocity = new Vector3(0, 0, 0);

        protected override void OnStart()
        {
            _previousRotation = transform.rotation;
            _previousPosition = transform.position;
            if (localOrigin)
            {
                _startPose = transform.position;
                _startRotation = transform.rotation;
            }
        }

        public void FixedUpdate()
        {
            Velocity = (transform.position - _previousPosition) / Time.fixedDeltaTime;
            _previousPosition = transform.position;

            Quaternion deltaRotation = transform.rotation * Quaternion.Inverse(_previousRotation);

            deltaRotation.ToAngleAxis(out var angle, out var axis);

            AngularVelocity = Mathf.Deg2Rad * angle * axis / Time.fixedDeltaTime;

            _previousRotation = transform.rotation;
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            if (localOrigin)
            {
                data.pose.pose.position =
                    (Quaternion.Inverse(_startRotation) * (transform.position - _startPose)).ToRos();
                data.pose.pose.orientation = (Quaternion.Inverse(_startRotation) * transform.rotation).ToRos();
            }
            else
            {
                data.pose.pose.position = startPos.InverseTransformPoint(transform.position).ToRos();
                data.pose.pose.orientation = (Quaternion.Inverse(startPos.rotation) * transform.rotation).ToRos();
            }


            data.twist.twist.linear = (Quaternion.Inverse(transform.rotation) * Velocity).ToRos();
            data.twist.twist.angular = (Quaternion.Inverse(transform.rotation) * AngularVelocity).ToRosAngular();

            if (applyNoise)
            {
                data.pose.pose.position.x += GaussianNoise.Generate(positionNoiseStdDev);
                data.pose.pose.position.y += GaussianNoise.Generate(positionNoiseStdDev);
                data.pose.pose.position.z += GaussianNoise.Generate(positionNoiseStdDev);
                
                data.pose.pose.orientation.x += GaussianNoise.Generate(rotationNoiseStdDev);
                data.pose.pose.orientation.y += GaussianNoise.Generate(rotationNoiseStdDev);
                data.pose.pose.orientation.z += GaussianNoise.Generate(rotationNoiseStdDev);
                data.pose.pose.orientation.w += GaussianNoise.Generate(rotationNoiseStdDev);
                
                data.twist.twist.linear.x += GaussianNoise.Generate(linearVelocityNoiseStdDev);
                data.twist.twist.linear.y += GaussianNoise.Generate(linearVelocityNoiseStdDev);
                data.twist.twist.linear.z += GaussianNoise.Generate(linearVelocityNoiseStdDev);
                
                data.twist.twist.angular.x += GaussianNoise.Generate(angularVelocityNoiseStdDev);
                data.twist.twist.angular.y += GaussianNoise.Generate(angularVelocityNoiseStdDev);
                data.twist.twist.angular.z += GaussianNoise.Generate(angularVelocityNoiseStdDev);
            }

            data.child_frame_id = childFrameId;
            data.pose.covariance[0] = covariancePose[0];
            data.pose.covariance[7] = covariancePose[1];
            data.pose.covariance[14] = covariancePose[2];
            data.pose.covariance[21] = covariancePose[3];
            data.pose.covariance[28] = covariancePose[4];
            data.pose.covariance[35] = covariancePose[5];
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