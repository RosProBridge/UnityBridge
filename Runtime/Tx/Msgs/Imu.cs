using System;
using UnityEngine;
using ProBridge.Utils;

namespace ProBridge.Tx
{
    [AddComponentMenu("ProBridge/Tx/Imu")]
    [RequireComponent(typeof(Rigidbody))]
    public class Imu : ProBridgeTxMsgStamped<ROS.Msgs.Sensors.Imu>
    {
        public Rigidbody Body { get; private set; }
        public Vector3 Acceleration { get; private set; }

        private float _lastTime;
        private Vector3 _lastVel;

        protected override string GetMsgType()
        {
            return "sensor_msgs.msg.Imu";
        }

        protected override void OnStart()
        {
            Body = GetComponent<Rigidbody>();
            _lastVel = Vector3.zero;
            _lastTime = Time.time;
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            var t = Time.time;
            var dt = t - _lastTime;
            if (dt <= 0)
                Debug.LogWarning("Empty delta time");
            else
            {
                var bv = Body.velocity;
                Acceleration = (bv - _lastVel) / dt;
                _lastVel = bv;
                _lastTime = t;
            }

            data.angular_velocity = (Quaternion.Inverse(Body.rotation) * Body.angularVelocity).ToRosAngular();
            data.linear_acceleration = Acceleration.ToRos();
            data.orientation = Body.rotation.ToRos();

            return base.GetMsg(ts);
        }
    }
}