using UnityEngine;

using System;

using ProBridge.ROS;

namespace ProBridge
{
    [RequireComponent(typeof(Rigidbody))]
    public class Imu : ProBridgeBehaviour<Imu>
    {
        public string frameId;

        public Rigidbody Body { get; private set; }
        public Vector3 Acceleration { get; private set; }

        private Vector3 _lastVel;

        protected override void OnStart()
        {
            Body = GetComponent<Rigidbody>();
            _lastVel = Vector3.zero;
        }

        protected override void OnStop()
        {
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            Acceleration = (Body.velocity - _lastVel) / Time.deltaTime;
            _lastVel = Body.velocity;
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            var data = new ROS.Msgs.Sensors.Imu()
            {
                header = new ROS.Msgs.Std.Header()
                {
                    frame_id = frameId,
                    stamp = ts
                },
                angular_velocity = (Quaternion.Inverse(Body.rotation) * Body.angularVelocity).ToRosAngular(),
                linear_acceleration = Acceleration.ToRos(),
                orientation = Body.rotation.ToRos()
            };

            return new ProBridge.Msg()
            {
                n = topic,
                t = "sensor_msgs.msg.Imu",
                q = qos,
                d = data
            };
        }
    }
}