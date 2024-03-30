using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System;
using ProBridge.ROS;

namespace ProBridge
{
    public class Nav : ProBridgeBehaviour<Nav>
    {
        public string frameId;

        [Header("Params")]
        public Vector3 startLLA = new Vector3();

        [Header("Values")]
        public double latitude;
        public double longitude;
        public double altitude;

        private Vector3 startPos;

        protected override void OnStart()
        {
            startPos = transform.position;
        }

        protected override void OnStop()
        {
        }

        private void Update()
        {
            double la = startLLA.x;
            double lo = startLLA.y;
            double al = startLLA.z;

            GeoConverter.Local2Global(transform.position - startPos, ref la, ref lo, ref al);

            latitude = la;
            longitude = lo;
            altitude = al;
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            var data = new ROS.Msgs.Sensors.NavSatFix
            {
                header = new ROS.Msgs.Std.Header()
                {
                    frame_id = frameId,
                    stamp = ts
                },
                latitude = latitude,
                longitude = longitude,
                altitude = altitude
            };

            return new ProBridge.Msg()
            {
                n = topic,
                t = "sensor_msgs.msg.NavSatFix",
                q = qos,
                d = data
            };
        }
    }
}
