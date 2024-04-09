using System;
using UnityEngine;
using ProBridge.Utils;

namespace ProBridge.Tx.Sensor
{
    [AddComponentMenu("ProBridge/Tx/Sensor/NavSatFix")]
    public class NavSatFix : ProBridgeTxStamped<ROS.Msgs.Sensors.NavSatFix>
    {
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
            data.latitude = latitude;
            data.longitude = longitude;
            data.altitude = altitude;

            return base.GetMsg(ts);
        }
    }
}
