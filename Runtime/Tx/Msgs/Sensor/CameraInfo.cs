using System;
using System.Collections;
using UnityEngine;

namespace ProBridge.Tx.Sensor
{
    [AddComponentMenu("ProBridge/Tx/CameraInfo")]
    public class CameraInfo : ProBridgeTxStamped<ROS.Msgs.Sensors.CameraInfo>
    {
        public Camera camera;

        protected override void OnStart()
        {
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data.height = (uint)camera.pixelHeight;
            data.width = (uint)camera.pixelWidth;
            
            data.distortion_model = "plumb_bob";

            data.d = new double[] { 0, 0, 0, 0, 0 };

            data.k[0] = camera.focalLength; // fx
            data.k[1] = 0;
            data.k[2] = camera.pixelWidth / 2; // cx
            data.k[3] = 0;
            data.k[4] = camera.focalLength; // fy
            data.k[5] = camera.pixelHeight / 2; // cy
            data.k[6] = 0;
            data.k[7] = 0; 
            data.k[8] = 1; // bottom-right corner of K is 1

            data.r[0] = 1;
            data.r[1] = 0;
            data.r[2] = 0;
            data.r[3] = 0;
            data.r[4] = 1;
            data.r[5] = 0;
            data.r[6] = 0;
            data.r[7] = 0;
            data.r[8] = 1;

            data.p[0] = camera.focalLength; // fx
            data.p[1] = 0;
            data.p[2] = camera.pixelWidth / 2;  // cx
            data.p[3] = 0; // Tx
            data.p[4] = 0;
            data.p[5] = camera.focalLength; // fy
            data.p[6] = camera.pixelHeight / 2; // cy
            data.p[7] = 0; // Ty
            data.p[8] = 0;
            data.p[9] = 0;
            data.p[10] = 1;
            data.p[11] = 0;  // For monocular cameras, last element of P is 0

            data.binning_x = 1;
            data.binning_y = 1;


            data.roi = new ROS.Msgs.Sensors.RegionOfInterest
            {
                x_offset = 0,
                y_offset = 0,
                height = data.height,
                width = data.width,
                do_rectify = false
            };

            return base.GetMsg(ts);
        }

    }
}