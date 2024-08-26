
using System;
using ProBridge.ROS.Msgs.Sensors;
using ProBridge.ROS.Msgs.Std;

namespace ProBridge.Tx.Sensor
{
    public class PointCloud2Tx : ProBridgeTxStamped<ROS.Msgs.Sensors.PointCloud2>
    {
        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            //RunWithSampleData();


            return base.GetMsg(ts);
        }

        private void RunWithSampleData()
        {
            PointField[] fields = new PointField[3];

            fields[0] = new PointField
            {
                name = "x",
                offset = 0,
                datatype = PointField.FLOAT32,
                count = 1
            };

            fields[1] = new PointField
            {
                name = "y",
                offset = 4,
                datatype = PointField.FLOAT32,
                count = 1
            };

            fields[2] = new PointField
            {
                name = "z",
                offset = 8,
                datatype = PointField.FLOAT32,
                count = 1
            };

            // Sample point cloud data (e.g., 3 points)
            byte[] data = new byte[36];
            BitConverter.GetBytes(1.0f).CopyTo(data, 0);   // x1
            BitConverter.GetBytes(1.0f).CopyTo(data, 4);   // y1
            BitConverter.GetBytes(1.0f).CopyTo(data, 8);   // z1
            BitConverter.GetBytes(2.0f).CopyTo(data, 12);  // x2
            BitConverter.GetBytes(2.0f).CopyTo(data, 16);  // y2
            BitConverter.GetBytes(2.0f).CopyTo(data, 20);  // z2
            BitConverter.GetBytes(3.0f).CopyTo(data, 24);  // x3
            BitConverter.GetBytes(3.0f).CopyTo(data, 28);  // y3
            BitConverter.GetBytes(3.0f).CopyTo(data, 32);  // z3

            
            this.data.header = new Header();
            this.data.height = 1;            // Unordered point cloud, so height is 1
            this.data.width = 3;             // Three points in the cloud
            this.data.is_bigendian = false;  // Assuming little-endian format
            this.data.point_step = 12;       // 12 bytes per point (3 * 4 bytes for x, y, z)
            this.data.row_step = 36;         // 36 bytes per row (since height is 1, row_step = width * point_step)
            this.data.data = data;           // The point cloud data
            this.data.is_dense = true;       // Assume all points are valid
            this.data.fields = fields;       // The layout of the data
        }
    }
}