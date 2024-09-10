using geometry_msgs.msg;
using std_msgs;
using std_msgs.msg;

namespace sensor_msgs
{
    namespace msg
    {
        public class PointField : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "sensor_msgs.msg.PointField";
            }

            // Constants for data types
            public const byte INT8 = 1;
            public const byte UINT8 = 2;
            public const byte INT16 = 3;
            public const byte UINT16 = 4;
            public const byte INT32 = 5;
            public const byte UINT32 = 6;
            public const byte FLOAT32 = 7;
            public const byte FLOAT64 = 8;

            public string name; // Name of the field
            public uint offset; // Offset from the start of the point struct
            public byte datatype; // Datatype of the field
            public uint count; // Number of elements in the field
        }

        public class PointCloud2 : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "sensor_msgs.msg.PointCloud2";
            }

            public Header header { get; set; } = new Header();

            public uint height; // Height of the point cloud data
            public uint width; // Width of the point cloud data

            public PointField[] fields; // Array of PointField messages that describe the layout of the data

            public bool is_bigendian; // Is this data bigendian?
            public uint point_step; // Length of a point in bytes
            public uint row_step; // Length of a row in bytes

            public byte[] data; // Actual point cloud data, serialized as a byte array

            public bool is_dense; // Is this point cloud data dense?
        }

        public class Imu : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "sensor_msgs.msg.Imu";
            }

            public Header header { get; set; } = new Header();
            public Quaternion orientation = new Quaternion() { w = 1 };
            public double[] orientation_covariance = new double[9];
            public Vector3 angular_velocity = new Vector3();
            public double[] angular_velocity_covariance = new double[9];
            public Vector3 linear_acceleration = new Vector3();
            public double[] linear_acceleration_covariance = new double[9];
        }

        public class NavSatStatus : IRosMsg
        {
            // ROS Message Type
            string IRosMsg.GetRosType()
            {
                return "sensor_msgs.msg.NavSatStatus";
            }

            // Constants for Status
            public const sbyte STATUS_NO_FIX = -1;
            public const sbyte STATUS_FIX = 0;
            public const sbyte STATUS_SBAS_FIX = 1;
            public const sbyte STATUS_GBAS_FIX = 2;

            // Constants for Service
            public const ushort SERVICE_GPS = 1;
            public const ushort SERVICE_GLONASS = 2;
            public const ushort SERVICE_COMPASS = 4;
            public const ushort SERVICE_GALILEO = 8;

            /// <summary>
            /// Status of the GPS fix.
            /// </summary>
            public sbyte status;

            /// <summary>
            /// Which Global Navigation Satellite System (GNSS) is being used.
            /// </summary>
            public ushort service;

            public NavSatStatus()
            {
                status = STATUS_NO_FIX;
                service = SERVICE_GPS;
            }
        }

        public class NavSatFix : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "sensor_msgs.msg.NavSatFix";
            }

            public const byte COVARIANCE_TYPE_UNKNOWN = 0;
            public const byte COVARIANCE_TYPE_APPROXIMATED = 1;
            public const byte COVARIANCE_TYPE_DIAGONAL_KNOWN = 2;
            public const byte COVARIANCE_TYPE_KNOWN = 3;

            public Header header { get; set; } = new Header();

            public NavSatStatus status = new NavSatStatus();

            /// <summary>
            /// Latitude [degrees]. Positive is north of equator; negative is south
            /// </summary>
            public double latitude;

            /// <summary>
            /// Longitude [degrees]. Positive is east of prime meridian; negative is west.
            /// </summary>
            public double longitude;

            /// <summary>
            /// Altitude [m]. Positive is above the WGS 84 ellipsoid 
            /// (quiet NaN if no altitude is available).
            /// </summary>
            public double altitude;

            /// <summary>
            /// // Position covariance [m^2] defined relative to a tangential plane
            /// through the reported position. The components are East, North, and
            /// Up (ENU), in row-major order.
            /// Beware: this coordinate system exhibits singularities at the poles.
            /// </summary>
            public double[] position_covariance = new double[9];

            /// <summary>
            /// If the covariance of the fix is known, fill it in completely. If the
            /// GPS receiver provides the variance of each measurement, put them
            /// along the diagonal. If only Dilution of Precision is available,
            /// estimate an approximate covariance from that.
            /// </summary>
            public byte position_covariance_type;
        }


        public class CompressedImage : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "sensor_msgs.msg.CompressedImage";
            }

            public Header header { get; set; } = new Header();

            /// <summary>
            /// Specifies the format of the data
            /// Acceptable values:
            /// jpeg, png
            /// </summary>
            public string format;

            /// <summary>
            /// Compressed image buffer
            /// </summary>
            public byte[] data;
        }

        public class RegionOfInterest : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "sensor_msgs.msg.RegionOfInterest";
            }

            /// <summary>
            /// Leftmost pixel of the ROI.(0 if the ROI includes the left edge of the image)
            /// <summary>
            public uint x_offset;

            /// <summary>
            /// Topmost pixel of the ROI. (0 if the ROI includes the top edge of the image)
            /// <summary>
            public uint y_offset;

            /// <summary>
            /// Height of ROI
            /// <summary>
            public uint height;

            /// <summary>
            /// Width of ROI
            /// <summary>
            public uint width;

            /// <summary>
            /// True if a distinct rectified ROI should be calculated from the "raw"
            /// ROI in this message. Typically this should be False if the full image
            /// is captured (ROI not used), and True if a subwindow is captured (ROI used).
            /// <summary>
            public bool do_rectify;
        }

        public class CameraInfo : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "sensor_msgs.msg.CameraInfo";
            }

            public Header header { get; set; } = new Header();

            /// <summary>
            /// The image dimensions (height) with which the camera was calibrated. 
            /// Normally this will be the full camera resolution in pixels.
            /// <summary>
            public uint height;

            /// <summary>
            /// The image dimensions (width) with which the camera was calibrated. 
            /// Normally this will be the full camera resolution in pixels.
            /// <summary>
            public uint width;

            /// <summary>
            /// The distortion model used. Supported models are listed in
            /// sensor_msgs/distortion_models.hpp. For most cameras, "plumb_bob" - a
            /// simple model of radial and tangential distortion - is sufficent.
            /// <summary>
            public string distortion_model;

            /// <summary>
            /// The distortion parameters, size depending on the distortion model.
            /// For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
            /// </summary>
            public double[] d;

            /// <summary>
            /// Intrinsic camera matrix for the raw (distorted) images.3x3 row-major matrix.
            /// [fx 0 cx] 
            /// K = [ 0 fy cy]
            /// [ 0 0 1]
            /// Projects 3D points in the camera coordinate frame to 2D pixel coordinates 
            /// using the focal lengths (fx, fy) and principal point (cx, cy).
            /// </summary>
            public double[] k = new double[9];

            /// <summary>
            /// Rectification matrix (stereo cameras only). 3x3 row-major matrix.
            /// A rotation matrix aligning the camera coordinate system to the ideal
            /// stereo image plane so that epipolar lines in both stereo images are
            /// parallel.
            /// </summary>
            public double[] r = new double[9];

            /// <summary>
            /// Projection/camera matrix. 3x4 row-major matrix.
            /// [fx' 0 cx' Tx]
            /// P = [ 0 fy' cy' Ty]
            /// [ 0 0 1 0]
            /// By convention, this matrix specifies the intrinsic (camera) matrix
            /// of the processed (rectified) image.
            /// </summary>
            public double[] p = new double[12];

            /// <summary>
            /// Binning refers here to any camera setting which combines rectangular
            /// neighborhoods of pixels into larger "super-pixels." It reduces the
            /// resolution of the output image to
            /// (width / binning_x) x (height / binning_y).
            /// The default values binning_x = binning_y = 0 is considered the same
            /// as binning_x = binning_y = 1 (no subsampling).
            /// </summary>
            public uint binning_x;

            /// <summary>
            /// Binning refers here to any camera setting which combines rectangular
            /// neighborhoods of pixels into larger "super-pixels." It reduces the
            /// resolution of the output image to
            /// (width / binning_x) x (height / binning_y).
            /// The default values binning_x = binning_y = 0 is considered the same
            /// as binning_x = binning_y = 1 (no subsampling).
            /// </summary>
            public uint binning_y;

            /// <summary>
            /// Region of interest (subwindow of full camera resolution), given in
            /// full resolution (unbinned) image coordinates. A particular ROI
            /// always denotes the same window of pixels on the camera sensor,
            /// regardless of binning settings.
            /// </summary>
            public RegionOfInterest roi;
        }

        public class Image : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "sensor_msgs.msg.Image";
            }

            public Header header { get; set; } = new Header();

            /// <summary>
            /// image height, that is, number of rows
            /// </summary>
            public uint height;

            /// <summary>
            /// image width, that is, number of columns
            /// </summary>
            public uint width;

            /// <summary>
            /// Encoding of pixels -- channel meaning, ordering, size
            /// taken from the list of strings in include/sensor_msgs/image_encodings.hpp
            /// Encoding type example: "rgb8", "bgr8", "mono8", "16UC1", "32FC1"
            /// </summary>
            public string encoding;

            /// <summary>
            /// is this data bigendian?
            /// </summary>
            public byte is_bigendian;

            /// <summary>
            /// Full row length in bytes
            /// </summary>
            public uint step;

            /// <summary>
            /// actual matrix data, size is (step * rows)
            /// </summary>
            public byte[] data;
        }
    }
}