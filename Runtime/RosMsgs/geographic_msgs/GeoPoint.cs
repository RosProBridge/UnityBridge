using System;
using std_msgs;


namespace geographic_msgs
{
    namespace msg
    {
        /// <summary>
        /// Geographic point, using the WGS 84 reference ellipsoid.
        /// </summary>
        [System.Serializable]
        public class  GeoPoint: IRosMsg
        {
            public string GetRosType()
            {
                return "geographic_msgs.msg.GeoPoint";
            }

            /// <summary>
            /// Latitude [degrees]. Positive is north of equator; negative is south
            /// (-90 <= latitude <= +90).
            /// </summary>
            public double latitude;

            /// <summary>
            /// Longitude [degrees]. Positive is east of prime meridian; negative is
            /// west (-180 <= longitude <= +180). At the poles, latitude is -90 or
            /// +90, and longitude is irrelevant, but must be in range.
            /// </summary>
            public double longitude;

            /// <summary>
            /// Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).
            /// </summary>
            public double altitude;
        }
    }
}