using System;
using geometry_msgs.msg;
using sensor_msgs.msg;
using std_msgs;
using std_msgs.msg;

namespace visualization_msgs
{
    namespace msg
    {
        public class UVCoordinate : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "visualization_msgs.msg.UVCoordinate";
            }

            /// <summary>
            /// Location of the pixel as a ratio of the width of a 2D texture.
            /// Values should be in range: [0.0-1.0].
            /// </summary>
            public float u;

            /// <summary>
            /// Location of the pixel as a ratio of the width of a 2D texture.
            /// Values should be in range: [0.0-1.0].
            /// </summary>
            public float v;
        }

        public class MeshFile : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "visualization_msgs.msg.MeshFile";
            }

            /// <summary>
            /// The filename is used for both debug purposes and to provide a file extension
            /// for whatever parser is used.
            /// </summary>
            public string filename;

            /// <summary>
            /// This stores the raw text of the mesh file.
            /// </summary>
            public byte[] data;
        }
#if ROS_V2
        public class Marker : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "visualization_msgs.msg.Marker";
            }

            public const Int32 ARROW = 0;
            public const Int32 CUBE = 1;
            public const Int32 SPHERE = 2;
            public const Int32 CYLINDER = 3;
            public const Int32 LINE_STRIP = 4;
            public const Int32 LINE_LIST = 5;
            public const Int32 CUBE_LIST = 6;
            public const Int32 SPHERE_LIST = 7;
            public const Int32 POINTS = 8;
            public const Int32 TEXT_VIEW_FACING = 9;
            public const Int32 MESH_RESOURCE = 10;
            public const Int32 TRIANGLE_LIST = 11;
            public const Int32 ADD = 0;
            public const Int32 MODIFY = 0;
            public const Int32 DELETE = 2;
            public const Int32 DELETEALL = 3;

            /// <summary>
            ///  Header for timestamp and frame id.
            /// </summary>
            public Header header { get; set; } = new Header();

            /// <summary>
            /// Namespace in which to place the object.
            /// Used in conjunction with id to create a unique name for the object.
            /// </summary>
            public string ns;

            /// <summary>
            /// Object ID used in conjunction with the namespace for manipulating and deleting the object later.
            /// </summary>
            public Int32 id;

            /// <summary>
            /// Type of object.
            /// </summary>
            public Int32 type;

            /// <summary>
            /// Action to take; one of:
            /// - 0 add/modify an object
            /// - 1 (deprecated)
            /// - 2 deletes an object (with the given ns and id)
            /// - 3 deletes all objects (or those with the given ns if any)
            /// </summary>
            public Int32 action;

            /// <summary>
            /// Pose of the object with respect the frame_id specified in the header.
            /// </summary>
            public Pose pose;

            /// <summary>
            /// Scale of the object; 1,1,1 means default (usually 1 meter square).
            /// </summary>
            public Vector3 scale;

            /// <summary>
            /// Color of the object; in the range: [0.0-1.0]
            /// </summary>
            public ColorRGBA color;

            /// <summary>
            /// How long the object should last before being automatically deleted.
            /// 0 indicates forever.
            /// </summary>
            public Duration lifetime;

            /// <summary>
            /// If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
            /// </summary>
            public bool frame_locked;

            /// <summary>
            /// Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ARROW_STRIP, etc.)
            /// </summary>
            public Point[] points;

            /// <summary>
            /// Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
            /// The number of colors provided must either be 0 or equal to the number of points provided.
            /// NOTE: alpha is not yet used
            /// </summary>
            public ColorRGBA[] colors;

            /// <summary>
            /// Texture resource is a special URI that can either reference a texture file in
            /// a format acceptable to (resource retriever)[https://docs.ros.org/en/rolling/p/resource_retriever/]
            /// or an embedded texture via a string matching the format:
            /// "embedded://texture_name"
            /// </summary>
            public string texture_resource;

            /// <summary>
            /// An image to be loaded into the rendering engine as the texture for this marker.
            /// This will be used iff texture_resource is set to embedded.
            /// </summary>
            public CompressedImage texture;

            /// <summary>
            /// Location of each vertex within the texture; in the range: [0.0-1.0]
            /// </summary>
            public UVCoordinate[] uv_coordinates;

            /// <summary>
            /// Only used for text markers
            /// </summary>
            public string text;

            /// <summary>
            /// Only used for MESH_RESOURCE markers.
            /// Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
            /// Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
            /// use the following format for mesh_resource:
            ///   "embedded://mesh_name"
            /// </summary>
            public string mesh_resource;

            public MeshFile mesh_file;
            public bool mesh_use_embedded_materials;
        }
#else
    public class Marker : IRosMsg, IStamped
    {
        string IRosMsg.GetRosType() { return "visualization_msgs.msg.Marker"; }
     
        public const byte ARROW = 0;
        public const byte CUBE = 1;
        public const byte SPHERE = 2;
        public const byte CYLINDER = 3;
        public const byte LINE_STRIP = 4;
        public const byte LINE_LIST = 5;
        public const byte CUBE_LIST = 6;
        public const byte SPHERE_LIST = 7;
        public const byte POINTS = 8;
        public const byte TEXT_VIEW_FACING = 9;
        public const byte MESH_RESOURCE = 10;
        public const byte TRIANGLE_LIST = 11;
        public const byte ADD = 0;
        public const byte MODIFY = 0;
        public const byte DELETE = 2;
        public const byte DELETEALL = 3;

        /// <summary>
        ///  Header for timestamp and frame id.
        /// </summary>
        public Header header { get; set; } = new Header();

        /// <summary>
        /// Namespace in which to place the object.
        /// Used in conjunction with id to create a unique name for the object.
        /// </summary>
        public string ns;

        /// <summary>
        /// Object ID used in conjunction with the namespace for manipulating and deleting the object later.
        /// </summary>
        public Int32 id;

        /// <summary>
        /// Type of object.
        /// </summary>
        public Int32 type;

        /// <summary>
        /// Action to take; one of:
        /// - 0 add/modify an object
        /// - 1 (deprecated)
        /// - 2 deletes an object (with the given ns and id)
        /// - 3 deletes all objects (or those with the given ns if any)
        /// </summary>
        public Int32 action;

        /// <summary>
        /// Pose of the object with respect the frame_id specified in the header.
        /// </summary>
        public Pose pose;

        /// <summary>
        /// Scale of the object; 1,1,1 means default (usually 1 meter square).
        /// </summary>
        public Vector3 scale;

        /// <summary>
        /// Color of the object; in the range: [0.0-1.0]
        /// </summary>
        public ColorRGBA color;

        /// <summary>
        /// How long the object should last before being automatically deleted.
        /// 0 indicates forever.
        /// </summary>
        public Duration lifetime;

        /// <summary>
        /// If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
        /// </summary>
        public bool frame_locked;

        /// <summary>
        /// Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ARROW_STRIP, etc.)
        /// </summary>
        public Point[] points;

        /// <summary>
        /// Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
        /// The number of colors provided must either be 0 or equal to the number of points provided.
        /// NOTE: alpha is not yet used
        /// </summary>
        public ColorRGBA[] colors;


        /// <summary>
        /// Only used for text markers
        /// </summary>
        public string text;

        /// <summary>
        /// only used for MESH_RESOURCE markers
        /// </summary>
        public string mesh_resource;
        public bool mesh_use_embedded_materials;
    }
#endif
        public class MarkerArray : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "visualization_msgs.msg.MarkerArray";
            }

            public Marker[] markers;
        }
    }
}