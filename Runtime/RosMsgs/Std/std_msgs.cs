using System;

namespace std_msgs
{
    namespace msg
    {
        public abstract class StdMsg<T> : IRosMsg
        {
            public T data;

            public abstract string GetRosType();
        }

        public class StdTime : StdMsg<Time>
        {
            public override string GetRosType()
            {
                return "std_msgs.msg.Time";
            }

            public StdTime(TimeSpan time)
            {
                data = time;
            }
        }

        public class StdDuration : StdMsg<Duration>
        {
            public override string GetRosType()
            {
                return "std_msgs.msg.Duration";
            }

            public StdDuration(TimeSpan duration)
            {
                data = duration;
            }
        }

        public class StdBool : StdMsg<bool>
        {
            public override string GetRosType()
            {
                return "std_msgs.msg.Bool";
            }
        }

        public class StdFloat : StdMsg<float>
        {
            public override string GetRosType()
            {
                return "std_msgs.msg.Float32";
            }
        }

        public class StdFloat64 : StdMsg<double>
        {
            public override string GetRosType()
            {
                return "std_msgs.msg.Float64";
            }
        }

        public class StdInt : StdMsg<int>
        {
            public override string GetRosType()
            {
                return "std_msgs.msg.Int32";
            }
        }

        public class StdString : StdMsg<string>
        {
            public override string GetRosType()
            {
                return "std_msgs.msg.String";
            }
        }

        public class ColorRGBA : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "std_msgs.msg.ColorRGBA";
            }

            public float r;
            public float g;
            public float b;
            public float a;
        }

        public class Header : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "std_msgs.msg.Header";
            }

#if ROS_V2
#else
        public UInt32 seq;
#endif
            public Time stamp = new Time();
            public string frame_id;
        }

        public interface IStamped
        {
            Header header { get; }
        }

        public class MultiArrayDimension : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "std_msgs.msg.MultiArrayDimension";
            }

            /// <summary>
            /// label of given dimension
            /// </summary>
            public string label;

            /// <summary>
            /// size of given dimension (in type units)
            /// </summary>
            public UInt32 size;

            /// <summary>
            /// stride of given dimension
            /// </summary>
            public UInt32 stride;
        }

        public class MultiArrayLayout : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "std_msgs.msg.MultiArrayLayout";
            }

            /// <summary>
            /// Array of dimension properties
            /// </summary>
            public MultiArrayDimension[] dim;

            /// <summary>
            /// padding elements at front of data
            /// </summary>
            public UInt32 data_offset;
        }

        public class StdFloat64MultiArray : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "std_msgs.msg.Float64MultiArray";
            }

            /// <summary>
            /// specification of data layout
            /// </summary>
            public MultiArrayLayout layout;

            /// <summary>
            /// array of data
            /// </summary>
            public double[] data;
        }
    }
}