using System;

namespace std_msgs
{
#if ROS_V2
    public class Duration
    {
        public uint sec;
        public uint nanosec;

        public static implicit operator Duration(TimeSpan value)
        {
            return new Duration()
            {
                sec = (uint)(value.TotalSeconds - 62135596800L), // convert seconds into unix time
                nanosec = (uint)((value.Ticks - ((long)value.TotalSeconds * TimeSpan.TicksPerSecond)) *
                                 (1e9d / TimeSpan.TicksPerSecond))
            };
        }

        public static implicit operator TimeSpan(Duration value)
        {
            return new TimeSpan((long)((value.sec + 62135596800L) * TimeSpan.TicksPerSecond +
                                       (value.nanosec / 1e6 * TimeSpan.TicksPerMillisecond)));
        }
    }
#else
      public class Duration
    {
        public uint secs;
        public uint nsecs;

        public static implicit operator Duration(TimeSpan value)
        {
            return new Duration()
            {
                secs = (uint)(value.TotalSeconds - 62135596800L), // convert seconds into unix time
                nsecs =
 (uint)((value.Ticks - ((long)value.TotalSeconds * TimeSpan.TicksPerSecond)) * (1e9d / TimeSpan.TicksPerSecond))
            };
        }

        public static implicit operator TimeSpan(Duration value)
        {
            return new TimeSpan((long)((value.secs + 62135596800L) * TimeSpan.TicksPerSecond + (value.nsecs / 1e6 * TimeSpan.TicksPerMillisecond)));
        }
    }
#endif
}