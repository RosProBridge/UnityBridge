using std_msgs;

namespace std_srvs
{
    namespace srv
    {
        // Request/response types follow the rosidl naming: <Service>_Request / <Service>_Response.
        // ROS 2 adds a dummy uint8 member to empty structures, so it is part of their CDR data.

        public class Empty_Request : IRosMsg
        {
#if ROS_V2
            public byte structure_needs_at_least_one_member;
#endif
            public string GetRosType() => "std_srvs.srv.Empty_Request";
        }

        public class Empty_Response : IRosMsg
        {
#if ROS_V2
            public byte structure_needs_at_least_one_member;
#endif
            public string GetRosType() => "std_srvs.srv.Empty_Response";
        }

        public class Trigger_Request : IRosMsg
        {
#if ROS_V2
            public byte structure_needs_at_least_one_member;
#endif
            public string GetRosType() => "std_srvs.srv.Trigger_Request";
        }

        public class Trigger_Response : IRosMsg
        {
            /// <summary>Indicate successful run of triggered service.</summary>
            public bool success;

            /// <summary>Informational, e.g. for error messages.</summary>
            public string message = "";

            public string GetRosType() => "std_srvs.srv.Trigger_Response";
        }

        public class SetBool_Request : IRosMsg
        {
            /// <summary>E.g. for hardware enabling / disabling.</summary>
            public bool data;

            public string GetRosType() => "std_srvs.srv.SetBool_Request";
        }

        public class SetBool_Response : IRosMsg
        {
            /// <summary>Indicate successful run of triggered service.</summary>
            public bool success;

            /// <summary>Informational, e.g. for error messages.</summary>
            public string message = "";

            public string GetRosType() => "std_srvs.srv.SetBool_Response";
        }
    }
}
