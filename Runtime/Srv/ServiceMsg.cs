namespace ProBridge.Srv
{
    internal static class ServiceMsg
    {
        private const string RequestSuffix = "_Request";

        /// <summary>"std_srvs.srv.Trigger_Request" -> "std_srvs.srv.Trigger".</summary>
        public static string ServiceTypeOf<TRequest>() where TRequest : std_msgs.IRosMsg, new()
        {
            var requestType = new TRequest().GetRosType();
            return requestType.EndsWith(RequestSuffix)
                ? requestType.Substring(0, requestType.Length - RequestSuffix.Length)
                : requestType;
        }

        public static ProBridge.Msg Create(string service, string serviceType, string kind, long id, object data, int replyPort = 0)
        {
            return new ProBridge.Msg
            {
#if ROS_V2
                v = 2,
                q = new Qos(),
#else
                v = 1,
#endif
                n = service,
                t = serviceType,
                c = 0,
                k = kind,
                id = id,
                replyPort = replyPort,
                d = data
            };
        }
    }
}
