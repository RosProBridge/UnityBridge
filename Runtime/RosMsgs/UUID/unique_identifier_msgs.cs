
using std_msgs;

namespace unique_identifier_msgs
{
   
    namespace msg
    {
        public  class UUID : IRosMsg
        {
            string IRosMsg.GetRosType()
            {
                return "unique_identifier_msgs.msg.UUID";
            }

            public byte[] uuid = new byte[16];
        }
    }
}
