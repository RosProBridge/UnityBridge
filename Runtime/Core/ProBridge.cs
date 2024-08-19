using System;
using System.Collections.Generic;
using System.Text;
using System.IO;
using System.IO.Compression;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;

using Newtonsoft.Json;
using ProBridge.Utils;

namespace ProBridge
{
    public class ProBridge : IDisposable
    {
        private const uint MAX_UDP_SIZE = 65500;

        public class Msg
        {
            public byte v;

            /// <summary>
            /// Topic name
            /// </summary>
            public string n;

            /// <summary>
            /// Type of message
            /// </summary>
            public string t;
            

#if ROS_V2
            /// <summary>
            /// Quality od service, like as "qos_profile_system_default" or "qos_profile_sensor_data"
            /// </summary>
            public int q = 10;
#endif
            /// <summary>
            /// Data Compression Level (0-9)
            /// </summary>
            public int c;

            /// <summary>
            /// Value of object
            /// </summary>
            public object d;
        }

        public delegate void OnMessage(Msg msg);
        public OnMessage onMessageHandler = null;

        private bool _active = true;
        private UdpClient _recv = null;
        private UdpClient _send = new UdpClient();
        private Thread _th = null;


        public ProBridge(int port = 47777)
        {
            _recv = new UdpClient(port);
            (_th = new Thread(new ThreadStart(Receive))).Start();
        }

        public void Dispose()
        {
            _active = false;
            if (_th != null)
            {
                _recv?.Dispose();
                if (_th.Join(1000))
                    _th?.Abort();
            }
        }

        public void SendMsg(PublisherSocket publisher, Msg msg)
        {
            if ((publisher is null) || (msg is null))
                return;
            
            var dict = new Dictionary<string, object>();
            dict["v"] = msg.v;
            dict["t"] = msg.t;
            dict["n"] = msg.n;
#if ROS_V2
            dict["q"] = msg.q;
#endif
            dict["c"] = msg.c;
            
            var json = JsonConvert.SerializeObject(dict);
            var header = Encoding.ASCII.GetBytes(json);
            
            using (var compressedStream = new MemoryStream())
            using (var zipStream = new GZipStream(compressedStream, CompressionLevel.Fastest))
            {
                zipStream.Write(header, 0, header.Length);
                zipStream.Close();
                header = compressedStream.ToArray();
            }

            byte[] rosMsg = CDRSerializer.Serialize(msg.d);

            if (msg.c > 0)
            {
                using (var compressedStream = new MemoryStream())
                using (var zipStream = new GZipStream(compressedStream, CompressionLevel.Fastest))
                {
                    zipStream.Write(rosMsg, 0, rosMsg.Length);
                    zipStream.Close();
                    rosMsg = compressedStream.ToArray();
                }
            }

            short headerLen = (short)header.Length;
            
            byte[] buf = new byte[header.Length + sizeof(short) + rosMsg.Length];
            Buffer.BlockCopy(BitConverter.GetBytes(headerLen), 0, buf, 0, sizeof(short));
            Buffer.BlockCopy(header, 0, buf, sizeof(short), header.Length);
            Buffer.BlockCopy(rosMsg, 0, buf, sizeof(short) + header.Length, rosMsg.Length);
            
            publisher.SendFrame(buf);
        }

        public void Receive()
        {
            while (_active)
            {
                try
                {
                    IPEndPoint receivedIpEndPoint = null;
                    var receivedBytes = _recv.Receive(ref receivedIpEndPoint);

                    var n = 0;
                    var buf = new byte[65535];
                    using (var ms = new MemoryStream(receivedBytes))
                    using (var gz = new GZipStream(ms, CompressionMode.Decompress))
                        n = gz.Read(buf, 0, buf.Length);

                    if (n <= 0)
                        return;

                    Array.Resize(ref buf, n);
                    string json = Encoding.UTF8.GetString(buf);
                    var msg = JsonConvert.DeserializeObject<Msg>(json);

                    onMessageHandler?.Invoke(msg);
                }
                catch (Exception e)
                {
                    Console.WriteLine(e.ToString());
                }
            }
        }
    }
}