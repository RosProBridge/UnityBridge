using System;
using System.Text;
using System.IO;
using System.IO.Compression;
using System.Net;
using System.Net.Sockets;
using System.Threading;

using Newtonsoft.Json;

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
            public object q = 10;
#endif

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

        public void SendMsg(ProBridgeHost host, Msg msg)
        {
            if ((host is null) || (msg is null))
                return;

            var json = JsonConvert.SerializeObject(msg);
            var data = ASCIIEncoding.ASCII.GetBytes(json);
            byte[] buf = null;

            using (var compressedStream = new MemoryStream())
            using (var zipStream = new GZipStream(compressedStream, CompressionMode.Compress))
            {
                zipStream.Write(data, 0, data.Length);
                zipStream.Close();
                buf = compressedStream.ToArray();
            }

            if (buf.Length > MAX_UDP_SIZE)
                Console.WriteLine($"Skipping large message of size {buf.Length} bytes.");
            else
                _send.Send(buf, buf.Length, new IPEndPoint(IPAddress.Parse(host.addr), host.port));
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