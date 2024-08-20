using System;
using System.Collections.Generic;
using System.Text;
using System.IO;
using System.IO.Compression;
using System.Net;
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

        
        private int _port;
        private bool _active = true;
        private Thread _th = null;


        public ProBridge(int port = 47777)
        {
            _port = port;
            (_th = new Thread(new ThreadStart(Receive))).Start();
        }

        public void Dispose()
        {
            _active = false;
            NetMQConfig.Cleanup();
            if (_th != null)
            {
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
            SubscriberSocket _socket;
            _socket = new SubscriberSocket();
            _socket.Connect($"tcp://127.0.0.1:{_port}");
            _socket.Subscribe("");
            while (_active)
            {
                try
                {
                    if (!_socket.TryReceiveFrameBytes(out var messageData)) continue;

                    short n = BitConverter.ToInt16(messageData, 0);
                    
                    

                    var headerBytes = new byte[n];
                    Array.Copy(messageData, 2, headerBytes, 0, n);

                    using (var compressedStream = new MemoryStream(headerBytes))
                    using (var zipStream = new GZipStream(compressedStream, CompressionMode.Decompress))
                    {
                        var header = new byte[n];
                        zipStream.Read(header, 0, n);

                        var msg = JsonConvert.DeserializeObject<Msg>(Encoding.UTF8.GetString(header));

                        var rosMsg = new byte[messageData.Length - 2 - n];
                        Array.Copy(messageData, 2 + n, rosMsg, 0, rosMsg.Length);
                        

                        if (msg.c > 0)
                        {
                            using (var subcompressedStream = new MemoryStream(rosMsg))
                            using (var subzipStream = new GZipStream(subcompressedStream, CompressionMode.Decompress))
                            {
                                var decompressedBytes = new byte[msg.c];
                                subzipStream.Read(decompressedBytes, 0, msg.c);
                                msg.d = decompressedBytes;
                            }
                        }
                        else
                        {
                            msg.d = rosMsg;
                        }
                        
                        onMessageHandler?.Invoke(msg);
                    }
                }
                catch (Exception e)
                {
                    Console.WriteLine(e.ToString());
                }
            }
            _socket.Close();
            NetMQConfig.Cleanup();
        }
    }
}