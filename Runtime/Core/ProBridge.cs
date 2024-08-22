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
        private string _ip = "127.0.0.1";
        private bool _active = true;
        private Thread _th = null;
        private SubscriberSocket _socket;


        public ProBridge(int port = 47777, string ip = "127.0.0.1")
        {
            _ip = ip;
            _port = port;
            (_th = new Thread(new ThreadStart(Receive))).Start();
        }

        public void Dispose()
        {
            _active = false;
            _socket.Close();
            NetMQConfig.Cleanup(
                false); // Must be here to work more than once, and false to not block when there are unprocessed messages.
            if (_th != null)
            {
                if (_th.Join(1000))
                    _th?.Abort();
            }
        }

        public void SendMsg(PublisherSocket publisher, Msg msg)
        {
            if (publisher == null || msg == null) return;

            var messageData = new Dictionary<string, object>
            {
                { "v", msg.v },
                { "t", msg.t },
                { "n", msg.n },
#if ROS_V2
                { "q", msg.q },
#endif
                { "c", msg.c }
            };

            var json = JsonConvert.SerializeObject(messageData);
            var header = CompressData(json);

            byte[] rosMsg = CDRSerializer.Serialize(msg.d);

            if (msg.c > 0)
            {
                rosMsg = CompressData(rosMsg, msg.c);
            }

            var buf = new byte[sizeof(short) + header.Length + rosMsg.Length];
            Buffer.BlockCopy(BitConverter.GetBytes((short)header.Length), 0, buf, 0, sizeof(short));
            Buffer.BlockCopy(header, 0, buf, sizeof(short), header.Length);
            Buffer.BlockCopy(rosMsg, 0, buf, sizeof(short) + header.Length, rosMsg.Length);

            publisher.SendFrame(buf);
        }

        private byte[] CompressData(string data)
        {
            using (var compressedStream = new MemoryStream())
            using (var zipStream = new GZipStream(compressedStream, CompressionLevel.Fastest))
            {
                var dataBytes = Encoding.ASCII.GetBytes(data);
                zipStream.Write(dataBytes, 0, dataBytes.Length);
                zipStream.Close();
                return compressedStream.ToArray();
            }
        }

        private byte[] CompressData(byte[] data, int compressionLevel = 1)
        {
            // TODO: use compressionLevel (current issue is that there is only 3 levels in GZipStream)
            using (var compressedStream = new MemoryStream())
            using (var zipStream = new GZipStream(compressedStream, CompressionLevel.Fastest))
            {
                zipStream.Write(data, 0, data.Length);
                zipStream.Close();
                return compressedStream.ToArray();
            }
        }

        private void Receive()
        {
            _socket = new SubscriberSocket();

            _socket.Connect($"tcp://{_ip}:{_port}");
            _socket.Subscribe("");

            while (_active)
            {
                try
                {
                    if (!_socket.TryReceiveFrameBytes(out var messageData))
                        continue;

                    ProcessMessage(messageData);
                }
                catch (Exception e)
                {
                    Console.WriteLine(e.ToString());
                }
            }
        }

        private void ProcessMessage(byte[] messageData)
        {
            var headerSize = BitConverter.ToInt16(messageData, 0);
            var headerBytes = new byte[headerSize];
            Array.Copy(messageData, 2, headerBytes, 0, headerSize);

            var msg = DeserializeMessage(headerBytes);
            var rosMsg = GetROSMessage(messageData, headerSize);

            msg.d = msg.c > 0 ? DecompressROSMessage(rosMsg, msg.c) : rosMsg;

            onMessageHandler?.Invoke(msg);
        }

        private Msg DeserializeMessage(byte[] headerBytes)
        {
            using (var compressedStream = new MemoryStream(headerBytes))
            using (var zipStream = new GZipStream(compressedStream, CompressionMode.Decompress))
            {
                var header = new byte[headerBytes.Length];
                zipStream.Read(header, 0, header.Length);
                return JsonConvert.DeserializeObject<Msg>(Encoding.UTF8.GetString(header));
            }
        }

        private byte[] GetROSMessage(byte[] messageData, int headerSize)
        {
            var rosMsgSize = messageData.Length - 2 - headerSize;
            var rosMsg = new byte[rosMsgSize];
            Array.Copy(messageData, 2 + headerSize, rosMsg, 0, rosMsgSize);
            return rosMsg;
        }

        private byte[] DecompressROSMessage(byte[] rosMsg, int compressionLevel = 1)
        {
            // TODO: use compressionLevel (current issue is that there is only 3 levels in GZipStream)
            using (var subcompressedStream = new MemoryStream(rosMsg))
            using (var subzipStream = new GZipStream(subcompressedStream, CompressionMode.Decompress))
            {
                var decompressedBytes = new byte[subzipStream.Length];
                subzipStream.Read(decompressedBytes, 0, (int)subzipStream.Length);
                return decompressedBytes;
            }
        }
    }
}