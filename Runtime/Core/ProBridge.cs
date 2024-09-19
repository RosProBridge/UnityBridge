using System;
using System.Collections.Generic;
using System.Text;
using System.IO;
using System.IO.Compression;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;
using Newtonsoft.Json;
using ProBridge.Utils;

namespace ProBridge
{
    public class ProBridge : IDisposable
    {
        private const int MAX_MSGS_PER_FRAME = 100;

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
            public Qos q;
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
        private PullSocket _pullSocket;


        public ProBridge(int port = 47777, string ip = "127.0.0.1")
        {
            _ip = ip;
            _port = port;


            _pullSocket = new PullSocket();
            _pullSocket.Bind($"tcp://{_ip}:{_port}");
        }

        public void Dispose()
        {
            _active = false;
            // _socket.Close();
            NetMQConfig.Cleanup(
                false); // Must be here to work more than once, and false to not block when there are unprocessed messages.
            if (_th != null)
            {
                if (_th.Join(1000))
                    _th?.Abort();
            }
        }

        public void SendMsg(PushSocket pushSocket, Msg msg)
        {
            if (pushSocket == null || msg == null) return;
            var messageData = new Dictionary<string, object>
            {
                { "v", msg.v },
                { "t", msg.t },
                { "n", msg.n },
#if ROS_V2
                { "q", msg.q.GetValue() },
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

            pushSocket.TrySendFrame(buf);
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

        public void TryReceive()
        {
            for (int i = 0; i < MAX_MSGS_PER_FRAME; i++)
            {
                if (!_pullSocket.TryReceiveFrameBytes(out var messageData))
                    break;
                ProcessMessage(messageData);
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
            using (var decompressedStream = new MemoryStream())
            {
                zipStream.CopyTo(decompressedStream);
                decompressedStream.Position = 0;
                using (var reader = new StreamReader(decompressedStream, Encoding.UTF8))
                {
                    var jsonString = reader.ReadToEnd();


                    var messageData = JsonConvert.DeserializeObject<Dictionary<string, object>>(jsonString);
                    

                    var tmpV = (long)messageData["v"];
                    var tmpC = (long)messageData["c"];

                    var msg = new Msg();
                    msg.v = (byte)tmpV;
                    msg.t = (string)messageData["t"];
                    msg.n = (string)messageData["n"];
#if ROS_V2
                    msg.q = new Qos(messageData["q"]);
#endif
                    msg.c = (int)tmpC;
                    
                    return msg;
                }
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