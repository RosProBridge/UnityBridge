using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Text;
using System.IO;
using System.IO.Compression;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;
using Newtonsoft.Json;
using ProBridge.Utils;
using Unity.Profiling;

namespace ProBridge
{
    public class ProBridge : IDisposable
    {
        private const int MAX_MSGS_PER_FRAME = 100;

        public class Msg
        {
            /// <summary>Value of <see cref="k"/>: the sender serves service <see cref="n"/> of type <see cref="t"/>.</summary>
            public const string KindAdvertise = "adv";

            /// <summary>Value of <see cref="k"/> for a service request.</summary>
            public const string KindRequest = "req";

            /// <summary>Value of <see cref="k"/> for a service response.</summary>
            public const string KindResponse = "res";

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
            /// Quality of service, like as "qos_profile_system_default" or "qos_profile_sensor_data"
            /// </summary>
            public Qos q;
#else
            /// <summary>
            /// Latching flag for the message. If true, the message is a latched version of the previous message.
            /// </summary>
            public bool l = false;
#endif
            /// <summary>
            /// Data Compression Level (0-9)
            /// </summary>
            public int c;

            /// <summary>
            /// Value of object
            /// </summary>
            public object d;

            /// <summary>
            /// Message kind: null for a topic message, <see cref="KindAdvertise"/>, <see cref="KindRequest"/> or <see cref="KindResponse"/> for a service.
            /// For a service call <see cref="n"/> is the service name and <see cref="t"/> the service type.
            /// </summary>
            public string k;

            /// <summary>
            /// Service call id: a response carries the id of its request.
            /// </summary>
            public long id;

            /// <summary>
            /// Advertisement or request: port of the sender's <see cref="ProBridgeServer"/>. The receiver connects
            /// to it (at the sender's IP) to send service requests / responses back, no config needed.
            /// </summary>
            public int replyPort;
        }


        public enum MessageType
        {
            Log,
            Error,
            Warning
        }

        public delegate void OnMessage(Msg msg);
        public delegate void OnDebugMsg(string msg, MessageType messageType);

        public OnMessage onMessageHandler = null;
        public OnDebugMsg onDebugHandler = null;


        private int _port;
        private string _ip = "127.0.0.1";
        private PullSocket _pullSocket;
        private ProBridgeConnectionMonitor _connectionMonitor;


        public bool IsConnected => _connectionMonitor != null && _connectionMonitor.IsConnected;

        public ProBridge(int port = 47777, string ip = "127.0.0.1")
        {
            _ip = ip;
            _port = port;


            // Attach the monitor before Bind, otherwise peers reconnecting immediately are never reported.
            _pullSocket = new PullSocket();
            SetupMonitor();
            _pullSocket.Bind($"tcp://{_ip}:{_port}");
        }

        public void SetupMonitor()
        {
            if (_pullSocket == null || _connectionMonitor != null)
                return;

            _connectionMonitor = new ProBridgeConnectionMonitor(
                _pullSocket,
                // Unique per instance: the previous scene's monitor may still hold its endpoint.
                $"inproc://monitor-server-{_ip}:{_port}-{Guid.NewGuid():N}",
                ProBridgeConnectionMonitor.Mode.Bind);
        }

        public void PumpConnectionEvents(Action<bool> onStatusChanged)
        {
            _connectionMonitor?.Pump(onStatusChanged);
        }

        public void Dispose()
        {
            _connectionMonitor?.Dispose();
            _pullSocket.Close();
        }

        private static readonly ProfilerMarker SerializeMarker = new ProfilerMarker("ProBridge.Serialize");
        private static readonly ConcurrentBag<MemoryStream> StreamPool = new ConcurrentBag<MemoryStream>();
        private const int StreamPoolSize = 32;

        /// <summary>
        /// Sends a message through <paramref name="host"/>. Only the CDR serialization runs on the calling (main) thread,
        /// into a pooled buffer; the header, compression and the socket send run on the host's sender thread.
        /// </summary>
        public void SendMsg(ProBridgeHost host, Msg msg)
        {
            if (host == null || host.pushSocket == null || msg == null) return;

            var header = BuildHeader(msg);
            var payload = RentStream();
            try
            {
                using (SerializeMarker.Auto())
                    CDRSerializer.Serialize(msg.d, payload);
            }
            catch (Exception e)
            {
                ReturnStream(payload);
                LogError($"Failed to serialize message for {msg.n} of type {msg.t} : {e}");
                return;
            }

            host.EnqueueFrame(header, payload, msg.c);
        }

        /// <summary>
        /// Synchronous send on the calling thread. Prefer <see cref="SendMsg(ProBridgeHost, Msg)"/>:
        /// it keeps compression and the socket send off the main thread.
        /// </summary>
        public void SendMsg(PushSocket pushSocket, Msg msg)
        {
            if (pushSocket == null || msg == null) return;

            var payload = RentStream();
            try
            {
                CDRSerializer.Serialize(msg.d, payload);
            }
            catch (Exception e)
            {
                ReturnStream(payload);
                LogError($"Failed to serialize message for {msg.n} of type {msg.t} : {e}");
                return;
            }

            byte[] frame = null;
            int length = BuildFrame(BuildHeader(msg), payload, msg.c, ref frame);
            ReturnStream(payload);
            pushSocket.TrySendFrame(frame, length);
        }

        internal static Dictionary<string, object> BuildHeader(Msg msg)
        {
            var messageData = new Dictionary<string, object>
            {
                { "v", msg.v },
                { "t", msg.t },
                { "n", msg.n },
#if ROS_V2
                { "q", msg.q.GetValue() },
#else
                { "l", msg.l },
#endif
                { "c", msg.c }
            };
            if (msg.k != null)
            {
                messageData["k"] = msg.k;
                messageData["id"] = msg.id;
                if (msg.replyPort > 0)
                    messageData["p"] = msg.replyPort;
            }
            return messageData;
        }

        /// <summary>
        /// Builds the wire frame (header length, gzipped JSON header, CDR payload compressed if requested)
        /// into <paramref name="frame"/>, growing it when needed. Returns the frame length.
        /// </summary>
        // Gzipped headers by JSON: a topic's header doesn't change, gzip (and its large buffers) is needed once.
        // Service messages (call id in the header) are not cached.
        private static readonly ConcurrentDictionary<string, byte[]> HeaderCache = new ConcurrentDictionary<string, byte[]>();
        private const int HeaderCacheLimit = 1024;

        private static byte[] GetHeader(Dictionary<string, object> headerData)
        {
            var json = JsonConvert.SerializeObject(headerData);
            if (headerData.ContainsKey("k"))
                return CompressData(json);

            if (HeaderCache.TryGetValue(json, out var header))
                return header;
            if (HeaderCache.Count >= HeaderCacheLimit)
                HeaderCache.Clear();
            return HeaderCache[json] = CompressData(json);
        }

        internal static int BuildFrame(Dictionary<string, object> headerData, MemoryStream payload, int compressionLevel, ref byte[] frame)
        {
            var header = GetHeader(headerData);

            byte[] body = payload.GetBuffer();
            int bodyLength = (int)payload.Length;
            if (compressionLevel > 0)
            {
                body = CompressData(body, bodyLength, compressionLevel);
                bodyLength = body.Length;
            }

            int length = sizeof(short) + header.Length + bodyLength;
            if (frame == null || frame.Length < length)
                frame = new byte[length];

            frame[0] = (byte)(header.Length & 0xFF);
            frame[1] = (byte)((header.Length >> 8) & 0xFF);
            Buffer.BlockCopy(header, 0, frame, sizeof(short), header.Length);
            Buffer.BlockCopy(body, 0, frame, sizeof(short) + header.Length, bodyLength);
            return length;
        }

        internal static MemoryStream RentStream()
        {
            return StreamPool.TryTake(out var stream) ? stream : new MemoryStream();
        }

        internal static void ReturnStream(MemoryStream stream)
        {
            if (StreamPool.Count < StreamPoolSize)
                StreamPool.Add(stream);
        }

        private static byte[] CompressData(string data)
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

        private static byte[] CompressData(byte[] data, int length, int compressionLevel = 1)
        {
            using (var compressedStream = new MemoryStream())
            using (var zipStream = new GZipStream(compressedStream,
                       (compressionLevel == 1 ? CompressionLevel.Fastest : CompressionLevel.Optimal)))
            {
                zipStream.Write(data, 0, length);
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

            msg.d = msg.c > 0 ? DecompressROSMessage(rosMsg) : rosMsg;

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
#else
                    msg.l = (bool)messageData["l"];
#endif
                    msg.c = (int)tmpC;

                    if (messageData.TryGetValue("k", out var kind))
                        msg.k = (string)kind;
                    if (messageData.TryGetValue("id", out var id))
                        msg.id = (long)id;
                    
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

        private byte[] DecompressROSMessage(byte[] rosMsg)
        {
            using (var subcompressedStream = new MemoryStream(rosMsg))
            using (var subzipStream = new GZipStream(subcompressedStream, CompressionMode.Decompress))
            using (var decompressedStream = new MemoryStream())
            {
                subzipStream.CopyTo(decompressedStream);
                return decompressedStream.ToArray();
            }
        }

        private void LogMessage(string message)
        {
            onDebugHandler?.Invoke(message, MessageType.Log);
        }
        
        private void LogError(string message)
        {
            onDebugHandler?.Invoke(message, MessageType.Error);
        }

        private void LogWarning(string message)
        {
            onDebugHandler?.Invoke(message, MessageType.Warning);
        }
    }
}