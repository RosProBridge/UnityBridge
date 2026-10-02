using System.Collections.Concurrent;
using System.Threading;
using NetMQ;

namespace ProBridge
{
    /// <summary>
    /// Buffer pool for NetMQ messages: NetMQ copies every sent frame into a buffer from its pool, and the default
    /// pool allocates a new array each time (garbage for every message). Buffers are pooled by power-of-two size.
    /// (NetMQ's own BufferManagerBufferPool needs System.ServiceModel, which Unity doesn't have.)
    /// </summary>
    internal sealed class ProBridgeBufferPool : IBufferPool
    {
        private const int MinSizeLog2 = 8;    // 256 B
        private const int MaxSizeLog2 = 24;   // 16 MB; larger buffers are not pooled
        private const int MaxPerBucket = 16;

        private static int _installed;

        private readonly ConcurrentBag<byte[]>[] _buckets = new ConcurrentBag<byte[]>[MaxSizeLog2 + 1];

        private ProBridgeBufferPool()
        {
            for (int i = MinSizeLog2; i <= MaxSizeLog2; i++)
                _buckets[i] = new ConcurrentBag<byte[]>();
        }

        /// <summary>Installs the pool for all NetMQ sockets (once per domain).</summary>
        public static void Install()
        {
            if (Interlocked.Exchange(ref _installed, 1) == 0)
                BufferPool.SetCustomBufferPool(new ProBridgeBufferPool());
        }

        public byte[] Take(int size)
        {
            int log2 = BucketOf(size);
            if (log2 > MaxSizeLog2)
                return new byte[size];

            return _buckets[log2].TryTake(out var buffer) ? buffer : new byte[1 << log2];
        }

        public void Return(byte[] buffer)
        {
            if (buffer == null)
                return;

            int length = buffer.Length;
            // Only buffers of exactly a bucket size came from here
            if ((length & (length - 1)) != 0)
                return;

            int log2 = BucketOf(length);
            if (log2 < MinSizeLog2 || log2 > MaxSizeLog2)
                return;

            var bucket = _buckets[log2];
            if (bucket.Count < MaxPerBucket)
                bucket.Add(buffer);
        }

        public void Dispose()
        {
        }

        private static int BucketOf(int size)
        {
            int log2 = MinSizeLog2;
            while ((1 << log2) < size && log2 <= MaxSizeLog2)
                log2++;
            return log2;
        }
    }
}
