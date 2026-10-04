namespace ProBridge.Tx
{
    /// <summary>
    /// Common view of the publishers (ProBridgeTx, TfSender) for UI and tools: switch sending on and off,
    /// count the messages actually sent.
    /// </summary>
    public interface IProBridgeTx
    {
        /// <summary>Topic the messages are sent to.</summary>
        string Topic { get; }

        /// <summary>False: nothing is built or sent (sensors don't compute their data).</summary>
        bool Active { get; set; }

        /// <summary>Messages handed to the bridge for sending since start (to measure the actual rate).</summary>
        long SentCount { get; }

        /// <summary>Send period in seconds (0: every physics step); can be changed at runtime.</summary>
        float SendRate { get; set; }
    }
}
