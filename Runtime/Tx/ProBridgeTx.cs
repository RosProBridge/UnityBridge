using System;
using ProBridge.Utils;
using UnityEngine;

namespace ProBridge.Tx
{
    public abstract class ProBridgeTx<T> : MonoBehaviour, ISimStepListener where T : std_msgs.IRosMsg, new()
    {
        private const float SkippedReportInterval = 5f;

        #region Inspector
        public ProBridgeHost host;
        [Tooltip("Send period in simulation seconds. 0 sends every simulation (physics) step. " +
                 "Messages are never sent more often than once per step.")]
        [Min(0f)]
        public float sendRate = 0.025f;

        [Tooltip("Send only when SendMsg() is called. Send Rate is ignored.")]
        public bool manualSend = false;

        public string topic = "";
        [Range(0, 2)]
        public int compressionLevel = 0;
        [Tooltip("Keep building messages (data, OnSendMessage) while the host has no connection to the ROS side. " +
                 "Off: nothing is computed until the link is up.")]
        public bool useWithoutLink = false;


#if ROS_V2
        [Header("QOS")]
        public Qos qos;
#endif
        #endregion

        public bool Active { get; set; } = true;

        protected ProBridgeTx()
        {
#if ROS_V2
            // Default for new components; serialized values override it.
            qos = CreateDefaultQos();
#endif
        }

#if ROS_V2
        /// <summary>QoS of a newly added publisher. Override to give a publisher type its own default.</summary>
        protected virtual Qos CreateDefaultQos() => null;
#endif

        public T data { get; } = new T();

        public EventHandler<ProBridge.Msg> OnSendMessage { get; set; } = delegate { };

        private ProBridge Bridge { get { return ProBridgeServer.Instance?.Bridge; } }

        private long _lastSimTime = 0;
        private double _nextSendTime;
        private int _skippedCount;
        private float _nextSkippedReport;

        private bool sentHostMissingMsg;

        private void OnEnable()
        {
            if (Bridge == null)
            {
                enabled = false;
                Debug.LogWarning("Don't inited ROS bridge server.");
                return;
            }

            if (!manualSend)
            {
                if (sendRate > 0f && sendRate < Time.fixedDeltaTime)
                    Debug.LogWarning($"[{topic}] sendRate {sendRate}s is shorter than the physics step {Time.fixedDeltaTime}s: " +
                                    $"messages are sent once per step ({1f / Time.fixedDeltaTime:0} Hz). Set sendRate to 0 to send every step.", this);

                _nextSendTime = Time.fixedTimeAsDouble;
                ProBridgeServer.AddSimStepListener(this);
            }
            AfterEnable();
        }

        private void OnDisable()
        {
            ProBridgeServer.RemoveSimStepListener(this);
            AfterDisable();
        }

        void ISimStepListener.OnSimStep()
        {
            if (sendRate > 0f)
            {
                double now = Time.fixedTimeAsDouble;
                if (now + 1e-6 < _nextSendTime)
                    return;

                // Keep the average rate, but never queue up several sends when falling behind.
                _nextSendTime += sendRate;
                if (_nextSendTime <= now)
                    _nextSendTime = now + sendRate;
            }

            SendMsg();
        }

        protected void SendMsg()
        {
            if (!host)
            {
                if (!sentHostMissingMsg) Debug.LogWarning($"No host assigned for topic {topic}.");
                sentHostMissingMsg = true;
                return;
            }

            sentHostMissingMsg = false;

            bool linked = host.IsConnected;
            if (!linked && !useWithoutLink) return;

            // A message is stamped with SimTime, so two sends within one simulation step would carry the same stamp.
            // Scheduled sends happen once per step; this only triggers for extra manual SendMsg() calls.
            var st = ProBridgeServer.SimTime.Ticks;
            if (_lastSimTime >= st)
            {
                ReportSkipped();
                return;
            }
            _lastSimTime = st;

            if (!Active || topic == "") return;
            ProBridge.Msg msg;
            try
            {
                msg = GetMsg(ProBridgeServer.SimTime);
            }
            catch (Exception e)
            {
                Debug.LogWarning($"Failed to get message for {topic}, PC might be running slower than requested frequency." + e);
                return;
            }
            OnSendMessage?.Invoke(this, msg);
            if (linked && Bridge != null)
                Bridge.SendMsg(host.pushSocket, msg);
        }

        protected virtual ProBridge.Msg GetMsg(TimeSpan ts)
        {
            return new ProBridge.Msg()
            {
#if ROS_V2
                v = 2,
#else
                v = 1,
#endif
                n = topic,
                t = data.GetRosType(),
                c = compressionLevel,
#if ROS_V2
                q = qos,
#endif
                d = data
            };
        }

        private void ReportSkipped()
        {
            _skippedCount++;
            if (Time.realtimeSinceStartup < _nextSkippedReport)
                return;

            Debug.LogWarning($"[{topic}] Skipped {_skippedCount} message(s): SendMsg() was called again before SimTime " +
                             $"advanced (physics step {Time.fixedDeltaTime}s). Check extra SendMsg() calls.", this);
            _skippedCount = 0;
            _nextSkippedReport = Time.realtimeSinceStartup + SkippedReportInterval;
        }

        protected virtual void AfterEnable() { }
        protected virtual void AfterDisable() { }
    }
}
