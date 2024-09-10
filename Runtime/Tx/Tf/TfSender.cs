using System;
using ProBridge.Utils;
using System.Collections.Generic;
using System.Threading;
using geometry_msgs.msg;
using std_msgs;
using tf2_msgs.msg;
using UnityEngine;
using Quaternion = UnityEngine.Quaternion;

namespace ProBridge.Tx.Tf
{
    [AddComponentMenu("ProBridge/Tf/Sender")]
    public class TfSender : ProBridgeSingletone<TfSender>
    {
        #region Inspector

        public ProBridgeHost host;
        public float sendRate = 0.1f;
        [Range(0, 9)] public int compressionLevel = 0;
        public string dynamicTopic = "/tf";
        public string staticTopic = "/tf_static";
#if ROS_V2
        [Header("Dynamic QOS")] public Qos dynamicQos;
        [Header("Static QOS")] public Qos staticQos;
#endif

        #endregion

        public bool Active { get; set; } = true;

        private ProBridge Bridge;

        private long _lastSimTime = 0;
        private List<TfLink> _links = new List<TfLink>();
        private TransformStamped[] staticTransforms;

        private void Start()
        {
            Bridge = ProBridgeServer.Instance.Bridge;
            if (Bridge == null)
            {
                enabled = false;
                Debug.LogWarning("ROS bridge server not initialized.");
                return;
            }

            host.onSubscriberConnect += SendStaticMsg;
            UpdateTree();
            

            InvokeRepeating("UpdateTree", 0, 0.9f);
            InvokeRepeating("SendDynamicMsg", 1, sendRate);
        }

        private void OnDisable()
        {
            CancelInvoke("SendMsg");
        }

        protected void UpdateTree()
        {
            lock (_links)
            {
                _links.Clear();
                _links.AddRange(FindObjectsOfType<TfLink>());
            }
            staticTransforms = GetTransforms(true);
        }

        private TransformStamped[] GetTransforms(bool staticTransforms = false)
        {
            var list = new List<TransformStamped>();
            var stamp = ProBridgeServer.SimTime;
            lock (_links)
            {
                foreach (var link in _links)
                {
                    if (link is null || !link.isActiveAndEnabled || link.frame_id == "" ||
                        (link.is_static != staticTransforms))
                        continue;

                    foreach (var child in link.children)
                    {
                        if (child is null || !child.isActiveAndEnabled || child.frame_id == "")
                            continue;

                        var localPosition = link.transform.InverseTransformPoint(child.transform.position);
                        var localRotation = Quaternion.Inverse(link.transform.rotation) * child.transform.rotation;

                        var ts = new TransformStamped();
                        ts.header.stamp = stamp;
                        ts.header.frame_id = link.frame_id;
                        ts.child_frame_id = child.frame_id;
                        ts.transform.translation = localPosition.ToRos();
                        ts.transform.rotation = localRotation.ToRos();

                        list.Add(ts);
                    }
                }
            }

            return list.ToArray();
        }


        protected void SendStaticMsg(object obj, EventArgs args)
        {
            /*
             * At present, the only available event is ZMQ_EVENT_ACCEPTED. However, this event doesn't
             * guarantee that the subscriber is ready to receive messages. As a workaround, we're using a delay (sleep) here.
             * A more reliable solution would be to use ZMQ_EVENT_HANDSHAKE_SUCCEED, but this event is not yet available in NetMQ.
             */
            Thread.Sleep(300);
            SendMsg(true);
        }

        protected void SendDynamicMsg()
        {
            SendMsg();
        }

        protected void SendMsg(bool staticT = false)
        {
            var st = ProBridgeServer.SimTime.Ticks;
            if (_lastSimTime >= st && !staticT)
            {
                Debug.LogWarning("Can't send message before update SimTime.");
                return;
            }

            if(!staticT) _lastSimTime = st;

            if (Active && Bridge != null)
            {
                var data = new TFMessage();
                data.transforms = staticT ? staticTransforms : GetTransforms();

                var msg = new ProBridge.Msg()
                {
#if ROS_V2
                    v = 2,
#else
                    v = 1,
#endif
                    n = staticT? staticTopic : dynamicTopic,
                    t = (data as IRosMsg).GetRosType(),
                    c = compressionLevel,
#if ROS_V2
                    q = staticT ? staticQos.GetValue() : dynamicQos.GetValue(),
#endif
                    d = data
                };

                Bridge.SendMsg(host.publisher, msg);
            }
        }
    }
}