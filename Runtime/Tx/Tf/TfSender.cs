using System;
using ProBridge.ROS.Msgs;
using ProBridge.Utils;
using System.Collections.Generic;
using System.Threading;
using ProBridge.ROS.Msgs.TF2;
using UnityEngine;

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
        private ROS.Msgs.Geometry.TransformStamped[] staticTransforms;

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

        private ROS.Msgs.Geometry.TransformStamped[] GetTransforms(bool staticTransforms = false)
        {
            var list = new List<ROS.Msgs.Geometry.TransformStamped>();
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

                        var ts = new ROS.Msgs.Geometry.TransformStamped();
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