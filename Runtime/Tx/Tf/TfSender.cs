using System;
using System.Collections;
using ProBridge.Utils;
using System.Collections.Generic;
using geometry_msgs.msg;
using std_msgs;
using tf2_msgs.msg;
using UnityEngine;
using Quaternion = UnityEngine.Quaternion;

namespace ProBridge.Tx.Tf
{
    [AddComponentMenu("ProBridge/Tx/tf2_msgs/Sender")]
    public class TfSender : ProBridgeSingletone<TfSender>
    {
        #region Inspector

        public ProBridgeHost host;
        public float sendRate = 0.1f;
        [Range(0, 2)] public int compressionLevel = 0;
        public bool sendDynamic = true;
        public string dynamicTopic = "/tf";
        public bool sendStatic = true;
        public string staticTopic = "/tf_static";
#if ROS_V2
        [Header("Dynamic QOS")] public Qos dynamicQos;
        [Header("Static QOS")] public Qos staticQos;
#endif

        #endregion

        public bool Active { get; set; } = true;

        public ProBridge Bridge;

        private long _lastSimTime = 0;
        private List<TfLink> _links = new List<TfLink>();
        private TransformStamped[] staticTransforms;
        private bool sendStaticMsgRequest = false;

        public void CallRepeatingMethods()
        {
            InvokeRepeating("UpdateTree", 0, 0.9f);
            InvokeRepeating("SendDynamicMsg", 1, sendRate);
        }

        private void OnDisable()
        {
            CancelInvoke("SendMsg");
        }

        public void UpdateTree()
        {
            lock (_links)
            {
                _links.Clear();
                _links.AddRange(FindObjectsOfType<TfLink>());
            }

            staticTransforms = GetTransforms(true);
            if (sendStaticMsgRequest)
            {
                sendStaticMsgRequest = false;
                if(sendStatic) StartCoroutine(StaticSendingCoroutine());
            }
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


        public void SendStaticMsg(object obj, EventArgs args)
        {
            /*
             * At present, the only available event is ZMQ_EVENT_ACCEPTED. However, this event doesn't
             * guarantee that the subscriber is ready to receive messages. As a workaround, making a request to start a coroutine on the
             * main thread, this coroutine does regular checks until staticTransforms are ready.
             * A more reliable solution would be to use ZMQ_EVENT_HANDSHAKE_SUCCEED, but this event is not yet available in NetMQ.
             */
            sendStaticMsgRequest = true;
        }
        
        private IEnumerator StaticSendingCoroutine()
        {
            while(_lastSimTime >= ProBridgeServer.SimTime.Ticks)
            {
                yield return new WaitForSeconds(0.1f);
            }
            Debug.Log("Connected to a new subscriber. Sending static!");
            SendMsg(true);
        }

        protected void SendDynamicMsg()
        {
            if(sendDynamic) SendMsg();
        }

        protected void SendMsg(bool staticT = false)
        {
            var st = ProBridgeServer.SimTime.Ticks;
            if (_lastSimTime >= st && !staticT)
            {
                Debug.LogWarning("Can't send message before update SimTime.");
                return;
            }

            if (!staticT) _lastSimTime = st;

            if (Active && Bridge != null)
            {
                var data = new TFMessage();
                data.transforms = staticT ? staticTransforms : GetTransforms();
                if (data.transforms.Length == 0) return;

                var msg = new ProBridge.Msg()
                {
#if ROS_V2
                    v = 2,
#else
                    v = 1,
#endif
                    n = staticT ? staticTopic : dynamicTopic,
                    t = (data as IRosMsg).GetRosType(),
                    c = compressionLevel,
#if ROS_V2
                    q = staticT ? staticQos : dynamicQos,
#endif
                    d = data
                };

                Bridge.SendMsg(host.pushSocket, msg);
            }
        }
    }
}