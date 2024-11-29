using System;
using System.Collections;
using ProBridge.Utils;
using System.Collections.Generic;
using geometry_msgs.msg;
using std_msgs;
using tf2_msgs.msg;
using UnityEngine;
using Quaternion = UnityEngine.Quaternion;
using Newtonsoft.Json.Linq;

namespace ProBridge.Tx.Tf
{
    [AddComponentMenu("ProBridge/Tx/tf2_msgs/Sender")]
    public class TfSender : ProBridgeSingletone<TfSender>
    {
        #region Inspector

        public ProBridgeHost host;
        public float sendRate = 0.1f;
        [Range(0, 2)] public int compressionLevel = 0;
        public string dynamicTopic = "/tf";
        public string staticTopic = "/tf_static";
#if ROS_V2
        private Qos __dynamicQos = new Qos(10L);
        private Qos __staticQos = new Qos(Qos.GetLatchQos());
#endif

        #endregion

        public bool Active { get; set; } = true;

        public ProBridge Bridge;

        private long _lastSimTime = 0;
        private List<TfLink> _links = new List<TfLink>();
        private TransformStamped[] __staticTransforms;

        private bool _needUpdateStaticMsgs = false;

        public void CallRepeatingMethods()
        {
            InvokeRepeating("UpdateStaticMsgs", 1, 1);
            InvokeRepeating("SendDynamicMsg", 1, sendRate);
        }

        private void OnDisable()
        {
            CancelInvoke("SendDynamicMsg");
        }

        public void LinkAdd(TfLink value)
        {
            lock (_links)
            {
                _links.Add(value);

                if (value.is_static)
                    _needUpdateStaticMsgs = true;
            }
        }

        public void LinkRemove(TfLink value)
        {
            lock (_links)
            {
                _links.Remove(value);
                if (value.is_static)
                    _needUpdateStaticMsgs = true;
            }
        }


        public void SendStaticMsg(object obj, EventArgs args)
        {
            /*
             * At present, the only available event is ZMQ_EVENT_ACCEPTED. However, this event doesn't
             * guarantee that the subscriber is ready to receive messages. As a workaround, making a request to start a coroutine on the
             * main thread, this coroutine does regular checks until staticTransforms are ready.
             * A more reliable solution would be to use ZMQ_EVENT_HANDSHAKE_SUCCEED, but this event is not yet available in NetMQ.
             */
            _needUpdateStaticMsgs = true;
        }

        private void UpdateStaticMsgs()
        {
            if (_needUpdateStaticMsgs)
            {
                _needUpdateStaticMsgs = false;
                __staticTransforms = GetTransforms(true);
                StartCoroutine(StaticSendingCoroutine());
            }
        }

        private IEnumerator StaticSendingCoroutine()
        {
            while (_lastSimTime >= ProBridgeServer.SimTime.Ticks)
            {
                yield return new WaitForSeconds(0.1f);
            }
            Debug.Log("Connected to a new subscriber. Sending static!");
            SendMsg(true);
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

            if (!staticT) _lastSimTime = st;

            if (Active && Bridge != null)
            {
                var data = new TFMessage();
                data.transforms = staticT ? __staticTransforms : GetTransforms();

                if (data.transforms.Length == 0 && staticT == false)
                    return;

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
                    q = staticT ? __staticQos : __dynamicQos,
#endif
                    d = data
                };

                Bridge.SendMsg(host.pushSocket, msg);
            }
        }
    }
}