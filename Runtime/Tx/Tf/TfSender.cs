using ProBridge.ROS.Msgs;
using ProBridge.Utils;
using System.Collections.Generic;
using UnityEngine;

namespace ProBridge.Tx.Tf
{
    [AddComponentMenu("ProBridge/Tf/Sender")]
    public class TfSender : ProBridgeSingletone<TfSender>
    {
        #region Inspector
        public ProBridgeHost host;
        public float sendRate = 0.1f;
        #endregion

        public bool Active { get; set; } = true;

        private ProBridge Bridge { get { return ProBridgeServer.Instance?.Bridge; } }

        private long _lastSimTime = 0;
        private List<TfLink> _links = new List<TfLink>();

        private void OnEnable()
        {
            if (Bridge == null)
            {
                enabled = false;
                Debug.LogWarning("Don't inited ROS bridge server.");
                return;
            }

            InvokeRepeating("UpdateTree", 0, 0.9f);
            InvokeRepeating("SendMsg", 1, sendRate);
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
        }

        private ROS.Msgs.Geometry.TransformStamped[] GetTransforms()
        {
            var list = new List<ROS.Msgs.Geometry.TransformStamped>();
            var stamp = ProBridgeServer.SimTime;
            lock (_links)
            {
                foreach (var link in _links)
                {
                    if (link is null || !link.isActiveAndEnabled || link.frame_id == "")
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

        protected void SendMsg()
        {
            var st = ProBridgeServer.SimTime.Ticks;
            if (_lastSimTime >= st)
            {
                Debug.LogWarning("Can't send message before update SimTime.");
                return;
            }
            _lastSimTime = st;

            if (Active && Bridge != null)
            {
                var data = new ROS.Msgs.Tf.tfMessage();
                data.transforms = GetTransforms();

                var msg = new ProBridge.Msg()
                {
                    n = "/tf",
                    t = (data as IRosMsg).GetRosType(),
#if ROS_V2
                    q = 10,
#endif
                    d = data
                };

                Bridge.SendMsg(host, msg);
            }
        }
    }
}