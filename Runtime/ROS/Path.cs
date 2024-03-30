using UnityEngine;

using System;

using ProBridge.ROS;

namespace ProBridge
{
    [RequireComponent(typeof(Spline.SplineLine))]
    public class Path : ProBridgeBehaviour<Path>
    {
        public string frameId;

        //[Header("Params")]
        //public Spline.SplineLine line;

        private ROS.Msgs.Geometry.PoseStamped[] poses_;
        public Spline.SplineLine line { get; private set; }

        protected override void OnStart()
        {
            line = GetComponent<Spline.SplineLine>();
            poses_ = new ROS.Msgs.Geometry.PoseStamped[line is null ? 0 : line.Points.Length];
            for (int i = 0; i < line.Points.Length; i++)
            {
                var g_pose = line.transform.TransformPoint(line.Points[i].point);
                poses_[i] = new ROS.Msgs.Geometry.PoseStamped();
                poses_[i].pose.position = g_pose.ToRos();
            }
        }

        protected override void OnStop()
        {
        }
        private void Update()
        {
        }
        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            var data = new ROS.Msgs.Nav.Path
            {
                header = new ROS.Msgs.Std.Header()
                {
                    frame_id = frameId,
                    stamp = ts
                },
                poses = new ROS.Msgs.Geometry.PoseStamped[line is null ? 0 : line.Points.Length],
            };

            data.poses = poses_;

            for (int i = 0; i < line.Points.Length; i++)
            {
                data.poses[i].header.frame_id = frameId;
                data.poses[i].header.stamp = ts;
            }

            return new ProBridge.Msg()
            {
                n = topic,
                t = "nav_msgs.msg.Path",
                q = qos,
                d = data
            };
        }
    }
}