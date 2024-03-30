using UnityEngine;

namespace ProBridge
{
    public class PoseFromLine : MonoBehaviour
    {
        public Transform targetPointfront;
        public Transform targetPointback;
        public Rigidbody body;
        public Spline.SplineLine line;
        private float distance_front;
        private float distance_back;
        private float angle;
        private float line_angle;
        // Start is called before the first frame update
        void Start()
        {

        }
        public float distance_front_ { get { return distance_front; } }
        public float distance_back_ { get { return distance_back; } }
        public float angle_ { get { return angle; } }
        // Update is called once per frame
        void Update()
        {

            distance_front = FindDistToLine(targetPointfront);
            distance_back = FindDistToLine(targetPointback);
            Debug.Log(distance_front);
            Debug.Log(distance_back);

        }
        private float FindDistToLine(Transform targetPoint)
        {
            var g_target_pos = targetPoint.position;
            float min_dis = float.PositiveInfinity;
            int min_idx = -1;
            var bm_angle = body.rotation.eulerAngles.y;
            float distance = 0;
            for (int i = 0; i < line.Points.Length; i++)
            {
                var dist = Vector3.Distance(line.transform.TransformPoint(line.Points[i].point), g_target_pos);
                if (dist < min_dis)
                {
                    min_dis = dist;
                    min_idx = i;
                }
                else
                    break;
            }
            if (min_idx > 0 && min_idx < line.Points.Length)
            {
                Vector3 g_start = line.transform.TransformPoint(line.Points[min_idx - 1].point);
                Vector3 g_mid = line.transform.TransformPoint(line.Points[min_idx].point);
                Vector3 g_end = line.transform.TransformPoint(line.Points[min_idx + 1].point);

                Vector3 ProjPt1 = ProjectPointOnSegment(g_target_pos, g_start, g_mid);
                Vector3 ProjPt2 = ProjectPointOnSegment(g_target_pos, g_mid, g_end);


                if (is_between(ProjPt1, g_start, g_mid))
                {
                    Debug.DrawLine(g_target_pos, ProjPt1, Color.red);
                    distance = Vector3.Distance(g_target_pos, ProjPt1);

                    var g_v = g_mid - g_start;

                    line_angle = Vector3.Angle(Vector3.forward, g_v);

                }
                else if (is_between(ProjPt2, g_mid, g_end))
                {
                    Debug.DrawLine(g_target_pos, ProjPt2, Color.red);
                    distance = Vector3.Distance(g_target_pos, ProjPt2);
                    var g_v = g_end - g_mid;
                    line_angle = Vector3.Angle(Vector3.forward, g_v);

                }
                else
                {
                    float disti = Vector3.Distance(g_start, g_target_pos);
                    float disti_ = Vector3.Distance(g_mid, g_target_pos);
                    float disti__ = Vector3.Distance(g_end, g_target_pos);
                    if ((disti < disti_) && (disti < disti__))
                    {
                        Debug.DrawLine(g_target_pos, g_start, Color.blue);
                        distance = Vector3.Distance(g_target_pos, g_start);
                        var g_v = g_start - g_mid;
                        line_angle = Vector3.Angle(Vector3.forward, g_v);

                    }
                    if ((disti_ < disti) && (disti_ < disti__))
                    {
                        Debug.DrawLine(g_target_pos, g_mid, Color.blue);
                        distance = Vector3.Distance(g_target_pos, g_mid);
                        var g_v = g_start - g_mid;
                        line_angle = Vector3.Angle(Vector3.forward, g_v);


                    }
                    if ((disti__ < disti) && (disti__ < disti_))
                    {
                        Debug.DrawLine(g_target_pos, g_end, Color.blue);
                        distance = Vector3.Distance(g_target_pos, g_end);
                        var g_v = g_mid - g_end;
                        line_angle = Vector3.Angle(Vector3.forward, g_v);

                    }
                }
            }
            angle = bm_angle - line_angle;
            angle = normalize_angle(angle);
            if ((angle > 180) || (angle < -180))
            { Debug.Log(angle); }
            return distance;
        }

        private Vector3 ProjectPointOnSegment(Vector3 P, Vector3 w0, Vector3 w1)
        {
            Vector3 projectedPoint = Vector3.Project((P - w0), (w1 - w0)) + w0;
            return projectedPoint;
        }

        private bool is_between(Vector3 P, Vector3 w0, Vector3 w1)
        {
            double epsilon = 0.000001;
            return ((Vector3.Distance(w0, P) + Vector3.Distance(P, w1) - Vector3.Distance(w0, w1)) < epsilon);
        }
        private float normalize_angle(float angle_deg)
        {
            float angle_norm = (angle_deg + 180) % (2 * 180) - 180;
            return angle_norm;
        }

    }
}