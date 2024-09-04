using System;
using System.Collections;
using UnityEngine;

namespace ProBridge.Tx.Sensor
{
    [AddComponentMenu("ProBridge/Tx/Image")]
    public class Image : ProBridgeTxStamped<ROS.Msgs.Sensors.Image>
    {
        public RenderTexture renderTexture;

        private Texture2D tex;
        private WaitForEndOfFrame frameEnd = new WaitForEndOfFrame();

        protected override void OnStart()
        {
            InvokeRepeating("CallSnapShot", 0, sendRate);
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data.height = (uint)renderTexture.height;
            data.width = (uint)renderTexture.width;
            data.encoding = "rgb8";
            data.is_bigendian = false; 
            data.step = (uint)(3 * renderTexture.width); 

            if (tex != null)
            {
                data.data = tex.GetRawTextureData();
            }
            else
            {
                data.height = 1;
                data.width = 1;
                data.data = new byte[3];
            }

            return base.GetMsg(ts);
        }

        private void CallSnapShot()
        {
            StartCoroutine(TakeSnapshot());
        }

        public IEnumerator TakeSnapshot()
        {
            yield return frameEnd;

            RenderTexture.active = renderTexture;

            tex = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGB24, false);
            tex.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
            tex.Apply();

            RenderTexture.active = null;
        }

    }
}