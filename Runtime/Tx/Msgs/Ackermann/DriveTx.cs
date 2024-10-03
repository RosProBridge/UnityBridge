using UnityEngine;

namespace ProBridge.Tx.Ackermann
{
    [AddComponentMenu("ProBridge/Tx/Ackermann/Drive")]
    public class DriveTx : ProBridgeTx<ackermann_msgs.msg.AckermannDrive>
    {
    }
}