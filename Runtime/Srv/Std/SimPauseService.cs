using std_srvs.srv;
using UnityEngine;

namespace ProBridge.Srv.Std
{
    /// <summary>
    /// std_srvs/SetBool: data = true pauses the simulation, false resumes it (see <see cref="SimPause"/>).
    /// </summary>
    [AddComponentMenu("ProBridge/Srv/std_srvs/Sim Pause")]
    public class SimPauseService : ProBridgeService<SetBool_Request, SetBool_Response>
    {
        public static bool IsPaused => SimPause.IsPaused;

        protected override SetBool_Response OnRequest(SetBool_Request request)
        {
            if (request.data == SimPause.IsPaused)
                return new SetBool_Response { success = true, message = SimPause.IsPaused ? "Already paused" : "Already running" };

            SimPause.SetPaused(request.data);

            Debug.Log($"[{service}] Simulation {(SimPause.IsPaused ? "paused" : "resumed")}", this);
            return new SetBool_Response { success = true, message = SimPause.IsPaused ? "Paused" : "Resumed" };
        }
    }
}
