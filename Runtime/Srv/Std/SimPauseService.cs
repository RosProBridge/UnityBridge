using std_srvs.srv;
using UnityEngine;

namespace ProBridge.Srv.Std
{
    /// <summary>
    /// std_srvs/SetBool: data = true pauses the simulation, false resumes it.
    /// Pause: Time.timeScale = 0 (physics, simulation time / clock and publishers stop) and audio is paused;
    /// in the editor the Pause button is pressed as well. Incoming messages are still handled.
    /// Releasing the editor Pause button by hand resumes the simulation too.
    /// </summary>
    [AddComponentMenu("ProBridge/Srv/std_srvs/Sim Pause")]
    public class SimPauseService : ProBridgeService<SetBool_Request, SetBool_Response>
    {
        // Time scale restored on resume (keeps a slow-motion setting).
        private float _resumeTimeScale = 1f;

        public static bool IsPaused
        {
            get
            {
#if UNITY_EDITOR
                if (UnityEditor.EditorApplication.isPaused)
                    return true;
#endif
                return Time.timeScale == 0f;
            }
        }

        protected override void OnEnable()
        {
            base.OnEnable();
#if UNITY_EDITOR
            UnityEditor.EditorApplication.pauseStateChanged += OnEditorPauseStateChanged;
#endif
        }

        protected override void OnDisable()
        {
#if UNITY_EDITOR
            UnityEditor.EditorApplication.pauseStateChanged -= OnEditorPauseStateChanged;
#endif
            base.OnDisable();
        }

        protected override SetBool_Response OnRequest(SetBool_Request request)
        {
            if (request.data == IsPaused)
                return new SetBool_Response { success = true, message = IsPaused ? "Already paused" : "Already running" };

            if (request.data)
                Pause();
            else
                Resume();

            Debug.Log($"[{service}] Simulation {(IsPaused ? "paused" : "resumed")}", this);
            return new SetBool_Response { success = true, message = IsPaused ? "Paused" : "Resumed" };
        }

        private void Pause()
        {
            if (Time.timeScale > 0f)
                _resumeTimeScale = Time.timeScale;
            Time.timeScale = 0f;
            AudioListener.pause = true;
#if UNITY_EDITOR
            UnityEditor.EditorApplication.isPaused = true;
#endif
        }

        private void Resume()
        {
            if (Time.timeScale == 0f)
                Time.timeScale = _resumeTimeScale > 0f ? _resumeTimeScale : 1f;
            AudioListener.pause = false;
#if UNITY_EDITOR
            UnityEditor.EditorApplication.isPaused = false;
#endif
        }

#if UNITY_EDITOR
        // A pause by the editor button needs nothing more (the whole loop stops, IsPaused reports it).
        // Releasing the button must also undo the time scale and audio pause set by the service.
        private void OnEditorPauseStateChanged(UnityEditor.PauseState state)
        {
            if (state == UnityEditor.PauseState.Unpaused && (Time.timeScale == 0f || AudioListener.pause))
                Resume();
        }
#endif
    }
}
