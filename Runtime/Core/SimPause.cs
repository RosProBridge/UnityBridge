using System;
using UnityEngine;

namespace ProBridge
{
    /// <summary>
    /// Pause of the whole simulation, shared by the ROS service (SimPauseService) and the UI.
    /// Pause: Time.timeScale = 0 (physics, animations, simulation time / clock and publishers stop) and audio is paused.
    /// The editor Pause button is not pressed (scripts and UI would stop, e.g. a "resume" button), but pressing it
    /// by hand counts as a pause, and Resume() releases it. Incoming bridge messages are still handled while paused
    /// (see ProBridgeServer).
    /// </summary>
    public static class SimPause
    {
        // Time scale restored on resume (keeps a slow-motion setting).
        private static float _resumeTimeScale = 1f;
        private static bool _lastReported;

        /// <summary>Raised on the main thread when the simulation is paused (true) or resumed (false).</summary>
        public static event Action<bool> Changed;

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

        public static void Pause()
        {
            if (Time.timeScale > 0f)
                _resumeTimeScale = Time.timeScale;
            Time.timeScale = 0f;
            AudioListener.pause = true;
            ReportChange();
        }

        public static void Resume()
        {
            if (Time.timeScale == 0f)
                Time.timeScale = _resumeTimeScale > 0f ? _resumeTimeScale : 1f;
            AudioListener.pause = false;
#if UNITY_EDITOR
            UnityEditor.EditorApplication.isPaused = false;
#endif
            ReportChange();
        }

        public static void SetPaused(bool paused)
        {
            if (paused)
                Pause();
            else
                Resume();
        }

        private static void ReportChange()
        {
            bool paused = IsPaused;
            if (paused == _lastReported)
                return;
            _lastReported = paused;
            Changed?.Invoke(paused);
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void ResetOnPlay()
        {
            _resumeTimeScale = 1f;
            _lastReported = false;
            Changed = null;
#if UNITY_EDITOR
            UnityEditor.EditorApplication.pauseStateChanged -= OnEditorPauseStateChanged;
            UnityEditor.EditorApplication.pauseStateChanged += OnEditorPauseStateChanged;
#endif
        }

#if UNITY_EDITOR
        // Pressing the editor Pause button is a pause too; releasing it also undoes the time scale and audio pause.
        private static void OnEditorPauseStateChanged(UnityEditor.PauseState state)
        {
            if (!Application.isPlaying)
                return;

            if (state == UnityEditor.PauseState.Unpaused && (Time.timeScale == 0f || AudioListener.pause))
                Resume();
            else
                ReportChange();
        }
#endif
    }
}
