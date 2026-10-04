using System;
using std_srvs.srv;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace ProBridge.Srv.Std
{
    /// <summary>
    /// std_srvs/Trigger: reloads the active scene. The response is sent first, the scene is reloaded on the next frame.
    /// A project can do the reload itself (e.g. through its loading screen) by setting <see cref="ReloadOverride"/>.
    /// </summary>
    [AddComponentMenu("ProBridge/Srv/std_srvs/Scene Reload")]
    public class SceneReloadService : ProBridgeService<Trigger_Request, Trigger_Response>
    {
        /// <summary>
        /// Reloads the given (active) scene instead of the default immediate reload, e.g. through a loading screen.
        /// Called on the main thread. Null: SceneManager reload.
        /// </summary>
        public static Action<Scene> ReloadOverride;

        private bool _reloadPending;

        protected override Trigger_Response OnRequest(Trigger_Request request)
        {
            var scene = SceneManager.GetActiveScene();
            if (_reloadPending)
                return new Trigger_Response { success = false, message = $"Scene {scene.name} is already reloading" };

            _reloadPending = true;
            return new Trigger_Response { success = true, message = $"Reloading scene {scene.name}" };
        }

        private void Update()
        {
            if (!_reloadPending)
                return;
            _reloadPending = false;

            var scene = SceneManager.GetActiveScene();
            Debug.Log($"[{service}] Reloading scene {scene.name}", this);
            if (ReloadOverride != null)
            {
                ReloadOverride(scene);
                return;
            }
#if UNITY_EDITOR
            // In the editor a scene missing from Build Settings can only be loaded by path.
            if (scene.buildIndex < 0)
            {
                UnityEditor.SceneManagement.EditorSceneManager.LoadSceneInPlayMode(
                    scene.path, new LoadSceneParameters(LoadSceneMode.Single));
                return;
            }
#endif
            SceneManager.LoadScene(scene.buildIndex);
        }
    }
}
