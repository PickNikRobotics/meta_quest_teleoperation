using System.Collections;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit.AffordanceSystem.Receiver.Rendering;

/// <summary>
/// Keeps Meta passthrough enabled without rebuilding its compositor layer.
/// </summary>
[DefaultExecutionOrder(1000)]
public sealed class PassthroughBootstrap : MonoBehaviour
{
    private static PassthroughBootstrap _instance;
    private Coroutine _initializeRoutine;
    private Coroutine _affordanceGuardRoutine;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void Install()
    {
        if (_instance != null)
        {
            return;
        }

        var bootstrapObject = new GameObject(nameof(PassthroughBootstrap));
        DontDestroyOnLoad(bootstrapObject);
        _instance = bootstrapObject.AddComponent<PassthroughBootstrap>();
    }

    private void OnEnable()
    {
        OVRManager.HMDMounted += EnsurePassthrough;
        Application.focusChanged += OnApplicationFocusChanged;
        EnsurePassthrough();
        DisableBrokenAffordances();
        _affordanceGuardRoutine = StartCoroutine(GuardAffordances());
    }

    private void OnDisable()
    {
        OVRManager.HMDMounted -= EnsurePassthrough;
        Application.focusChanged -= OnApplicationFocusChanged;

        if (_affordanceGuardRoutine != null)
        {
            StopCoroutine(_affordanceGuardRoutine);
            _affordanceGuardRoutine = null;
        }
    }

    private void OnApplicationFocusChanged(bool hasFocus)
    {
        if (hasFocus)
        {
            EnsurePassthrough();
        }
    }

    private void EnsurePassthrough()
    {
        if (_initializeRoutine != null)
        {
            return;
        }

        _initializeRoutine = StartCoroutine(InitializeWhenReady());
    }

    private IEnumerator InitializeWhenReady()
    {
        const float timeoutSeconds = 15f;
        float deadline = Time.realtimeSinceStartup + timeoutSeconds;
        OVRManager manager = null;

        while (Time.realtimeSinceStartup < deadline)
        {
            manager = FindFirstObjectByType<OVRManager>();
            if (manager != null && OVRManager.OVRManagerinitialized)
            {
                break;
            }

            yield return null;
        }

        if (manager == null || !OVRManager.OVRManagerinitialized)
        {
            Debug.LogError("[PassthroughBootstrap] OVRManager did not initialize.");
            _initializeRoutine = null;
            yield break;
        }

        manager.isInsightPassthroughEnabled = true;
        ConfigureCamera();
        OVRPassthroughLayer layer = GetOrCreateLayer(manager);

        deadline = Time.realtimeSinceStartup + timeoutSeconds;
        while (Time.realtimeSinceStartup < deadline && !OVRManager.IsInsightPassthroughInitialized())
        {
            yield return null;
        }

        if (!OVRManager.IsInsightPassthroughInitialized())
        {
            Debug.LogError("[PassthroughBootstrap] Insight Passthrough initialization timed out.");
            _initializeRoutine = null;
            yield break;
        }

        Debug.Log(
            $"[PassthroughBootstrap] Ready. initialized={OVRManager.IsInsightPassthroughInitialized()}, " +
            $"userPresent={OVRPlugin.userPresent}, layer={layer.name}, enabled={layer.enabled}");

        _initializeRoutine = null;
    }

    private static OVRPassthroughLayer GetOrCreateLayer(OVRManager manager)
    {
        OVRPassthroughLayer layer = FindFirstObjectByType<OVRPassthroughLayer>();
        if (layer == null)
        {
            layer = manager.gameObject.AddComponent<OVRPassthroughLayer>();
        }

#pragma warning disable CS0618
        layer.projectionSurfaceType = OVRPassthroughLayer.ProjectionSurfaceType.Reconstructed;
        layer.overlayType = OVROverlay.OverlayType.Underlay;
#pragma warning restore CS0618
        layer.hidden = false;
        layer.textureOpacity = 1f;
        layer.enabled = true;
        return layer;
    }

    private static void ConfigureCamera()
    {
        Camera mainCamera = Camera.main;
        if (mainCamera == null)
        {
            Debug.LogWarning("[PassthroughBootstrap] Main Camera was not found.");
            return;
        }

        mainCamera.clearFlags = CameraClearFlags.SolidColor;
        Color background = mainCamera.backgroundColor;
        background.a = 0f;
        mainCamera.backgroundColor = background;
    }

    private static IEnumerator GuardAffordances()
    {
        var interval = new WaitForSecondsRealtime(1f);
        while (true)
        {
            yield return interval;
            DisableBrokenAffordances();
        }
    }

#pragma warning disable CS0618
    private static void DisableBrokenAffordances()
    {
        ColorMaterialPropertyAffordanceReceiver[] receivers =
            FindObjectsByType<ColorMaterialPropertyAffordanceReceiver>(
                FindObjectsInactive.Exclude,
                FindObjectsSortMode.None);

        foreach (ColorMaterialPropertyAffordanceReceiver receiver in receivers)
        {
            var helper = receiver.materialPropertyBlockHelper;
            Renderer renderer = helper != null ? helper.rendererTarget : null;
            Material[] materials = renderer != null ? renderer.sharedMaterials : null;
            bool valid = materials != null &&
                         helper.materialIndex >= 0 &&
                         helper.materialIndex < materials.Length &&
                         materials[helper.materialIndex] != null;

            if (valid)
            {
                continue;
            }

            receiver.enabled = false;
            if (helper != null)
            {
                helper.enabled = false;
            }
        }
    }
#pragma warning restore CS0618
}
