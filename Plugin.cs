using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using BepInEx;
using BepInEx.Configuration;
using BepInEx.Logging;
using HarmonyLib;
using UnityEngine;
using UnityEngine.SceneManagement;
using NuclearOption.SavedMission;


namespace AircraftWaypoints
{
    // ========== DATA MODEL ==========

    public enum RouteMode { Loop, OneShot, PingPong }

    [Serializable]
    public class Waypoint
    {
        public float x;
        public float y; // altitude (GlobalPosition y)
        public float z;
        public float speed; // m/s, 0 = no override

        public GlobalPosition ToGlobalPosition() => new GlobalPosition(x, y, z);
    }

    [Serializable]
    public class WaypointRoute
    {
        public string name = "New Route";
        public int mode; // RouteMode as int for JsonUtility
        public bool aglMode; // true = altitude is Above Ground Level (terrain-relative)
        public bool rawFollow; // true = ignore terrain avoidance, follow path exactly
        public List<Waypoint> waypoints = new List<Waypoint>();

        public RouteMode Mode
        {
            get => (RouteMode)mode;
            set => mode = (int)value;
        }
    }

    [Serializable]
    public class FormationOffset
    {
        public float right;   // meters right of route heading (negative = left)
        public float up;      // meters above route altitude
        public float forward; // meters ahead (+) or behind (-) along route heading
    }

    [Serializable]
    public class AircraftAssignment
    {
        public string uniqueName; // SavedAircraft.UniqueName
        public int routeIndex;    // Index into routeLibrary
        public float offsetRight;
        public float offsetUp;
        public float offsetForward;
    }

    // ========== CAMERA DATA ==========

    [Serializable]
    public class CameraWaypoint
    {
        public float x, y, z;       // global position
        public float rx, ry, rz;    // euler rotation
        public float fov;           // 0 = keep current
    }

    [Serializable]
    public class CameraRoute
    {
        public string name = "Camera Route";
        public int mode; // RouteMode as int
        public float speed = 50f; // m/s along path
        public bool lookAtTarget; // always face followed unit
        public bool autoSpeed; // match speed to followed unit
        public List<CameraWaypoint> waypoints = new List<CameraWaypoint>();

        public RouteMode Mode
        {
            get => (RouteMode)mode;
            set => mode = (int)value;
        }
    }

    [Serializable]
    public class MissionWaypointData
    {
        public List<WaypointRoute> routes = new List<WaypointRoute>();
        public List<AircraftAssignment> assignments = new List<AircraftAssignment>();
        public List<CameraRoute> cameraRoutes = new List<CameraRoute>();
    }

    public class AircraftWaypointState
    {
        public Aircraft aircraft;
        public WaypointRoute route;
        public FormationOffset offset;
        public int currentWaypointIndex;
        public bool pingPongReverse;
        public bool active = true;

    }

    // ========== WAYPOINT MANAGER ==========

    public static class WaypointManager
    {
        // Editor: assignments by UniqueName
        public static Dictionary<string, WaypointRoute> assignmentsByName = new Dictionary<string, WaypointRoute>();
        public static Dictionary<string, FormationOffset> formationOffsets = new Dictionary<string, FormationOffset>();

        // Runtime: tracked aircraft by InstanceID
        public static Dictionary<int, AircraftWaypointState> trackedAircraft = new Dictionary<int, AircraftWaypointState>();

        // Route library
        public static List<WaypointRoute> routeLibrary = new List<WaypointRoute>();

        // UI state
        public static string selectedUniqueName;
        public static int selectedRouteIndex = -1;

        public static void ClearRuntime()
        {
            trackedAircraft.Clear();
        }

        public static void ClearAll()
        {
            trackedAircraft.Clear();
            formationOffsets.Clear();
            selectedUniqueName = null;
        }

        public static void AssignRouteByName(string uniqueName, WaypointRoute route, FormationOffset offset = null)
        {
            if (string.IsNullOrEmpty(uniqueName) || route == null) return;
            assignmentsByName[uniqueName] = route;
            if (offset != null)
                formationOffsets[uniqueName] = offset;
            else
                formationOffsets.Remove(uniqueName);
        }

        public static void UnassignByName(string uniqueName)
        {
            assignmentsByName.Remove(uniqueName);
            formationOffsets.Remove(uniqueName);
        }

        public static void AssignToAircraft(Aircraft aircraft, WaypointRoute route, FormationOffset offset = null)
        {
            int id = aircraft.GetInstanceID();
            if (trackedAircraft.TryGetValue(id, out var state))
            {
                state.route = route;
                state.offset = offset;
                state.currentWaypointIndex = 0;
                state.pingPongReverse = false;
                state.active = true;
            }
            else
            {
                trackedAircraft[id] = new AircraftWaypointState
                {
                    aircraft = aircraft,
                    route = route,
                    offset = offset,
                    currentWaypointIndex = 0,
                    active = true
                };
            }
        }

        public static AircraftWaypointState GetState(Aircraft aircraft)
        {
            if (aircraft == null) return null;
            trackedAircraft.TryGetValue(aircraft.GetInstanceID(), out var state);
            return state;
        }

        /// <summary>Raycast terrain height at a local XZ position. Returns 0 (sea level) on miss.</summary>
        public static float GetGroundHeight(Vector3 localPos)
        {
            RaycastHit hit;
            Vector3 rayOrigin = new Vector3(localPos.x, 10000f, localPos.z);
            if (Physics.Raycast(rayOrigin, Vector3.down, out hit, 20000f))
                return hit.point.y;
            return 0f;
        }

        public static bool TryGetWaypointDestination(Aircraft aircraft, out GlobalPosition destination, out float targetHeight, out float throttle, out bool aglMode, out bool rawFollow)
        {
            destination = default;
            targetHeight = 300f;
            throttle = 1f;
            aglMode = false;
            rawFollow = false;

            int id = aircraft.GetInstanceID();
            if (!trackedAircraft.TryGetValue(id, out var state) || state.route == null || state.route.waypoints.Count == 0 || !state.active)
                return false;

            aglMode = state.route.aglMode;
            rawFollow = state.route.rawFollow;
            var wp = state.route.waypoints[state.currentWaypointIndex];
            destination = wp.ToGlobalPosition();

            // AGL mode: wp.y is height above ground, convert to absolute
            if (aglMode)
            {
                Vector3 localDest = destination.ToLocalPosition();
                float groundH = GetGroundHeight(localDest);
                localDest.y = groundH + wp.y;
                destination = localDest.ToGlobalPosition();
            }

            float dist = FastMath.Distance(destination, aircraft.GlobalPosition());

            // Dynamic arrival radius: scale with aircraft speed for smoother turns
            float baseRadius = Plugin.Instance != null ? Plugin.Instance.arrivalRadius.Value : 500f;
            float speedFactor = Mathf.Clamp(aircraft.speed / 150f, 1f, 4f); // faster = wider turn
            float arrivalRadius = baseRadius * speedFactor;

            if (dist < arrivalRadius)
            {
                AdvanceWaypoint(state);
                if (!state.active) return false;
                wp = state.route.waypoints[state.currentWaypointIndex];
                destination = wp.ToGlobalPosition();
                if (aglMode)
                {
                    Vector3 ld = destination.ToLocalPosition();
                    ld.y = GetGroundHeight(ld) + wp.y;
                    destination = ld.ToGlobalPosition();
                }
                dist = FastMath.Distance(destination, aircraft.GlobalPosition());
            }

            // Look-ahead blending: as we approach current WP, blend toward next WP
            int nextIdx = GetNextWaypointIndex(state);
            if (nextIdx >= 0 && dist < arrivalRadius * 2f)
            {
                var nextWp = state.route.waypoints[nextIdx];
                GlobalPosition nextDest = nextWp.ToGlobalPosition();
                if (aglMode)
                {
                    Vector3 ld = nextDest.ToLocalPosition();
                    ld.y = GetGroundHeight(ld) + nextWp.y;
                    nextDest = ld.ToGlobalPosition();
                }

                float blend = Mathf.Clamp01(dist / (arrivalRadius * 2f));
                Vector3 currentDir = destination.ToLocalPosition();
                Vector3 nextDir = nextDest.ToLocalPosition();
                Vector3 blended = Vector3.Lerp(nextDir, currentDir, blend);

                // In AGL mode, also adjust blended point altitude to terrain
                if (aglMode)
                {
                    float groundH = GetGroundHeight(blended);
                    float blendedAgl = Mathf.Lerp(nextWp.y, wp.y, blend);
                    blended.y = groundH + blendedAgl;
                }

                destination = blended.ToGlobalPosition();
            }

            // Apply formation offset relative to route heading
            if (state.offset != null && (state.offset.right != 0 || state.offset.up != 0 || state.offset.forward != 0))
            {
                Vector3 heading = Vector3.forward;
                int headingNext = GetNextWaypointIndex(state);
                if (headingNext >= 0)
                {
                    var nextWp = state.route.waypoints[headingNext];
                    heading = (nextWp.ToGlobalPosition().ToLocalPosition() - wp.ToGlobalPosition().ToLocalPosition());
                }
                else if (state.currentWaypointIndex > 0)
                {
                    var prevWp = state.route.waypoints[state.currentWaypointIndex - 1];
                    heading = (wp.ToGlobalPosition().ToLocalPosition() - prevWp.ToGlobalPosition().ToLocalPosition());
                }
                heading.y = 0;
                if (heading.sqrMagnitude > 0.01f) heading.Normalize();
                else heading = Vector3.forward;

                Vector3 right = Vector3.Cross(Vector3.up, heading).normalized;
                Vector3 localDest = destination.ToLocalPosition();
                localDest += right * state.offset.right;
                localDest.y += state.offset.up;
                localDest += heading * state.offset.forward;
                destination = localDest.ToGlobalPosition();
            }

            // targetHeight: in AGL mode use the AGL value directly, otherwise use absolute
            if (aglMode)
            {
                float aglHeight = wp.y + (state.offset?.up ?? 0f);
                targetHeight = Mathf.Max(aglHeight, 5f);
            }
            else
            {
                targetHeight = Mathf.Clamp(wp.y + (state.offset?.up ?? 0f), 20f, 5000f);
            }

            // Base throttle from per-waypoint speed limit
            if (wp.speed > 0 && aircraft.speed > wp.speed * 1.1f)
                throttle = 0.1f;
            else if (wp.speed > 0 && aircraft.speed > wp.speed * 0.95f)
                throttle = 0.5f;
            else
                throttle = 1f;

            // Auto-decelerate for upcoming turns
            int brakeNextIdx = GetNextWaypointIndex(state);
            if (brakeNextIdx >= 0)
            {
                Vector3 acLocal = aircraft.GlobalPosition().ToLocalPosition();
                Vector3 wpLocal = destination.ToLocalPosition();
                var brakeNextWp = state.route.waypoints[brakeNextIdx];
                Vector3 nextLocal = brakeNextWp.ToGlobalPosition().ToLocalPosition();
                if (aglMode)
                {
                    wpLocal.y = GetGroundHeight(wpLocal) + wp.y;
                    nextLocal.y = GetGroundHeight(nextLocal) + brakeNextWp.y;
                }

                Vector3 dirToNext = (nextLocal - wpLocal);
                dirToNext.y = 0;
                Vector3 dirToCurrent = (wpLocal - acLocal);
                dirToCurrent.y = 0;

                if (dirToNext.sqrMagnitude > 1f && dirToCurrent.sqrMagnitude > 1f)
                {
                    float turnAngle = Vector3.Angle(dirToCurrent.normalized, dirToNext.normalized);
                    float distToWp = dirToCurrent.magnitude;
                    float brakeRadius = Plugin.Instance != null ? Plugin.Instance.arrivalRadius.Value : 500f;
                    float brakeZone = brakeRadius * 3f;

                    if (turnAngle > 30f && distToWp < brakeZone)
                    {
                        float turnFactor = Mathf.Clamp01((turnAngle - 30f) / 120f);
                        float proximityFactor = 1f - Mathf.Clamp01(distToWp / brakeZone);
                        float brakeFactor = turnFactor * proximityFactor;
                        float turnThrottle = Mathf.Lerp(1f, 0.2f, brakeFactor);
                        throttle = Mathf.Min(throttle, turnThrottle);
                    }
                }
            }

            return true;
        }

        /// <summary>Get the index of the next waypoint (after current), or -1 if none.</summary>
        private static int GetNextWaypointIndex(AircraftWaypointState state)
        {
            int count = state.route.waypoints.Count;
            if (count < 2) return -1;

            switch (state.route.Mode)
            {
                case RouteMode.Loop:
                    return (state.currentWaypointIndex + 1) % count;
                case RouteMode.OneShot:
                    int next = state.currentWaypointIndex + 1;
                    return next < count ? next : -1;
                case RouteMode.PingPong:
                    if (!state.pingPongReverse)
                        return state.currentWaypointIndex < count - 1 ? state.currentWaypointIndex + 1 : state.currentWaypointIndex - 1;
                    else
                        return state.currentWaypointIndex > 0 ? state.currentWaypointIndex - 1 : state.currentWaypointIndex + 1;
                default:
                    return -1;
            }
        }

        private static void AdvanceWaypoint(AircraftWaypointState state)
        {
            int count = state.route.waypoints.Count;
            switch (state.route.Mode)
            {
                case RouteMode.Loop:
                    state.currentWaypointIndex = (state.currentWaypointIndex + 1) % count;
                    break;
                case RouteMode.OneShot:
                    if (state.currentWaypointIndex < count - 1)
                        state.currentWaypointIndex++;
                    else
                        state.active = false;
                    break;
                case RouteMode.PingPong:
                    if (!state.pingPongReverse)
                    {
                        if (state.currentWaypointIndex < count - 1)
                            state.currentWaypointIndex++;
                        else { state.pingPongReverse = true; state.currentWaypointIndex--; }
                    }
                    else
                    {
                        if (state.currentWaypointIndex > 0)
                            state.currentWaypointIndex--;
                        else { state.pingPongReverse = false; state.currentWaypointIndex++; }
                    }
                    break;
            }
        }

        public static void CleanupDestroyed()
        {
            var toRemove = new List<int>();
            foreach (var kv in trackedAircraft)
                if (kv.Value.aircraft == null || kv.Value.aircraft.disabled)
                    toRemove.Add(kv.Key);
            foreach (var id in toRemove)
                trackedAircraft.Remove(id);
        }

        /// <summary>Called when an aircraft spawns in gameplay — auto-assigns route if mapped.</summary>
        public static void OnAircraftSpawned(Aircraft aircraft, string uniqueName)
        {
            if (string.IsNullOrEmpty(uniqueName)) return;
            if (assignmentsByName.TryGetValue(uniqueName, out var route) && route.waypoints.Count > 0)
            {
                formationOffsets.TryGetValue(uniqueName, out var offset);
                AssignToAircraft(aircraft, route, offset);
                Plugin.Log?.LogInfo($"Auto-assigned route '{route.name}' to aircraft '{uniqueName}'" +
                    (offset != null ? $" (offset R:{offset.right} U:{offset.up} F:{offset.forward})" : ""));
            }
        }
    }

    // ========== CAMERA PLAYBACK ==========

    public static class CameraPlayback
    {
        public static List<CameraRoute> cameraRouteLibrary = new List<CameraRoute>();
        public static int selectedCameraRouteIndex = -1;

        public static CameraRoute activeRoute;
        public static bool playing;
        public static bool paused;
        public static int currentSegment;
        public static float segmentT;
        public static float playbackSpeed = 1f;
        public static bool pingPongReverse;

        // Frame-calculated values (applied in LateUpdate postfix)
        public static Vector3 framePosition;
        public static Quaternion frameRotation;
        public static float frameFOV;
        public static bool frameReady;

        // Cached LookAt target (set at playback start)
        public static Unit cachedLookAtUnit;

        /// <summary>Get the next waypoint index, handling Loop wrap-around.</summary>
        private static int GetNextIdx(int seg, int count, RouteMode mode)
        {
            if (pingPongReverse)
                return seg - 1;
            if (mode == RouteMode.Loop)
                return (seg + 1) % count; // wrap: last → first
            return seg + 1;
        }

        public static void StartPlayback(CameraRoute route)
        {
            if (route == null || route.waypoints.Count < 2) return;
            activeRoute = route;
            currentSegment = 0;
            segmentT = 0;
            playing = true;
            paused = false;
            pingPongReverse = false;
            frameReady = false;

            // Cache LookAt target
            cachedLookAtUnit = null;
            if (route.lookAtTarget || route.autoSpeed)
            {
                var csm = UnityEngine.Object.FindObjectOfType<CameraStateManager>();
                if (csm != null && csm.followingUnit != null)
                    cachedLookAtUnit = csm.followingUnit;
            }

            Plugin.Log?.LogInfo($"Camera playback started: {route.name} ({route.waypoints.Count} pts, {route.speed} m/s)" +
                (cachedLookAtUnit != null ? $" target={cachedLookAtUnit.name}" : ""));
        }

        public static void StopPlayback()
        {
            playing = false;
            paused = false;
            activeRoute = null;
            frameReady = false;
            cachedLookAtUnit = null;
            Plugin.Log?.LogInfo("Camera playback stopped");
        }

        public static void TogglePause()
        {
            if (!playing) return;
            paused = !paused;
        }

        /// <summary>Called in Update() — advances time and calculates target position/rotation.</summary>
        public static void UpdatePlayback()
        {
            if (!playing || paused || activeRoute == null) { frameReady = false; return; }

            var wps = activeRoute.waypoints;
            int count = wps.Count;
            if (count < 2) { StopPlayback(); return; }

            // Get current segment endpoints
            int nextIdx = GetNextIdx(currentSegment, count, activeRoute.Mode);
            if (nextIdx < 0 || nextIdx >= count) { StopPlayback(); return; }
            var wpA = wps[currentSegment];
            var wpB = wps[nextIdx];

            Vector3 posA = new GlobalPosition(wpA.x, wpA.y, wpA.z).ToLocalPosition();
            Vector3 posB = new GlobalPosition(wpB.x, wpB.y, wpB.z).ToLocalPosition();
            float segmentLength = Vector3.Distance(posA, posB);

            // Determine speed: autoSpeed matches unit speed, otherwise use route speed
            float speed = activeRoute.speed * playbackSpeed;
            if (activeRoute.autoSpeed && cachedLookAtUnit != null)
            {
                // Use unit's current speed
                float unitSpeed = cachedLookAtUnit.speed;
                if (unitSpeed > 1f)
                    speed = unitSpeed * playbackSpeed;
            }

            // Advance based on speed
            if (segmentLength > 0.1f)
                segmentT += (speed * Time.deltaTime) / segmentLength;
            else
                segmentT = 1f;

            // Advance segments
            while (segmentT >= 1f && playing)
            {
                segmentT -= 1f;
                if (!AdvanceSegment()) { frameReady = false; return; }

                nextIdx = GetNextIdx(currentSegment, count, activeRoute.Mode);
                if (nextIdx < 0 || nextIdx >= count) { StopPlayback(); frameReady = false; return; }
                wpA = wps[currentSegment];
                wpB = wps[nextIdx];
                posA = new GlobalPosition(wpA.x, wpA.y, wpA.z).ToLocalPosition();
                posB = new GlobalPosition(wpB.x, wpB.y, wpB.z).ToLocalPosition();
                segmentLength = Vector3.Distance(posA, posB);
            }

            if (!playing) { frameReady = false; return; }

            // Interpolate position
            float t = Mathf.Clamp01(segmentT);
            Vector3 pos = Vector3.Lerp(posA, posB, t);

            // Catmull-Rom smoothing for position (if 4+ waypoints available)
            if (count >= 4)
            {
                int i0, i1, i2, i3;
                if (pingPongReverse)
                {
                    i1 = currentSegment;
                    i2 = Mathf.Max(currentSegment - 1, 0);
                    i0 = Mathf.Min(currentSegment + 1, count - 1);
                    i3 = Mathf.Max(currentSegment - 2, 0);
                }
                else if (activeRoute.Mode == RouteMode.Loop)
                {
                    i1 = currentSegment;
                    i2 = (currentSegment + 1) % count;
                    i0 = (currentSegment - 1 + count) % count;
                    i3 = (currentSegment + 2) % count;
                }
                else
                {
                    i1 = currentSegment;
                    i2 = Mathf.Min(currentSegment + 1, count - 1);
                    i0 = Mathf.Max(currentSegment - 1, 0);
                    i3 = Mathf.Min(currentSegment + 2, count - 1);
                }
                Vector3 p0 = new GlobalPosition(wps[i0].x, wps[i0].y, wps[i0].z).ToLocalPosition();
                Vector3 p1 = new GlobalPosition(wps[i1].x, wps[i1].y, wps[i1].z).ToLocalPosition();
                Vector3 p2 = new GlobalPosition(wps[i2].x, wps[i2].y, wps[i2].z).ToLocalPosition();
                Vector3 p3 = new GlobalPosition(wps[i3].x, wps[i3].y, wps[i3].z).ToLocalPosition();
                pos = CatmullRom(p0, p1, p2, p3, t);
            }

            // Interpolate rotation
            Quaternion rotA = Quaternion.Euler(wpA.rx, wpA.ry, wpA.rz);
            Quaternion rotB = Quaternion.Euler(wpB.rx, wpB.ry, wpB.rz);
            Quaternion rot = Quaternion.Slerp(rotA, rotB, t);

            // Interpolate FOV
            float fovA = wpA.fov > 0 ? wpA.fov : 60f;
            float fovB = wpB.fov > 0 ? wpB.fov : 60f;
            float fov = Mathf.Lerp(fovA, fovB, t);

            // LookAt override: face the cached target unit
            if (activeRoute.lookAtTarget && cachedLookAtUnit != null && !cachedLookAtUnit.disabled)
            {
                Vector3 targetPos = cachedLookAtUnit.transform.position;
                Vector3 dir = targetPos - pos;
                if (dir.sqrMagnitude > 1f)
                    rot = Quaternion.LookRotation(dir);
            }

            // Store for LateUpdate application
            framePosition = pos;
            frameRotation = rot;
            frameFOV = fov;
            frameReady = true;
        }

        /// <summary>Called via Harmony postfix on CameraStateManager.LateUpdate — applies camera AFTER game camera update.</summary>
        public static void ApplyCameraOverride()
        {
            if (!playing || !frameReady) return;

            var cam = Camera.main;
            if (cam == null) return;

            cam.transform.position = framePosition;
            cam.transform.rotation = frameRotation;
            cam.fieldOfView = frameFOV;
        }

        private static bool AdvanceSegment()
        {
            int count = activeRoute.waypoints.Count;
            switch (activeRoute.Mode)
            {
                case RouteMode.Loop:
                    // Loop has N segments (including last→first), so advance mod N
                    currentSegment = (currentSegment + 1) % count;
                    return true;
                case RouteMode.OneShot:
                    currentSegment++;
                    if (currentSegment >= count - 1) { StopPlayback(); return false; }
                    return true;
                case RouteMode.PingPong:
                    if (!pingPongReverse)
                    {
                        currentSegment++;
                        if (currentSegment >= count - 1) { pingPongReverse = true; }
                    }
                    else
                    {
                        currentSegment--;
                        if (currentSegment <= 0) { pingPongReverse = false; }
                    }
                    return true;
            }
            return true;
        }

        private static Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
        {
            float t2 = t * t;
            float t3 = t2 * t;
            return 0.5f * (
                (2f * p1) +
                (-p0 + p2) * t +
                (2f * p0 - 5f * p1 + 4f * p2 - p3) * t2 +
                (-p0 + 3f * p1 - 3f * p2 + p3) * t3
            );
        }

        public static float GetProgress()
        {
            if (activeRoute == null || activeRoute.waypoints.Count < 2) return 0;
            int totalSegments = activeRoute.Mode == RouteMode.Loop
                ? activeRoute.waypoints.Count  // Loop: N segments (including wrap)
                : activeRoute.waypoints.Count - 1;
            return (currentSegment + segmentT) / totalSegments;
        }

        public static void ClearAll()
        {
            StopPlayback();
            cameraRouteLibrary.Clear();
            selectedCameraRouteIndex = -1;
        }
    }

    // ========== HARMONY PATCHES ==========

    // Patch 0: Apply camera override AFTER game camera update
    [HarmonyPatch(typeof(CameraStateManager), "LateUpdate")]
    public static class Patch_CameraLateUpdate
    {
        static void Postfix()
        {
            try
            {
                CameraPlayback.ApplyCameraOverride();
            }
            catch (Exception e)
            {
                Plugin.Log?.LogError($"CameraLateUpdate patch error: {e.Message}");
            }
        }
    }

    // Patch 1: Hijack AI flight when waypoints are assigned
    [HarmonyPatch(typeof(AIPilotCombatModes), "FixedUpdateState")]
    public static class Patch_FixedUpdateState
    {
        private static FieldInfo f_aircraft;
        private static FieldInfo f_destination;
        private static FieldInfo f_controlInputs;
        private static bool reflectionReady;

        static void CacheReflection()
        {
            if (reflectionReady) return;
            var flags = BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Public;
            f_aircraft = typeof(PilotBaseState).GetField("aircraft", flags);
            f_destination = typeof(PilotBaseState).GetField("destination", flags);
            f_controlInputs = typeof(PilotBaseState).GetField("controlInputs", flags);
            reflectionReady = f_aircraft != null && f_destination != null && f_controlInputs != null;
        }

        static bool Prefix(AIPilotCombatModes __instance, Pilot pilot)
        {
            try
            {
                CacheReflection();
                if (!reflectionReady) return true;

                Aircraft aircraft = (Aircraft)f_aircraft.GetValue(__instance);
                if (aircraft == null) return true;

                if (!WaypointManager.TryGetWaypointDestination(aircraft, out GlobalPosition destination, out float targetHeight, out float throttle, out bool aglMode, out bool rawFollow))
                    return true;

                f_destination.SetValue(__instance, destination);

                ControlInputs inputs = (ControlInputs)f_controlInputs.GetValue(__instance);
                if (inputs != null)
                    inputs.throttle = throttle;

                float effort = Plugin.Instance != null ? Plugin.Instance.turnSmoothness.Value : 0.5f;

                // rawFollow: ignore terrain avoidance entirely, follow path exactly
                // AGL mode: disable followTerrain (we compute altitude via raycast)
                bool ignoreCollisions = rawFollow;
                bool followTerrain = rawFollow ? false : !aglMode;
                aircraft.autopilot.AutoAim(
                    destination, true, ignoreCollisions, false,
                    effort, 180f, followTerrain, targetHeight, Vector3.zero
                );

                return false;
            }
            catch (Exception e)
            {
                Plugin.Log?.LogError($"FixedUpdateState patch error: {e.Message}");
                return true;
            }
        }
    }

    // Patch 2: Auto-assign routes when aircraft spawns
    [HarmonyPatch(typeof(Spawner), "AfterSpawn")]
    public static class Patch_AfterSpawn
    {
        static void Postfix(SavedUnit savedUnit, Unit spawnedUnit)
        {
            try
            {
                if (savedUnit is SavedAircraft && spawnedUnit is Aircraft aircraft)
                {
                    string name = savedUnit.UniqueName;
                    if (!string.IsNullOrEmpty(name))
                        WaypointManager.OnAircraftSpawned(aircraft, name);
                }
            }
            catch (Exception e)
            {
                Plugin.Log?.LogError($"AfterSpawn patch error: {e.Message}");
            }
        }
    }

    // Patch 3: Auto-save waypoint data when mission is saved
    [HarmonyPatch(typeof(MissionSaveLoad), "SaveMission", new Type[] { typeof(Mission), typeof(bool) })]
    public static class Patch_SaveMission
    {
        static void Postfix(Mission mission)
        {
            try
            {
                // Only auto-save if we have waypoint data to save
                if (WaypointManager.routeLibrary.Count == 0 && WaypointManager.assignmentsByName.Count == 0)
                    return;

                string missionName = mission?.Name;
                if (string.IsNullOrEmpty(missionName)) return;

                WaypointFileIO.SaveMissionData(missionName);
                Plugin.Log?.LogInfo($"Auto-saved waypoint data for mission '{missionName}'");
            }
            catch (Exception e)
            {
                Plugin.Log?.LogError($"SaveMission patch error: {e.Message}");
            }
        }
    }

    // Patch 5: Auto-load waypoint data when gameplay starts (for non-editor play)
    [HarmonyPatch(typeof(MissionManager), "StartMission")]
    public static class Patch_StartMission
    {
        static void Postfix()
        {
            try
            {
                // Skip if already loaded (editor flow already loads via Patch_EditorLoadMission)
                if (WaypointManager.assignmentsByName.Count > 0) return;

                var mission = MissionManager.CurrentMission;
                string missionName = mission?.Name;
                if (string.IsNullOrEmpty(missionName)) return;

                if (WaypointFileIO.LoadMissionData(missionName))
                    Plugin.Log?.LogInfo($"Auto-loaded waypoint data at mission start for '{missionName}'");
            }
            catch (Exception e)
            {
                Plugin.Log?.LogError($"StartMission patch error: {e.Message}");
            }
        }
    }

    // ========== SIMPLE JSON (replaces broken JsonUtility) ==========

    public static class SimpleJson
    {
        public static string Serialize(MissionWaypointData data)
        {
            var sb = new System.Text.StringBuilder();
            sb.AppendLine("{");

            // Routes
            sb.AppendLine("  \"routes\": [");
            for (int i = 0; i < data.routes.Count; i++)
            {
                var r = data.routes[i];
                sb.AppendLine("    {");
                sb.AppendLine($"      \"name\": {Esc(r.name)},");
                sb.AppendLine($"      \"mode\": {r.mode},");
                sb.AppendLine($"      \"aglMode\": {B(r.aglMode)},");
                sb.AppendLine($"      \"rawFollow\": {B(r.rawFollow)},");
                sb.AppendLine("      \"waypoints\": [");
                for (int j = 0; j < r.waypoints.Count; j++)
                {
                    var w = r.waypoints[j];
                    sb.Append($"        {{\"x\":{F(w.x)},\"y\":{F(w.y)},\"z\":{F(w.z)},\"speed\":{F(w.speed)}}}");
                    sb.AppendLine(j < r.waypoints.Count - 1 ? "," : "");
                }
                sb.AppendLine("      ]");
                sb.Append("    }");
                sb.AppendLine(i < data.routes.Count - 1 ? "," : "");
            }
            sb.AppendLine("  ],");

            // Assignments
            sb.AppendLine("  \"assignments\": [");
            for (int i = 0; i < data.assignments.Count; i++)
            {
                var a = data.assignments[i];
                sb.Append($"    {{\"uniqueName\":{Esc(a.uniqueName)},\"routeIndex\":{a.routeIndex},\"offsetRight\":{F(a.offsetRight)},\"offsetUp\":{F(a.offsetUp)},\"offsetForward\":{F(a.offsetForward)}}}");
                sb.AppendLine(i < data.assignments.Count - 1 ? "," : "");
            }
            sb.AppendLine("  ],");

            // Camera routes
            sb.AppendLine("  \"cameraRoutes\": [");
            for (int i = 0; i < data.cameraRoutes.Count; i++)
            {
                var cr = data.cameraRoutes[i];
                sb.AppendLine("    {");
                sb.AppendLine($"      \"name\": {Esc(cr.name)},");
                sb.AppendLine($"      \"mode\": {cr.mode},");
                sb.AppendLine($"      \"speed\": {F(cr.speed)},");
                sb.AppendLine($"      \"lookAtTarget\": {B(cr.lookAtTarget)},");
                sb.AppendLine($"      \"autoSpeed\": {B(cr.autoSpeed)},");
                sb.AppendLine("      \"waypoints\": [");
                for (int j = 0; j < cr.waypoints.Count; j++)
                {
                    var cw = cr.waypoints[j];
                    sb.Append($"        {{\"x\":{F(cw.x)},\"y\":{F(cw.y)},\"z\":{F(cw.z)},\"rx\":{F(cw.rx)},\"ry\":{F(cw.ry)},\"rz\":{F(cw.rz)},\"fov\":{F(cw.fov)}}}");
                    sb.AppendLine(j < cr.waypoints.Count - 1 ? "," : "");
                }
                sb.AppendLine("      ]");
                sb.Append("    }");
                sb.AppendLine(i < data.cameraRoutes.Count - 1 ? "," : "");
            }
            sb.AppendLine("  ]");

            sb.AppendLine("}");
            return sb.ToString();
        }

        public static MissionWaypointData Deserialize(string json)
        {
            var data = new MissionWaypointData();
            try
            {
                // Use Unity's built-in JSON parser for reading (works for deserialization)
                var parsed = (Dictionary<string, object>)MiniJsonParser.Parse(json);

                if (parsed.ContainsKey("routes"))
                {
                    foreach (Dictionary<string, object> r in (List<object>)parsed["routes"])
                    {
                        var route = new WaypointRoute();
                        route.name = Str(r, "name", "Route");
                        route.mode = Int(r, "mode");
                        route.aglMode = Bool(r, "aglMode");
                        route.rawFollow = Bool(r, "rawFollow");
                        if (r.ContainsKey("waypoints"))
                        {
                            foreach (Dictionary<string, object> w in (List<object>)r["waypoints"])
                            {
                                route.waypoints.Add(new Waypoint
                                {
                                    x = Flt(w, "x"), y = Flt(w, "y"), z = Flt(w, "z"),
                                    speed = Flt(w, "speed")
                                });
                            }
                        }
                        data.routes.Add(route);
                    }
                }

                if (parsed.ContainsKey("assignments"))
                {
                    foreach (Dictionary<string, object> a in (List<object>)parsed["assignments"])
                    {
                        data.assignments.Add(new AircraftAssignment
                        {
                            uniqueName = Str(a, "uniqueName", ""),
                            routeIndex = Int(a, "routeIndex"),
                            offsetRight = Flt(a, "offsetRight"),
                            offsetUp = Flt(a, "offsetUp"),
                            offsetForward = Flt(a, "offsetForward")
                        });
                    }
                }

                if (parsed.ContainsKey("cameraRoutes"))
                {
                    foreach (Dictionary<string, object> cr in (List<object>)parsed["cameraRoutes"])
                    {
                        var camRoute = new CameraRoute();
                        camRoute.name = Str(cr, "name", "Camera Route");
                        camRoute.mode = Int(cr, "mode");
                        camRoute.speed = Flt(cr, "speed", 50f);
                        camRoute.lookAtTarget = Bool(cr, "lookAtTarget");
                        camRoute.autoSpeed = Bool(cr, "autoSpeed");
                        if (cr.ContainsKey("waypoints"))
                        {
                            foreach (Dictionary<string, object> cw in (List<object>)cr["waypoints"])
                            {
                                camRoute.waypoints.Add(new CameraWaypoint
                                {
                                    x = Flt(cw, "x"), y = Flt(cw, "y"), z = Flt(cw, "z"),
                                    rx = Flt(cw, "rx"), ry = Flt(cw, "ry"), rz = Flt(cw, "rz"),
                                    fov = Flt(cw, "fov", 60f)
                                });
                            }
                        }
                        data.cameraRoutes.Add(camRoute);
                    }
                }
            }
            catch (Exception e)
            {
                Plugin.Log?.LogError($"JSON parse error: {e.Message}");
            }
            return data;
        }

        private static string Esc(string s) => "\"" + (s ?? "").Replace("\\", "\\\\").Replace("\"", "\\\"") + "\"";
        private static string F(float v) => v.ToString("G9", System.Globalization.CultureInfo.InvariantCulture);
        private static string B(bool v) => v ? "true" : "false";
        private static string Str(Dictionary<string, object> d, string k, string def = "") => d.ContainsKey(k) ? d[k]?.ToString() ?? def : def;
        private static int Int(Dictionary<string, object> d, string k) => d.ContainsKey(k) ? (int)(double)d[k] : 0;
        private static float Flt(Dictionary<string, object> d, string k, float def = 0f) => d.ContainsKey(k) ? (float)(double)d[k] : def;
        private static bool Bool(Dictionary<string, object> d, string k) => d.ContainsKey(k) && d[k] is bool b && b;
    }

    // ========== MINI JSON PARSER ==========

    public static class MiniJsonParser
    {
        private static int idx;
        private static string src;

        public static object Parse(string json)
        {
            src = json;
            idx = 0;
            return ParseValue();
        }

        private static void SkipWhitespace() { while (idx < src.Length && " \t\n\r".IndexOf(src[idx]) >= 0) idx++; }

        private static object ParseValue()
        {
            SkipWhitespace();
            if (idx >= src.Length) return null;
            char c = src[idx];
            if (c == '{') return ParseObject();
            if (c == '[') return ParseArray();
            if (c == '"') return ParseString();
            if (c == 't') { idx += 4; return true; }
            if (c == 'f') { idx += 5; return false; }
            if (c == 'n') { idx += 4; return null; }
            return ParseNumber();
        }

        private static Dictionary<string, object> ParseObject()
        {
            var dict = new Dictionary<string, object>();
            idx++; // skip {
            SkipWhitespace();
            if (idx < src.Length && src[idx] == '}') { idx++; return dict; }
            while (idx < src.Length)
            {
                SkipWhitespace();
                string key = ParseString();
                SkipWhitespace();
                idx++; // skip :
                dict[key] = ParseValue();
                SkipWhitespace();
                if (idx < src.Length && src[idx] == ',') { idx++; continue; }
                break;
            }
            if (idx < src.Length && src[idx] == '}') idx++;
            return dict;
        }

        private static List<object> ParseArray()
        {
            var list = new List<object>();
            idx++; // skip [
            SkipWhitespace();
            if (idx < src.Length && src[idx] == ']') { idx++; return list; }
            while (idx < src.Length)
            {
                list.Add(ParseValue());
                SkipWhitespace();
                if (idx < src.Length && src[idx] == ',') { idx++; continue; }
                break;
            }
            if (idx < src.Length && src[idx] == ']') idx++;
            return list;
        }

        private static string ParseString()
        {
            idx++; // skip opening "
            var sb = new System.Text.StringBuilder();
            while (idx < src.Length)
            {
                char c = src[idx++];
                if (c == '"') return sb.ToString();
                if (c == '\\' && idx < src.Length)
                {
                    char next = src[idx++];
                    if (next == '"') sb.Append('"');
                    else if (next == '\\') sb.Append('\\');
                    else if (next == 'n') sb.Append('\n');
                    else if (next == 't') sb.Append('\t');
                    else sb.Append(next);
                }
                else sb.Append(c);
            }
            return sb.ToString();
        }

        private static double ParseNumber()
        {
            int start = idx;
            while (idx < src.Length && "-0123456789.eE+".IndexOf(src[idx]) >= 0) idx++;
            double.TryParse(src.Substring(start, idx - start), System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out double val);
            return val;
        }
    }

    // ========== FILE IO ==========

    public static class WaypointFileIO
    {
        private static string dataFolder;

        public static void Initialize()
        {
            dataFolder = Path.Combine(
                Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location),
                "WaypointData");
            if (!Directory.Exists(dataFolder))
                Directory.CreateDirectory(dataFolder);
        }

        public static string GetDataFolder() => dataFolder;

        public static void SaveMissionData(string missionName)
        {
            var data = new MissionWaypointData();
            data.routes = new List<WaypointRoute>(WaypointManager.routeLibrary);
            data.cameraRoutes = new List<CameraRoute>(CameraPlayback.cameraRouteLibrary);

            foreach (var kv in WaypointManager.assignmentsByName)
            {
                int idx = WaypointManager.routeLibrary.IndexOf(kv.Value);
                if (idx >= 0)
                {
                    var assignment = new AircraftAssignment { uniqueName = kv.Key, routeIndex = idx };
                    if (WaypointManager.formationOffsets.TryGetValue(kv.Key, out var offset))
                    {
                        assignment.offsetRight = offset.right;
                        assignment.offsetUp = offset.up;
                        assignment.offsetForward = offset.forward;
                    }
                    data.assignments.Add(assignment);
                }
            }

            string safeName = string.Join("_", missionName.Split(Path.GetInvalidFileNameChars()));
            if (string.IsNullOrEmpty(safeName)) safeName = "mission";
            string path = Path.Combine(dataFolder, safeName + ".json");
            string json = SimpleJson.Serialize(data);
            File.WriteAllText(path, json);
            Plugin.Log?.LogInfo($"Waypoint data saved: {path} ({data.routes.Count} routes, {data.assignments.Count} assignments)");
        }

        public static bool LoadMissionData(string missionName)
        {
            string safeName = string.Join("_", missionName.Split(Path.GetInvalidFileNameChars()));
            string path = Path.Combine(dataFolder, safeName + ".json");
            if (!File.Exists(path)) return false;

            try
            {
                string json = File.ReadAllText(path);
                var data = SimpleJson.Deserialize(json);

                WaypointManager.routeLibrary = data.routes ?? new List<WaypointRoute>();
                CameraPlayback.cameraRouteLibrary = data.cameraRoutes ?? new List<CameraRoute>();
                CameraPlayback.selectedCameraRouteIndex = CameraPlayback.cameraRouteLibrary.Count > 0 ? 0 : -1;
                WaypointManager.assignmentsByName.Clear();
                WaypointManager.formationOffsets.Clear();

                foreach (var a in data.assignments)
                {
                    if (a.routeIndex >= 0 && a.routeIndex < WaypointManager.routeLibrary.Count)
                    {
                        WaypointManager.assignmentsByName[a.uniqueName] = WaypointManager.routeLibrary[a.routeIndex];
                        if (a.offsetRight != 0 || a.offsetUp != 0 || a.offsetForward != 0)
                        {
                            WaypointManager.formationOffsets[a.uniqueName] = new FormationOffset
                            {
                                right = a.offsetRight,
                                up = a.offsetUp,
                                forward = a.offsetForward
                            };
                        }
                    }
                }

                Plugin.Log?.LogInfo($"Waypoint data loaded: {path} ({data.routes.Count} routes, {data.assignments.Count} assignments)");

                // Re-link already-spawned aircraft to loaded assignments
                var allAircraft = UnityEngine.Object.FindObjectsOfType<Aircraft>();
                foreach (var ac in allAircraft)
                {
                    if (ac == null || ac.disabled) continue;
                    string uname = "";
                    if (ac.SavedUnit != null)
                        uname = ac.SavedUnit.UniqueName ?? "";
                    else if (!string.IsNullOrEmpty(ac.UniqueName))
                        uname = ac.UniqueName;
                    if (!string.IsNullOrEmpty(uname))
                        WaypointManager.OnAircraftSpawned(ac, uname);
                }

                return true;
            }
            catch (Exception e)
            {
                Plugin.Log?.LogError($"Load failed: {e.Message}");
                return false;
            }
        }

        public static List<string> GetSavedFiles()
        {
            if (!Directory.Exists(dataFolder)) return new List<string>();
            return Directory.GetFiles(dataFolder, "*.json")
                .Where(f => !Path.GetFileName(f).StartsWith("__"))
                .ToList();
        }
    }

    // ========== UI ==========

    public class WaypointUI
    {
        private bool showUI;
        private bool placingWaypoint;
        private bool stylesInitialized;
        private Rect windowRect = new Rect(20, 100, 400, 620);
        private Vector2 aircraftScroll;
        private Vector2 waypointScroll;
        private Vector2 routeScroll;
        private float waypointAltitude = 300f;
        private float waypointSpeed;
        private string altitudeText = "300";
        private string speedText = "0";
        private string saveNameText = "mission1";
        private float cleanupTimer;
        private int uiTab;
        private List<string> savedFiles = new List<string>();

        // Formation offset editing
        private string offsetRightText = "0";
        private string offsetUpText = "0";
        private string offsetForwardText = "0";

        // Camera recording (aircraft waypoints)
        private bool recording;
        private float recordTimer;
        private float recordInterval = 0.5f;
        private string recordIntervalText = "0.5";

        // Camera route recording
        private bool camRecording;
        private float camRecordTimer;
        private float camRecordInterval = 0.2f;
        private string camRecordIntervalText = "0.2";
        private Vector2 camRouteScroll;
        private Vector2 camWaypointScroll;
        private Vector2 camTargetScroll;
        private string camSpeedText = "50";
        private string camPlaybackSpeedText = "1";
        private bool showTargetPicker;

        private GUIStyle boxStyle, labelStyle, buttonStyle, headerStyle, activeButtonStyle;
        private GUIStyle smallButtonStyle, textFieldStyle, wpLabelStyle, infoStyle;
        private Texture2D bgTex, activeBtnTex;

        // Route color palette
        private static readonly Color[] routeColors = new Color[]
        {
            new Color(1f, 0.8f, 0.2f, 0.8f),   // Yellow
            new Color(0.3f, 0.9f, 1f, 0.8f),    // Cyan
            new Color(0.4f, 1f, 0.4f, 0.8f),    // Green
            new Color(1f, 0.5f, 0.9f, 0.8f),    // Pink
            new Color(1f, 0.6f, 0.2f, 0.8f),    // Orange
            new Color(0.6f, 0.6f, 1f, 0.8f),    // Lavender
        };
        private static readonly Color[] camRouteColors = new Color[]
        {
            new Color(1f, 0.3f, 0.3f, 0.8f),    // Red
            new Color(0.3f, 0.5f, 1f, 0.8f),    // Blue
            new Color(0.8f, 0.3f, 1f, 0.8f),    // Purple
            new Color(1f, 0.9f, 0.3f, 0.8f),    // Gold
            new Color(0.3f, 1f, 0.8f, 0.8f),    // Teal
            new Color(1f, 0.5f, 0.5f, 0.8f),    // Salmon
        };

        private bool IsEditor => GameManager.gameState == GameState.Editor;

        public void SetSaveName(string name)
        {
            saveNameText = name;
        }

        private void InitStyles()
        {
            if (stylesInitialized) return;

            bgTex = MakeTex(1, 1, new Color(0.1f, 0.1f, 0.15f, 0.95f));
            activeBtnTex = MakeTex(1, 1, new Color(0.2f, 0.5f, 0.3f, 1f));

            boxStyle = new GUIStyle(GUI.skin.box) { padding = new RectOffset(8, 8, 8, 8) };
            boxStyle.normal.background = bgTex;

            labelStyle = new GUIStyle(GUI.skin.label) { fontSize = 12 };
            labelStyle.normal.textColor = Color.white;

            headerStyle = new GUIStyle(GUI.skin.label) { fontSize = 14, fontStyle = FontStyle.Bold };
            headerStyle.normal.textColor = new Color(0.6f, 0.85f, 1f);

            buttonStyle = new GUIStyle(GUI.skin.button) { fontSize = 11 };

            activeButtonStyle = new GUIStyle(GUI.skin.button) { fontSize = 11 };
            activeButtonStyle.normal.background = activeBtnTex;
            activeButtonStyle.normal.textColor = Color.white;

            smallButtonStyle = new GUIStyle(GUI.skin.button) { fontSize = 10, padding = new RectOffset(4, 4, 2, 2) };

            textFieldStyle = new GUIStyle(GUI.skin.textField) { fontSize = 11 };

            wpLabelStyle = new GUIStyle(GUI.skin.label) { fontSize = 11 };
            wpLabelStyle.normal.textColor = new Color(0.85f, 0.85f, 0.85f);

            infoStyle = new GUIStyle(GUI.skin.label) { fontSize = 10 };
            infoStyle.normal.textColor = new Color(0.5f, 0.7f, 0.5f);

            stylesInitialized = true;
        }

        private Texture2D MakeTex(int w, int h, Color col)
        {
            var tex = new Texture2D(w, h);
            tex.SetPixel(0, 0, col);
            tex.Apply();
            return tex;
        }

        public void HandleInput()
        {
            KeyCode key = Plugin.Instance != null ? Plugin.Instance.toggleKey.Value : KeyCode.F5;
            if (Input.GetKeyDown(key))
                showUI = !showUI;


            if (placingWaypoint && Input.GetMouseButtonDown(0))
            {
                var cam = Camera.main;
                if (cam != null)
                {
                    Ray ray = cam.ScreenPointToRay(Input.mousePosition);
                    if (Physics.Raycast(ray, out RaycastHit hit, 100000f))
                    {
                        GlobalPosition gp = hit.point.ToGlobalPosition();
                        var route = GetOrCreateCurrentRoute();
                        float y;
                        if (route != null && route.aglMode)
                            y = waypointAltitude; // AGL: store height above ground directly
                        else
                            y = gp.y + waypointAltitude; // Absolute: ground + altitude offset
                        AddWaypointToCurrentRoute(gp.x, y, gp.z);
                        placingWaypoint = false;
                    }
                }
            }

            if (placingWaypoint && Input.GetKeyDown(KeyCode.Escape))
                placingWaypoint = false;

            // Camera recording: place waypoint at camera position every N seconds
            if (recording)
            {
                recordTimer += Time.unscaledDeltaTime;
                if (recordTimer >= recordInterval)
                {
                    recordTimer -= recordInterval;
                    var cam = Camera.main;
                    if (cam != null)
                    {
                        Vector3 camPos = cam.transform.position;
                        GlobalPosition gp = camPos.ToGlobalPosition();
                        var route = GetOrCreateCurrentRoute();
                        if (route != null)
                        {
                            float y;
                            if (route.aglMode)
                            {
                                // AGL: store height above ground
                                float groundH = WaypointManager.GetGroundHeight(camPos);
                                y = camPos.y - groundH;
                                if (y < 1f) y = 1f;
                            }
                            else
                            {
                                y = gp.y;
                            }
                            route.waypoints.Add(new Waypoint { x = gp.x, y = y, z = gp.z, speed = waypointSpeed });
                        }
                    }
                }
            }

            // Camera route recording: position + rotation + FOV
            if (camRecording)
            {
                camRecordTimer += Time.unscaledDeltaTime;
                if (camRecordTimer >= camRecordInterval)
                {
                    camRecordTimer -= camRecordInterval;
                    var cam = Camera.main;
                    if (cam != null && CameraPlayback.selectedCameraRouteIndex >= 0 &&
                        CameraPlayback.selectedCameraRouteIndex < CameraPlayback.cameraRouteLibrary.Count)
                    {
                        var route = CameraPlayback.cameraRouteLibrary[CameraPlayback.selectedCameraRouteIndex];
                        Vector3 pos = cam.transform.position;
                        Vector3 euler = cam.transform.eulerAngles;
                        GlobalPosition gp = pos.ToGlobalPosition();
                        route.waypoints.Add(new CameraWaypoint
                        {
                            x = gp.x, y = gp.y, z = gp.z,
                            rx = euler.x, ry = euler.y, rz = euler.z,
                            fov = cam.fieldOfView
                        });
                    }
                }
            }

            if (!IsEditor)
            {
                cleanupTimer += Time.deltaTime;
                if (cleanupTimer > 5f)
                {
                    cleanupTimer = 0f;
                    WaypointManager.CleanupDestroyed();
                }
            }
        }

        private void AddWaypointToCurrentRoute(float x, float y, float z)
        {
            WaypointRoute route = GetOrCreateCurrentRoute();
            if (route == null) return;
            route.waypoints.Add(new Waypoint { x = x, y = y, z = z, speed = waypointSpeed });
            Plugin.Log?.LogInfo($"WP added: ({x:F0}, {y:F0}, {z:F0})");
        }

        private WaypointRoute GetOrCreateCurrentRoute()
        {
            if (WaypointManager.selectedRouteIndex >= 0 && WaypointManager.selectedRouteIndex < WaypointManager.routeLibrary.Count)
                return WaypointManager.routeLibrary[WaypointManager.selectedRouteIndex];

            var route = new WaypointRoute { name = "Route " + (WaypointManager.routeLibrary.Count + 1) };
            WaypointManager.routeLibrary.Add(route);
            WaypointManager.selectedRouteIndex = WaypointManager.routeLibrary.Count - 1;
            return route;
        }

        public void Draw()
        {
            if (!showUI) return;
            InitStyles();
            windowRect = GUI.Window(59812, windowRect, DrawWindow, "", boxStyle);
            DrawAllRouteMarkers();
            DrawAllCameraRouteMarkers();
            DrawAircraftMarkers();

            if (placingWaypoint)
            {
                var cam = Camera.main;
                if (cam != null)
                {
                    Ray ray = cam.ScreenPointToRay(Input.mousePosition);
                    if (Physics.Raycast(ray, out RaycastHit hit, 100000f))
                    {
                        // Ground point (hit) and altitude point
                        Vector3 groundPos = hit.point;
                        Vector3 altPos = groundPos + Vector3.up * waypointAltitude;

                        Vector3 groundScreen = cam.WorldToScreenPoint(groundPos);
                        Vector3 altScreen = cam.WorldToScreenPoint(altPos);

                        if (groundScreen.z > 0 && altScreen.z > 0)
                        {
                            float gx = groundScreen.x, gy = Screen.height - groundScreen.y;
                            float ax = altScreen.x, ay = Screen.height - altScreen.y;

                            // Vertical line: ground → altitude
                            GUI.color = new Color(1f, 1f, 0.3f, 0.7f);
                            DrawLine(new Vector2(gx, gy), new Vector2(ax, ay), 2f);

                            // Ground marker
                            GUI.color = new Color(1f, 0.5f, 0.2f, 0.8f);
                            GUI.DrawTexture(new Rect(gx - 4, gy - 4, 8, 8), Texture2D.whiteTexture);

                            // Altitude marker
                            GUI.color = new Color(1f, 1f, 0.3f, 0.9f);
                            GUI.DrawTexture(new Rect(ax - 5, ay - 5, 10, 10), Texture2D.whiteTexture);

                            // Altitude label
                            string altLabel = $"+{waypointAltitude:F0}m";
                            GUI.color = new Color(0, 0, 0, 0.8f);
                            GUI.Label(new Rect((gx + ax) / 2 + 6, (gy + ay) / 2 - 9, 80, 20), altLabel);
                            GUI.color = new Color(1f, 1f, 0.5f, 1f);
                            GUI.Label(new Rect((gx + ax) / 2 + 5, (gy + ay) / 2 - 10, 80, 20), altLabel);
                        }
                    }
                }

                float mx = Input.mousePosition.x, my = Screen.height - Input.mousePosition.y;
                GUI.color = Color.yellow;
                GUI.Label(new Rect(mx + 15, my, 250, 25),
                    $"Click to place WP (alt: {waypointAltitude}m) [ESC cancel]", labelStyle);
                GUI.color = Color.white;
            }
        }

        private void DrawWindow(int windowID)
        {
            GUILayout.Label("Aircraft Waypoints v2.0.0", headerStyle);
            GUILayout.Label(IsEditor ? "[Mission Editor Mode]" : "[Gameplay Mode]", infoStyle);
            GUILayout.Space(2);

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Aircraft", uiTab == 0 ? activeButtonStyle : buttonStyle)) uiTab = 0;
            if (GUILayout.Button("Routes", uiTab == 1 ? activeButtonStyle : buttonStyle)) uiTab = 1;
            if (GUILayout.Button("Camera", uiTab == 3 ? activeButtonStyle : buttonStyle)) uiTab = 3;
            if (GUILayout.Button("Save/Load", uiTab == 2 ? activeButtonStyle : buttonStyle)) uiTab = 2;
            GUILayout.EndHorizontal();
            GUILayout.Space(4);

            switch (uiTab)
            {
                case 0: DrawAircraftTab(); break;
                case 1: DrawRouteEditorTab(); break;
                case 2: DrawFileTab(); break;
                case 3: DrawCameraTab(); break;
            }

            GUI.DragWindow();
        }

        // ---- Tab 0: Aircraft ----
        private void DrawAircraftTab()
        {
            GUILayout.Label("AI Aircraft", headerStyle);

            // Get aircraft list — works in both editor and gameplay
            var allAircraft = UnityEngine.Object.FindObjectsOfType<Aircraft>();
            var aiList = new List<(Aircraft ac, string uniqueName, string displayName)>();

            foreach (var ac in allAircraft)
            {
                if (ac == null || ac.disabled) continue;

                // In editor, all aircraft are "AI" (no player control active)
                // In gameplay, filter to AI-only
                if (!IsEditor && ac.pilots != null && ac.pilots.Length > 0 && ac.pilots[0].playerControlled)
                    continue;

                string uname = "";
                if (ac.SavedUnit != null)
                    uname = ac.SavedUnit.UniqueName ?? "";
                else if (!string.IsNullOrEmpty(ac.UniqueName))
                    uname = ac.UniqueName;

                string dname = ac.definition != null ? ac.definition.name : "Aircraft";
                aiList.Add((ac, uname, dname));
            }

            if (aiList.Count == 0)
            {
                GUILayout.Label("No AI aircraft found. Place aircraft in the editor.", wpLabelStyle);
                return;
            }

            aircraftScroll = GUILayout.BeginScrollView(aircraftScroll, GUILayout.Height(180));
            foreach (var (ac, uname, dname) in aiList)
            {
                bool isSelected = WaypointManager.selectedUniqueName == uname && !string.IsNullOrEmpty(uname);
                bool hasRoute = !string.IsNullOrEmpty(uname) && WaypointManager.assignmentsByName.ContainsKey(uname);

                // Runtime status
                string status = "";
                if (!IsEditor)
                {
                    var state = WaypointManager.GetState(ac);
                    if (state != null && state.active)
                        status = $" [WP {state.currentWaypointIndex + 1}/{state.route?.waypoints.Count}]";
                }

                string routeTag = hasRoute ? " *" : "";
                string label = isSelected ? $"> {dname} [{uname}]{routeTag}{status}" : $"  {dname} [{uname}]{routeTag}{status}";

                if (GUILayout.Button(label, isSelected ? activeButtonStyle : buttonStyle))
                {
                    WaypointManager.selectedUniqueName = uname;
                    // Load existing offset into text fields
                    if (!string.IsNullOrEmpty(uname) && WaypointManager.formationOffsets.TryGetValue(uname, out var existOffset))
                    {
                        offsetRightText = existOffset.right.ToString("F0");
                        offsetUpText = existOffset.up.ToString("F0");
                        offsetForwardText = existOffset.forward.ToString("F0");
                    }
                    else
                    {
                        offsetRightText = "0"; offsetUpText = "0"; offsetForwardText = "0";
                    }
                }
            }
            GUILayout.EndScrollView();

            GUILayout.Space(4);

            // Assignment controls for selected aircraft
            string sel = WaypointManager.selectedUniqueName;
            if (!string.IsNullOrEmpty(sel))
            {
                GUILayout.Label($"Selected: {sel}", labelStyle);

                bool hasAssignment = WaypointManager.assignmentsByName.TryGetValue(sel, out var assignedRoute);
                if (hasAssignment)
                {
                    WaypointManager.formationOffsets.TryGetValue(sel, out var curOffset);
                    string offsetStr = curOffset != null ? $" [R:{curOffset.right:F0} U:{curOffset.up:F0} F:{curOffset.forward:F0}]" : "";
                    GUILayout.Label($"Route: {assignedRoute.name} ({assignedRoute.waypoints.Count} WPs, {assignedRoute.Mode}){offsetStr}", wpLabelStyle);
                    if (GUILayout.Button("Unassign Route", buttonStyle))
                        WaypointManager.UnassignByName(sel);
                }
                else
                {
                    GUILayout.Label("No route assigned", wpLabelStyle);
                }

                // Assign from route library
                if (WaypointManager.selectedRouteIndex >= 0 && WaypointManager.selectedRouteIndex < WaypointManager.routeLibrary.Count)
                {
                    var route = WaypointManager.routeLibrary[WaypointManager.selectedRouteIndex];
                    if (route.waypoints.Count > 0)
                    {
                        if (GUILayout.Button($"Assign: {route.name} ({route.waypoints.Count} WPs)", buttonStyle))
                        {
                            float.TryParse(offsetRightText, out float oR);
                            float.TryParse(offsetUpText, out float oU);
                            float.TryParse(offsetForwardText, out float oF);
                            FormationOffset offset = (oR != 0 || oU != 0 || oF != 0) ? new FormationOffset { right = oR, up = oU, forward = oF } : null;
                            WaypointManager.AssignRouteByName(sel, route, offset);
                            Plugin.Log?.LogInfo($"Assigned '{route.name}' to '{sel}'" + (offset != null ? $" offset R:{oR} U:{oU} F:{oF}" : ""));
                        }
                    }
                }
                else
                {
                    GUILayout.Label("Select a route in Routes tab to assign", wpLabelStyle);
                }

                // Formation offset controls
                GUILayout.Space(4);
                GUILayout.Label("Formation Offset (meters)", labelStyle);
                GUILayout.BeginHorizontal();
                GUILayout.Label("R:", wpLabelStyle, GUILayout.Width(16));
                offsetRightText = GUILayout.TextField(offsetRightText, textFieldStyle, GUILayout.Width(50));
                GUILayout.Label("U:", wpLabelStyle, GUILayout.Width(16));
                offsetUpText = GUILayout.TextField(offsetUpText, textFieldStyle, GUILayout.Width(50));
                GUILayout.Label("F:", wpLabelStyle, GUILayout.Width(14));
                offsetForwardText = GUILayout.TextField(offsetForwardText, textFieldStyle, GUILayout.Width(50));
                GUILayout.EndHorizontal();

                // Update offset on assigned aircraft
                if (hasAssignment)
                {
                    if (GUILayout.Button("Update Offset", smallButtonStyle))
                    {
                        float.TryParse(offsetRightText, out float oR);
                        float.TryParse(offsetUpText, out float oU);
                        float.TryParse(offsetForwardText, out float oF);
                        if (oR != 0 || oU != 0 || oF != 0)
                            WaypointManager.formationOffsets[sel] = new FormationOffset { right = oR, up = oU, forward = oF };
                        else
                            WaypointManager.formationOffsets.Remove(sel);
                        // Update runtime state too
                        var acObj = aiList.FirstOrDefault(a => a.uniqueName == sel).ac;
                        if (acObj != null)
                        {
                            var rtState = WaypointManager.GetState(acObj);
                            if (rtState != null)
                                rtState.offset = (oR != 0 || oU != 0 || oF != 0) ? new FormationOffset { right = oR, up = oU, forward = oF } : null;
                        }
                    }
                }

                // Formation presets
                GUILayout.BeginHorizontal();
                GUILayout.Label("Preset:", wpLabelStyle, GUILayout.Width(45));
                if (GUILayout.Button("Leader", smallButtonStyle)) { offsetRightText = "0"; offsetUpText = "0"; offsetForwardText = "0"; }
                if (GUILayout.Button("R Wing", smallButtonStyle)) { offsetRightText = "200"; offsetUpText = "0"; offsetForwardText = "-100"; }
                if (GUILayout.Button("L Wing", smallButtonStyle)) { offsetRightText = "-200"; offsetUpText = "0"; offsetForwardText = "-100"; }
                if (GUILayout.Button("Trail", smallButtonStyle)) { offsetRightText = "0"; offsetUpText = "0"; offsetForwardText = "-200"; }
                GUILayout.EndHorizontal();

                // In gameplay: manual runtime controls
                if (!IsEditor)
                {
                    var acObj = aiList.FirstOrDefault(a => a.uniqueName == sel).ac;
                    if (acObj != null)
                    {
                        var state = WaypointManager.GetState(acObj);
                        if (state != null)
                        {
                            GUILayout.BeginHorizontal();
                            if (GUILayout.Button(state.active ? "Pause" : "Resume", buttonStyle, GUILayout.Width(80)))
                                state.active = !state.active;
                            if (GUILayout.Button("Reset WP", buttonStyle, GUILayout.Width(80)))
                            {
                                state.currentWaypointIndex = 0;
                                state.pingPongReverse = false;
                                state.active = true;
                            }
                            GUILayout.EndHorizontal();
                        }
                    }
                }
            }
        }

        // ---- Tab 1: Route Editor ----
        private void DrawRouteEditorTab()
        {
            GUILayout.Label("Route Library", headerStyle);

            routeScroll = GUILayout.BeginScrollView(routeScroll, GUILayout.Height(80));
            for (int i = 0; i < WaypointManager.routeLibrary.Count; i++)
            {
                var r = WaypointManager.routeLibrary[i];
                bool isSel = i == WaypointManager.selectedRouteIndex;
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(isSel ? $"> {r.name} ({r.waypoints.Count})" : $"  {r.name} ({r.waypoints.Count})",
                    isSel ? activeButtonStyle : buttonStyle))
                {
                    WaypointManager.selectedRouteIndex = i;
                }
                if (GUILayout.Button("X", smallButtonStyle, GUILayout.Width(25)))
                {
                    // Remove assignments referencing this route
                    var toRemove = WaypointManager.assignmentsByName.Where(kv => kv.Value == r).Select(kv => kv.Key).ToList();
                    foreach (var key in toRemove)
                        WaypointManager.assignmentsByName.Remove(key);

                    WaypointManager.routeLibrary.RemoveAt(i);
                    if (WaypointManager.selectedRouteIndex >= WaypointManager.routeLibrary.Count)
                        WaypointManager.selectedRouteIndex = WaypointManager.routeLibrary.Count - 1;
                    break;
                }
                GUILayout.EndHorizontal();
            }
            GUILayout.EndScrollView();

            if (GUILayout.Button("+ New Route", buttonStyle))
            {
                var route = new WaypointRoute { name = "Route " + (WaypointManager.routeLibrary.Count + 1) };
                WaypointManager.routeLibrary.Add(route);
                WaypointManager.selectedRouteIndex = WaypointManager.routeLibrary.Count - 1;
            }

            GUILayout.Space(6);

            if (WaypointManager.selectedRouteIndex < 0 || WaypointManager.selectedRouteIndex >= WaypointManager.routeLibrary.Count)
            {
                GUILayout.Label("Select or create a route above", wpLabelStyle);
                return;
            }

            var currentRoute = WaypointManager.routeLibrary[WaypointManager.selectedRouteIndex];

            // Name
            GUILayout.BeginHorizontal();
            GUILayout.Label("Name:", labelStyle, GUILayout.Width(45));
            string newName = GUILayout.TextField(currentRoute.name, textFieldStyle);
            if (newName != currentRoute.name) currentRoute.name = newName;
            GUILayout.EndHorizontal();

            // Mode
            GUILayout.BeginHorizontal();
            GUILayout.Label("Mode:", labelStyle, GUILayout.Width(45));
            if (GUILayout.Button("Loop", currentRoute.Mode == RouteMode.Loop ? activeButtonStyle : buttonStyle))
                currentRoute.Mode = RouteMode.Loop;
            if (GUILayout.Button("OneShot", currentRoute.Mode == RouteMode.OneShot ? activeButtonStyle : buttonStyle))
                currentRoute.Mode = RouteMode.OneShot;
            if (GUILayout.Button("PingPong", currentRoute.Mode == RouteMode.PingPong ? activeButtonStyle : buttonStyle))
                currentRoute.Mode = RouteMode.PingPong;
            GUILayout.EndHorizontal();

            // Route options
            GUILayout.BeginHorizontal();
            currentRoute.aglMode = GUILayout.Toggle(currentRoute.aglMode, " AGL", GUILayout.Width(55));
            currentRoute.rawFollow = GUILayout.Toggle(currentRoute.rawFollow, " Raw", GUILayout.Width(55));
            GUILayout.EndHorizontal();
            var hintStyle = new GUIStyle(GUI.skin.label) { fontSize = 10, normal = { textColor = new Color(0.6f, 0.6f, 0.6f) } };
            GUILayout.Label("AGL: altitude above ground | Raw: follow path exactly, ignore terrain avoidance", hintStyle);

            GUILayout.Space(4);

            // Waypoints
            GUILayout.Label($"Waypoints ({currentRoute.waypoints.Count})", labelStyle);
            waypointScroll = GUILayout.BeginScrollView(waypointScroll, GUILayout.Height(120));
            for (int i = 0; i < currentRoute.waypoints.Count; i++)
            {
                var wp = currentRoute.waypoints[i];

                // Calculate angle between prev→current→next to detect sharp turns
                string angleWarning = "";
                if (i > 0 && i < currentRoute.waypoints.Count - 1)
                {
                    var prev = currentRoute.waypoints[i - 1];
                    var next = currentRoute.waypoints[i + 1];
                    Vector3 dirIn = new Vector3(wp.x - prev.x, 0, wp.z - prev.z).normalized;
                    Vector3 dirOut = new Vector3(next.x - wp.x, 0, next.z - wp.z).normalized;
                    float angle = Vector3.Angle(dirIn, dirOut);
                    if (angle > 120f) angleWarning = " [!!]";
                    else if (angle > 90f) angleWarning = " [!]";
                }

                GUILayout.BeginHorizontal();
                GUILayout.Label($"#{i + 1}", wpLabelStyle, GUILayout.Width(25));

                // Distance to next waypoint
                string distStr = "";
                if (i < currentRoute.waypoints.Count - 1)
                {
                    float d = FastMath.Distance(wp.ToGlobalPosition(), currentRoute.waypoints[i + 1].ToGlobalPosition());
                    distStr = d >= 1000 ? $" {d / 1000f:F1}km" : $" {d:F0}m";
                }

                GUILayout.Label($"({wp.x:F0},{wp.z:F0}) alt:{wp.y:F0}{distStr}{angleWarning}", wpLabelStyle, GUILayout.Width(230));
                if (GUILayout.Button("^", smallButtonStyle, GUILayout.Width(22)) && i > 0)
                {
                    currentRoute.waypoints.RemoveAt(i);
                    currentRoute.waypoints.Insert(i - 1, wp);
                }
                if (GUILayout.Button("X", smallButtonStyle, GUILayout.Width(22)))
                {
                    currentRoute.waypoints.RemoveAt(i);
                    break;
                }
                GUILayout.EndHorizontal();
            }
            GUILayout.EndScrollView();

            GUILayout.Space(4);

            // Add waypoint
            GUILayout.Label("Add Waypoint", labelStyle);
            GUILayout.BeginHorizontal();
            GUILayout.Label("Alt:", wpLabelStyle, GUILayout.Width(28));
            altitudeText = GUILayout.TextField(altitudeText, textFieldStyle, GUILayout.Width(55));
            float.TryParse(altitudeText, out waypointAltitude);
            GUILayout.Label("Spd:", wpLabelStyle, GUILayout.Width(30));
            speedText = GUILayout.TextField(speedText, textFieldStyle, GUILayout.Width(55));
            float.TryParse(speedText, out waypointSpeed);
            GUILayout.Label("(0=auto)", wpLabelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Click Map", buttonStyle))
                placingWaypoint = true;
            if (GUILayout.Button("At Camera", buttonStyle))
            {
                var cam = Camera.main;
                if (cam != null)
                {
                    Vector3 camPos = cam.transform.position;
                    GlobalPosition gp = camPos.ToGlobalPosition();
                    float y;
                    if (currentRoute.aglMode)
                    {
                        float groundH = WaypointManager.GetGroundHeight(camPos);
                        y = camPos.y - groundH;
                        if (y < 1f) y = 1f;
                    }
                    else
                    {
                        y = gp.y;
                    }
                    AddWaypointToCurrentRoute(gp.x, y, gp.z);
                }
            }
            GUILayout.EndHorizontal();

            // Camera recording
            GUILayout.BeginHorizontal();
            string recLabel = recording ? ">> RECORDING <<" : "Record Camera";
            var recStyle = recording ? activeButtonStyle : buttonStyle;
            if (GUILayout.Button(recLabel, recStyle))
            {
                if (recording)
                {
                    // Stopping recording — auto-save
                    recording = false;
                    if (!string.IsNullOrEmpty(saveNameText))
                        WaypointFileIO.SaveMissionData(saveNameText);
                    Plugin.Log?.LogInfo($"Recording stopped. {currentRoute.waypoints.Count} waypoints saved.");
                }
                else
                {
                    recording = true;
                    recordTimer = 0f;
                }
            }
            GUILayout.Label("Interval:", wpLabelStyle, GUILayout.Width(52));
            recordIntervalText = GUILayout.TextField(recordIntervalText, textFieldStyle, GUILayout.Width(35));
            float.TryParse(recordIntervalText, out recordInterval);
            if (recordInterval < 0.1f) recordInterval = 0.1f;
            GUILayout.Label("s", wpLabelStyle, GUILayout.Width(12));
            GUILayout.EndHorizontal();

            if (GUILayout.Button("Clear All Waypoints", buttonStyle))
                currentRoute.waypoints.Clear();
        }

        // ---- Tab 2: Save/Load ----
        private void DrawFileTab()
        {
            GUILayout.Label("Save / Load", headerStyle);
            GUILayout.Label("Save routes and assignments for reuse across missions.", wpLabelStyle);

            GUILayout.Space(4);

            // Save
            GUILayout.BeginHorizontal();
            GUILayout.Label("Name:", labelStyle, GUILayout.Width(45));
            saveNameText = GUILayout.TextField(saveNameText, textFieldStyle);
            GUILayout.EndHorizontal();

            string summary = $"{WaypointManager.routeLibrary.Count} routes, {WaypointManager.assignmentsByName.Count} assignments";
            if (GUILayout.Button($"Save ({summary})", buttonStyle))
                WaypointFileIO.SaveMissionData(saveNameText);

            GUILayout.Space(8);

            // Load
            GUILayout.Label("Saved Files", labelStyle);
            savedFiles = WaypointFileIO.GetSavedFiles();

            if (savedFiles.Count == 0)
            {
                GUILayout.Label($"No saved files in {WaypointFileIO.GetDataFolder()}", wpLabelStyle);
            }
            else
            {
                foreach (var file in savedFiles)
                {
                    string fname = Path.GetFileNameWithoutExtension(file);
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(fname, wpLabelStyle, GUILayout.Width(200));
                    if (GUILayout.Button("Load", smallButtonStyle, GUILayout.Width(50)))
                    {
                        WaypointFileIO.LoadMissionData(fname);
                        saveNameText = fname;
                    }
                    if (GUILayout.Button("Del", smallButtonStyle, GUILayout.Width(40)))
                    {
                        try { File.Delete(file); } catch { }
                    }
                    GUILayout.EndHorizontal();
                }
            }
        }

        // ---- Tab 3: Camera ----
        private void DrawCameraTab()
        {
            GUILayout.Label("Camera Routes", headerStyle);

            // Camera route list
            camRouteScroll = GUILayout.BeginScrollView(camRouteScroll, GUILayout.Height(70));
            for (int i = 0; i < CameraPlayback.cameraRouteLibrary.Count; i++)
            {
                var r = CameraPlayback.cameraRouteLibrary[i];
                bool isSel = i == CameraPlayback.selectedCameraRouteIndex;
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(isSel ? $"> {r.name} ({r.waypoints.Count})" : $"  {r.name} ({r.waypoints.Count})",
                    isSel ? activeButtonStyle : buttonStyle))
                {
                    CameraPlayback.selectedCameraRouteIndex = i;
                    camSpeedText = r.speed.ToString("F0");
                }
                if (GUILayout.Button("X", smallButtonStyle, GUILayout.Width(25)))
                {
                    if (CameraPlayback.activeRoute == r) CameraPlayback.StopPlayback();
                    CameraPlayback.cameraRouteLibrary.RemoveAt(i);
                    if (CameraPlayback.selectedCameraRouteIndex >= CameraPlayback.cameraRouteLibrary.Count)
                        CameraPlayback.selectedCameraRouteIndex = CameraPlayback.cameraRouteLibrary.Count - 1;
                    break;
                }
                GUILayout.EndHorizontal();
            }
            GUILayout.EndScrollView();

            if (GUILayout.Button("+ New Camera Route", buttonStyle))
            {
                var route = new CameraRoute { name = "Cam " + (CameraPlayback.cameraRouteLibrary.Count + 1) };
                CameraPlayback.cameraRouteLibrary.Add(route);
                CameraPlayback.selectedCameraRouteIndex = CameraPlayback.cameraRouteLibrary.Count - 1;
            }

            GUILayout.Space(4);

            if (CameraPlayback.selectedCameraRouteIndex < 0 ||
                CameraPlayback.selectedCameraRouteIndex >= CameraPlayback.cameraRouteLibrary.Count)
            {
                GUILayout.Label("Select or create a camera route", wpLabelStyle);
                return;
            }

            var currentRoute = CameraPlayback.cameraRouteLibrary[CameraPlayback.selectedCameraRouteIndex];

            // Route settings
            GUILayout.BeginHorizontal();
            GUILayout.Label("Name:", labelStyle, GUILayout.Width(40));
            string newName = GUILayout.TextField(currentRoute.name, textFieldStyle);
            if (newName != currentRoute.name) currentRoute.name = newName;
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Mode:", labelStyle, GUILayout.Width(40));
            if (GUILayout.Button("Loop", currentRoute.Mode == RouteMode.Loop ? activeButtonStyle : buttonStyle))
                currentRoute.Mode = RouteMode.Loop;
            if (GUILayout.Button("One", currentRoute.Mode == RouteMode.OneShot ? activeButtonStyle : buttonStyle))
                currentRoute.Mode = RouteMode.OneShot;
            if (GUILayout.Button("PgPg", currentRoute.Mode == RouteMode.PingPong ? activeButtonStyle : buttonStyle))
                currentRoute.Mode = RouteMode.PingPong;
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Speed:", labelStyle, GUILayout.Width(40));
            camSpeedText = GUILayout.TextField(camSpeedText, textFieldStyle, GUILayout.Width(50));
            float.TryParse(camSpeedText, out float spd);
            if (spd < 1f) spd = 1f;
            currentRoute.speed = spd;
            GUILayout.Label("m/s", wpLabelStyle, GUILayout.Width(30));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            currentRoute.lookAtTarget = GUILayout.Toggle(currentRoute.lookAtTarget, " LookAt", GUILayout.Width(70));
            currentRoute.autoSpeed = GUILayout.Toggle(currentRoute.autoSpeed, " AutoSpd", GUILayout.Width(75));
            GUILayout.EndHorizontal();

            // Target unit selector
            if (currentRoute.lookAtTarget || currentRoute.autoSpeed)
            {
                GUILayout.BeginHorizontal();
                string targetName = CameraPlayback.cachedLookAtUnit != null
                    ? (CameraPlayback.cachedLookAtUnit.definition != null
                        ? CameraPlayback.cachedLookAtUnit.definition.name
                        : CameraPlayback.cachedLookAtUnit.name)
                    : "None";
                GUILayout.Label($"Target: {targetName}", wpLabelStyle);
                if (GUILayout.Button(showTargetPicker ? "Close" : "Select", smallButtonStyle, GUILayout.Width(50)))
                    showTargetPicker = !showTargetPicker;
                GUILayout.EndHorizontal();

                if (showTargetPicker)
                {
                    camTargetScroll = GUILayout.BeginScrollView(camTargetScroll, GUILayout.Height(100));
                    var allUnits = UnityEngine.Object.FindObjectsOfType<Unit>();

                    // Build display list with identifiers
                    var unitList = new List<(Unit unit, string display)>();
                    var typeCount = new Dictionary<string, int>();
                    foreach (var unit in allUnits)
                    {
                        if (unit == null || unit.disabled) continue;
                        string typeName = unit.definition != null ? unit.definition.name : "Unit";

                        // Get unique identifier
                        string uid = "";
                        if (unit.SavedUnit != null && !string.IsNullOrEmpty(unit.SavedUnit.UniqueName))
                            uid = unit.SavedUnit.UniqueName;
                        else if (unit is Aircraft ac && !string.IsNullOrEmpty(ac.UniqueName))
                            uid = ac.UniqueName;

                        // Auto-assign index if no unique name
                        if (string.IsNullOrEmpty(uid))
                        {
                            if (!typeCount.ContainsKey(typeName)) typeCount[typeName] = 0;
                            typeCount[typeName]++;
                            uid = $"#{typeCount[typeName]}";
                        }

                        unitList.Add((unit, $"{typeName} ({uid})"));
                    }

                    // Sort: aircraft first, then by name
                    unitList.Sort((a, b) =>
                    {
                        bool aIsAc = a.unit is Aircraft;
                        bool bIsAc = b.unit is Aircraft;
                        if (aIsAc != bIsAc) return aIsAc ? -1 : 1;
                        return string.Compare(a.display, b.display, StringComparison.Ordinal);
                    });

                    foreach (var (unit, display) in unitList)
                    {
                        bool isCurrent = CameraPlayback.cachedLookAtUnit == unit;
                        if (GUILayout.Button(isCurrent ? $"> {display}" : $"  {display}",
                            isCurrent ? activeButtonStyle : buttonStyle))
                        {
                            CameraPlayback.cachedLookAtUnit = unit;
                            showTargetPicker = false;
                        }
                    }
                    GUILayout.EndScrollView();
                }
            }

            GUILayout.Space(2);

            // Camera waypoints list
            GUILayout.Label($"Camera Points ({currentRoute.waypoints.Count})", labelStyle);
            camWaypointScroll = GUILayout.BeginScrollView(camWaypointScroll, GUILayout.Height(80));
            for (int i = 0; i < currentRoute.waypoints.Count; i++)
            {
                var cwp = currentRoute.waypoints[i];
                GUILayout.BeginHorizontal();
                GUILayout.Label($"#{i + 1}", wpLabelStyle, GUILayout.Width(25));

                string distStr = "";
                if (i < currentRoute.waypoints.Count - 1)
                {
                    var next = currentRoute.waypoints[i + 1];
                    float d = Vector3.Distance(
                        new GlobalPosition(cwp.x, cwp.y, cwp.z).ToLocalPosition(),
                        new GlobalPosition(next.x, next.y, next.z).ToLocalPosition());
                    distStr = d >= 1000 ? $" {d / 1000f:F1}km" : $" {d:F0}m";
                }

                GUILayout.Label($"fov:{cwp.fov:F0}{distStr}", wpLabelStyle, GUILayout.Width(180));
                if (GUILayout.Button("^", smallButtonStyle, GUILayout.Width(22)) && i > 0)
                {
                    currentRoute.waypoints.RemoveAt(i);
                    currentRoute.waypoints.Insert(i - 1, cwp);
                }
                if (GUILayout.Button("X", smallButtonStyle, GUILayout.Width(22)))
                {
                    currentRoute.waypoints.RemoveAt(i);
                    break;
                }
                GUILayout.EndHorizontal();
            }
            GUILayout.EndScrollView();

            GUILayout.Space(2);

            // Add camera waypoint
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Add At Camera", buttonStyle))
            {
                var cam = Camera.main;
                if (cam != null)
                {
                    Vector3 pos = cam.transform.position;
                    Vector3 euler = cam.transform.eulerAngles;
                    GlobalPosition gp = pos.ToGlobalPosition();
                    currentRoute.waypoints.Add(new CameraWaypoint
                    {
                        x = gp.x, y = gp.y, z = gp.z,
                        rx = euler.x, ry = euler.y, rz = euler.z,
                        fov = cam.fieldOfView
                    });
                }
            }
            if (GUILayout.Button("Clear", smallButtonStyle, GUILayout.Width(50)))
                currentRoute.waypoints.Clear();
            GUILayout.EndHorizontal();

            // Camera recording
            GUILayout.BeginHorizontal();
            string camRecLabel = camRecording ? ">> REC <<" : "Record Path";
            if (GUILayout.Button(camRecLabel, camRecording ? activeButtonStyle : buttonStyle))
            {
                if (camRecording)
                {
                    camRecording = false;
                    Plugin.Log?.LogInfo($"Camera recording stopped. {currentRoute.waypoints.Count} points.");
                }
                else
                {
                    camRecording = true;
                    camRecordTimer = 0f;
                }
            }
            GUILayout.Label("Int:", wpLabelStyle, GUILayout.Width(25));
            camRecordIntervalText = GUILayout.TextField(camRecordIntervalText, textFieldStyle, GUILayout.Width(35));
            float.TryParse(camRecordIntervalText, out camRecordInterval);
            if (camRecordInterval < 0.05f) camRecordInterval = 0.05f;
            GUILayout.Label("s", wpLabelStyle, GUILayout.Width(12));
            GUILayout.EndHorizontal();

            GUILayout.Space(6);

            // ---- Playback Controls ----
            GUILayout.Label("Playback", headerStyle);

            // Progress bar
            if (CameraPlayback.playing)
            {
                float progress = CameraPlayback.GetProgress();
                Rect progRect = GUILayoutUtility.GetRect(0, 12, GUILayout.ExpandWidth(true));
                GUI.color = new Color(0.2f, 0.2f, 0.2f, 1f);
                GUI.DrawTexture(progRect, Texture2D.whiteTexture);
                GUI.color = new Color(0.3f, 0.8f, 0.4f, 1f);
                GUI.DrawTexture(new Rect(progRect.x, progRect.y, progRect.width * progress, progRect.height), Texture2D.whiteTexture);
                GUI.color = Color.white;
                GUI.Label(progRect, $" {progress:P0} seg {CameraPlayback.currentSegment + 1}/{currentRoute.waypoints.Count}", wpLabelStyle);
            }

            GUILayout.BeginHorizontal();
            bool isPlaying = CameraPlayback.playing && CameraPlayback.activeRoute == currentRoute;
            if (!isPlaying)
            {
                if (GUILayout.Button("Play", buttonStyle))
                    CameraPlayback.StartPlayback(currentRoute);
            }
            else
            {
                if (GUILayout.Button(CameraPlayback.paused ? "Resume" : "Pause", buttonStyle))
                    CameraPlayback.TogglePause();
                if (GUILayout.Button("Stop", buttonStyle))
                    CameraPlayback.StopPlayback();
            }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Speed:", wpLabelStyle, GUILayout.Width(40));
            camPlaybackSpeedText = GUILayout.TextField(camPlaybackSpeedText, textFieldStyle, GUILayout.Width(40));
            float.TryParse(camPlaybackSpeedText, out float pbSpeed);
            if (pbSpeed < 0.1f) pbSpeed = 0.1f;
            CameraPlayback.playbackSpeed = pbSpeed;
            GUILayout.Label("x", wpLabelStyle, GUILayout.Width(12));
            if (GUILayout.Button("0.5x", smallButtonStyle)) { CameraPlayback.playbackSpeed = 0.5f; camPlaybackSpeedText = "0.5"; }
            if (GUILayout.Button("1x", smallButtonStyle)) { CameraPlayback.playbackSpeed = 1f; camPlaybackSpeedText = "1"; }
            if (GUILayout.Button("2x", smallButtonStyle)) { CameraPlayback.playbackSpeed = 2f; camPlaybackSpeedText = "2"; }
            GUILayout.EndHorizontal();
        }

        // ---- Markers ----
        private Texture2D lineTex;

        private Color GetRouteColor(int index, bool selected)
        {
            Color c = routeColors[index % routeColors.Length];
            if (!selected) c.a *= 0.4f; // dim non-selected
            return c;
        }

        private Color GetCamRouteColor(int index, bool selected)
        {
            Color c = camRouteColors[index % camRouteColors.Length];
            if (!selected) c.a *= 0.4f;
            return c;
        }

        private void EnsureLineTex()
        {
            if (lineTex == null)
            {
                lineTex = new Texture2D(1, 1);
                lineTex.SetPixel(0, 0, Color.white);
                lineTex.Apply();
            }
        }

        // Draw ALL aircraft routes with per-route colors
        private void DrawAllRouteMarkers()
        {
            var cam = Camera.main;
            if (cam == null) return;
            EnsureLineTex();

            int bottomLabelY = 0; // stack bottom labels

            for (int ri = 0; ri < WaypointManager.routeLibrary.Count; ri++)
            {
                var route = WaypointManager.routeLibrary[ri];
                if (route.waypoints.Count == 0) continue;

                bool isSelected = ri == WaypointManager.selectedRouteIndex;
                Color baseColor = GetRouteColor(ri, isSelected);
                float lineWidth = isSelected ? 2.5f : 1.5f;
                float boxSize = isSelected ? 7 : 5;

                // Collect screen positions
                var screenPoints = new List<(Vector3 screen, bool visible)>();
                for (int i = 0; i < route.waypoints.Count; i++)
                {
                    Vector3 localPos = route.waypoints[i].ToGlobalPosition().ToLocalPosition();
                    if (route.aglMode)
                    {
                        float groundH = WaypointManager.GetGroundHeight(localPos);
                        localPos.y = groundH + route.waypoints[i].y;
                    }
                    Vector3 sp = cam.WorldToScreenPoint(localPos);
                    screenPoints.Add((sp, sp.z > 0));
                }

                // Lines between waypoints
                for (int i = 0; i < screenPoints.Count - 1; i++)
                {
                    var a = screenPoints[i];
                    var b = screenPoints[i + 1];
                    if (a.visible && b.visible)
                    {
                        float ay = Screen.height - a.screen.y;
                        float by = Screen.height - b.screen.y;
                        GUI.color = baseColor;
                        DrawLine(new Vector2(a.screen.x, ay), new Vector2(b.screen.x, by), lineWidth);

                        // Distance label (selected only, to avoid clutter)
                        if (isSelected)
                        {
                            float dist = FastMath.Distance(route.waypoints[i].ToGlobalPosition(), route.waypoints[i + 1].ToGlobalPosition());
                            float mx = (a.screen.x + b.screen.x) / 2f;
                            float my = (ay + by) / 2f;
                            string distLabel = dist >= 1000 ? $"{dist / 1000f:F1}km" : $"{dist:F0}m";
                            GUI.color = new Color(0, 0, 0, 0.7f);
                            GUI.Label(new Rect(mx - 9, my - 9, 60, 18), distLabel);
                            GUI.color = baseColor;
                            GUI.Label(new Rect(mx - 10, my - 10, 60, 18), distLabel);
                        }
                    }
                }

                // Loop line
                if (route.Mode == RouteMode.Loop && screenPoints.Count > 1)
                {
                    var a = screenPoints[screenPoints.Count - 1];
                    var b = screenPoints[0];
                    if (a.visible && b.visible)
                    {
                        Color loopColor = baseColor; loopColor.a *= 0.5f;
                        GUI.color = loopColor;
                        DrawLine(new Vector2(a.screen.x, Screen.height - a.screen.y),
                                 new Vector2(b.screen.x, Screen.height - b.screen.y), 1f);
                    }
                }

                // Waypoint markers
                for (int i = 0; i < screenPoints.Count; i++)
                {
                    if (!screenPoints[i].visible) continue;
                    float sx = screenPoints[i].screen.x;
                    float sy = Screen.height - screenPoints[i].screen.y;

                    // Selected route: full detail (altitude lines, labels)
                    if (isSelected)
                    {
                        var wp = route.waypoints[i];
                        Vector3 wpLocal = wp.ToGlobalPosition().ToLocalPosition();
                        Vector3 groundLocal;
                        string altText;
                        if (route.aglMode)
                        {
                            float groundH = WaypointManager.GetGroundHeight(wpLocal);
                            groundLocal = new Vector3(wpLocal.x, groundH, wpLocal.z);
                            altText = $"AGL {wp.y:F0}m";
                        }
                        else
                        {
                            groundLocal = new GlobalPosition(wp.x, 0, wp.z).ToLocalPosition();
                            altText = $"{wp.y:F0}m";
                        }
                        Vector3 gs = cam.WorldToScreenPoint(groundLocal);
                        if (gs.z > 0)
                        {
                            float gsy = Screen.height - gs.y;
                            Color altColor = baseColor; altColor.a *= 0.4f;
                            GUI.color = altColor;
                            DrawLine(new Vector2(sx, sy), new Vector2(gs.x, gsy), 1f);
                            float midY = (sy + gsy) / 2f;
                            GUI.color = new Color(0, 0, 0, 0.6f);
                            GUI.Label(new Rect(sx + 6, midY - 7, 80, 16), altText);
                            GUI.color = baseColor;
                            GUI.Label(new Rect(sx + 5, midY - 8, 80, 16), altText);
                        }
                    }

                    // WP label
                    string label = isSelected ? $"WP{i + 1}" : $"{route.name[0]}{i + 1}";
                    GUI.color = new Color(0, 0, 0, 0.8f);
                    GUI.Label(new Rect(sx - 14, sy - 21, 80, 20), label);
                    GUI.color = baseColor;
                    GUI.Label(new Rect(sx - 15, sy - 22, 80, 20), label);

                    // Box
                    GUI.DrawTexture(new Rect(sx - boxSize / 2, sy - boxSize / 2, boxSize, boxSize), Texture2D.whiteTexture);
                }

                // Route name + total distance at bottom
                if (route.waypoints.Count > 1)
                {
                    float totalDist = 0;
                    for (int i = 0; i < route.waypoints.Count - 1; i++)
                        totalDist += FastMath.Distance(route.waypoints[i].ToGlobalPosition(), route.waypoints[i + 1].ToGlobalPosition());
                    if (route.Mode == RouteMode.Loop)
                        totalDist += FastMath.Distance(route.waypoints[route.waypoints.Count - 1].ToGlobalPosition(), route.waypoints[0].ToGlobalPosition());

                    string totalLabel = totalDist >= 1000 ? $"{route.name}: {totalDist / 1000f:F1}km" : $"{route.name}: {totalDist:F0}m";
                    float ly = Screen.height - 40 - bottomLabelY * 16;
                    GUI.color = new Color(0, 0, 0, 0.7f);
                    GUI.Label(new Rect(11, ly + 1, 250, 20), totalLabel);
                    GUI.color = baseColor;
                    GUI.Label(new Rect(10, ly, 250, 20), totalLabel);
                    bottomLabelY++;
                }
            }
            GUI.color = Color.white;
        }

        // Draw ALL camera routes with per-route colors + direction indicators
        private void DrawAllCameraRouteMarkers()
        {
            var cam = Camera.main;
            if (cam == null) return;
            EnsureLineTex();

            for (int ri = 0; ri < CameraPlayback.cameraRouteLibrary.Count; ri++)
            {
                var route = CameraPlayback.cameraRouteLibrary[ri];
                if (route.waypoints.Count == 0) continue;

                bool isSelected = ri == CameraPlayback.selectedCameraRouteIndex;
                Color baseColor = GetCamRouteColor(ri, isSelected);
                float lineWidth = isSelected ? 2.5f : 1.5f;

                // Collect screen positions
                var screenPoints = new List<(Vector3 screen, bool visible, Vector3 localPos)>();
                for (int i = 0; i < route.waypoints.Count; i++)
                {
                    var cwp = route.waypoints[i];
                    Vector3 localPos = new GlobalPosition(cwp.x, cwp.y, cwp.z).ToLocalPosition();
                    Vector3 sp = cam.WorldToScreenPoint(localPos);
                    screenPoints.Add((sp, sp.z > 0, localPos));
                }

                // Lines between waypoints
                for (int i = 0; i < screenPoints.Count - 1; i++)
                {
                    var a = screenPoints[i];
                    var b = screenPoints[i + 1];
                    if (a.visible && b.visible)
                    {
                        float ay = Screen.height - a.screen.y;
                        float by = Screen.height - b.screen.y;
                        GUI.color = baseColor;
                        DrawLine(new Vector2(a.screen.x, ay), new Vector2(b.screen.x, by), lineWidth);
                    }
                }

                // Loop line
                if (route.Mode == RouteMode.Loop && screenPoints.Count > 1)
                {
                    var a = screenPoints[screenPoints.Count - 1];
                    var b = screenPoints[0];
                    if (a.visible && b.visible)
                    {
                        Color loopColor = baseColor; loopColor.a *= 0.5f;
                        GUI.color = loopColor;
                        DrawLine(new Vector2(a.screen.x, Screen.height - a.screen.y),
                                 new Vector2(b.screen.x, Screen.height - b.screen.y), 1f);
                    }
                }

                // Camera waypoint markers + direction arrow
                for (int i = 0; i < screenPoints.Count; i++)
                {
                    if (!screenPoints[i].visible) continue;
                    var cwp = route.waypoints[i];
                    float sx = screenPoints[i].screen.x;
                    float sy = Screen.height - screenPoints[i].screen.y;

                    // Camera icon: triangle (shows direction)
                    GUI.color = baseColor;
                    float triSize = isSelected ? 8 : 5;
                    GUI.DrawTexture(new Rect(sx - triSize / 2, sy - triSize / 2, triSize, triSize), Texture2D.whiteTexture);

                    // Direction line: short line showing where camera is pointing
                    if (isSelected)
                    {
                        Vector3 fwd = Quaternion.Euler(cwp.rx, cwp.ry, cwp.rz) * Vector3.forward;
                        Vector3 dirEnd = screenPoints[i].localPos + fwd * 30f;
                        Vector3 dirScreen = cam.WorldToScreenPoint(dirEnd);
                        if (dirScreen.z > 0)
                        {
                            Color dirColor = baseColor; dirColor.a *= 0.6f;
                            GUI.color = dirColor;
                            DrawLine(new Vector2(sx, sy), new Vector2(dirScreen.x, Screen.height - dirScreen.y), 1f);
                        }

                        // Label
                        string label = $"C{i + 1}";
                        GUI.color = new Color(0, 0, 0, 0.8f);
                        GUI.Label(new Rect(sx - 12, sy - 21, 80, 20), label);
                        GUI.color = baseColor;
                        GUI.Label(new Rect(sx - 13, sy - 22, 80, 20), label);
                    }
                }
            }
            GUI.color = Color.white;
        }

        private void DrawAircraftMarkers()
        {
            var cam = Camera.main;
            if (cam == null) return;

            var allAircraft = UnityEngine.Object.FindObjectsOfType<Aircraft>();
            foreach (var ac in allAircraft)
            {
                if (ac == null || ac.disabled) continue;

                string uname = "";
                if (ac.SavedUnit != null)
                    uname = ac.SavedUnit.UniqueName ?? "";
                else if (!string.IsNullOrEmpty(ac.UniqueName))
                    uname = ac.UniqueName;
                if (string.IsNullOrEmpty(uname)) continue;

                bool isSelected = WaypointManager.selectedUniqueName == uname;
                bool hasRoute = WaypointManager.assignmentsByName.ContainsKey(uname);
                if (!isSelected && !hasRoute) continue;

                Vector3 sp = cam.WorldToScreenPoint(ac.transform.position);
                if (sp.z <= 0) continue;
                float sx = sp.x;
                float sy = Screen.height - sp.y;

                string dname = ac.definition != null ? ac.definition.name : "Aircraft";

                if (isSelected)
                {
                    // Selected: bright cyan diamond + name
                    float size = 14;
                    GUI.color = new Color(0, 1f, 1f, 0.9f);
                    // Draw diamond shape using 4 small rects rotated
                    GUI.DrawTexture(new Rect(sx - size / 2, sy - 1, size, 2), Texture2D.whiteTexture);
                    GUI.DrawTexture(new Rect(sx - 1, sy - size / 2, 2, size), Texture2D.whiteTexture);
                    // Corner box
                    GUI.DrawTexture(new Rect(sx - size / 2 - 1, sy - size / 2 - 1, size + 2, 2), Texture2D.whiteTexture);
                    GUI.DrawTexture(new Rect(sx - size / 2 - 1, sy + size / 2 - 1, size + 2, 2), Texture2D.whiteTexture);
                    GUI.DrawTexture(new Rect(sx - size / 2 - 1, sy - size / 2 - 1, 2, size + 2), Texture2D.whiteTexture);
                    GUI.DrawTexture(new Rect(sx + size / 2 - 1, sy - size / 2 - 1, 2, size + 2), Texture2D.whiteTexture);

                    // Label
                    string selLabel = $">> {dname} <<";
                    GUI.color = new Color(0, 0, 0, 0.8f);
                    GUI.Label(new Rect(sx - 59, sy - 27, 160, 20), selLabel);
                    GUI.color = new Color(0, 1f, 1f, 1f);
                    GUI.Label(new Rect(sx - 60, sy - 28, 160, 20), selLabel);
                }
                else if (hasRoute)
                {
                    // Assigned but not selected: small green dot + short label
                    float dotSize = 6;
                    GUI.color = new Color(0.4f, 1f, 0.4f, 0.7f);
                    GUI.DrawTexture(new Rect(sx - dotSize / 2, sy - dotSize / 2, dotSize, dotSize), Texture2D.whiteTexture);

                    WaypointManager.formationOffsets.TryGetValue(uname, out var off);
                    string offTag = off != null ? $" R:{off.right:F0}" : "";
                    GUI.color = new Color(0, 0, 0, 0.6f);
                    GUI.Label(new Rect(sx + 5, sy - 9, 120, 18), $"{dname}{offTag}");
                    GUI.color = new Color(0.5f, 1f, 0.5f, 0.8f);
                    GUI.Label(new Rect(sx + 4, sy - 10, 120, 18), $"{dname}{offTag}");
                }
            }
            GUI.color = Color.white;
        }

        private void DrawLine(Vector2 a, Vector2 b, float width)
        {
            Vector2 delta = b - a;
            float length = delta.magnitude;
            if (length < 1f) return;

            float angle = Mathf.Atan2(delta.y, delta.x) * Mathf.Rad2Deg;

            var pivot = new Vector2(a.x, a.y);
            var matrix = GUI.matrix;
            GUIUtility.RotateAroundPivot(angle, pivot);
            GUI.DrawTexture(new Rect(a.x, a.y - width / 2f, length, width), lineTex);
            GUI.matrix = matrix;
        }
    }

    // ========== FRAME HELPER ==========

    internal class AWFrameHelper : MonoBehaviour
    {
        private void Update()
        {
            Plugin.Instance?.waypointUI?.HandleInput();
            CameraPlayback.UpdatePlayback();
        }

        private void OnGUI()
        {
            Plugin.Instance?.waypointUI?.Draw();
        }
    }

    // ========== PLUGIN ==========

    [BepInPlugin("com.noms.aircraftwaypoints", "Aircraft Waypoints", "2.1.0")]
    public class Plugin : BaseUnityPlugin
    {
        internal static ManualLogSource Log;
        internal static Plugin Instance;
        private Harmony harmony;

        internal WaypointUI waypointUI;
        internal ConfigEntry<KeyCode> toggleKey;

        internal ConfigEntry<float> arrivalRadius;
        internal ConfigEntry<float> turnSmoothness;

        private void Awake()
        {
            Log = Logger;
            Instance = this;

            toggleKey = Config.Bind("Controls", "ToggleKey", KeyCode.F5, "Key to toggle waypoint UI");

            arrivalRadius = Config.Bind("Flight", "ArrivalRadius", 500f, "Distance (m) to waypoint before advancing to next");
            turnSmoothness = Config.Bind("Flight", "TurnSmoothness", 0.5f, "AutoAim effort (0.2=smooth cinematic, 1.0=aggressive)");

            harmony = new Harmony("com.noms.aircraftwaypoints");
            harmony.PatchAll();

            WaypointFileIO.Initialize();
            waypointUI = new WaypointUI();

            try
            {
                var helperGo = new GameObject("AircraftWaypoints_FrameHelper");
                UnityEngine.Object.DontDestroyOnLoad(helperGo);
                helperGo.hideFlags = HideFlags.HideAndDontSave;
                helperGo.AddComponent<AWFrameHelper>();
            }
            catch (Exception e)
            {
                Log.LogWarning($"FrameHelper failed: {e.Message}");
            }

            SceneManager.sceneLoaded += OnSceneLoaded;
            Log.LogInfo("Aircraft Waypoints v2.0.0 loaded — F5 to toggle UI (Camera system added)");
        }

        private void OnSceneLoaded(UnityEngine.SceneManagement.Scene scene, LoadSceneMode mode)
        {
            WaypointManager.ClearRuntime();
        }

        private void OnDestroy()
        {
            harmony?.UnpatchSelf();
            SceneManager.sceneLoaded -= OnSceneLoaded;
        }
    }
}
