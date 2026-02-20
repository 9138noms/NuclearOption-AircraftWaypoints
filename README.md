# Aircraft Waypoints

Custom waypoint routes for AI aircraft in Nuclear Option. Create cinematic flight paths, low-altitude terrain runs, formation flights, and patrol routes.

## Features

- **Waypoint Route Editor** — Create routes with unlimited waypoints, set per-waypoint altitude and speed
- **AGL Mode** — Altitude calculated above ground level (terrain-relative), not sea level
- **Raw Follow** — Disable terrain avoidance for exact path following (cinematic low-altitude runs)
- **Camera Recording** — Fly the camera along a path and auto-place waypoints at configurable intervals
- **Formation Flying** — Assign multiple aircraft to the same route with Right/Up/Forward offsets
- **Route Modes** — Loop, OneShot, PingPong
- **Auto Refuel** — Aircraft RTB on low fuel, refuel, then resume waypoint route (patrol)
- **Auto Turn Braking** — Automatically slows down for sharp turns
- **Visual Markers** — On-screen waypoint markers with distance, altitude lines, and route total distance
- **Save/Load** — Save routes to JSON files, load across sessions
- **Auto-save on mission save** — Waypoint data automatically saved when you save a mission

## Controls

| Key | Action |
|-----|--------|
| F5 | Toggle waypoint UI |

## Installation

1. Install [BepInEx 5.x](https://github.com/BepInEx/BepInEx) (Unity Mono)
2. Copy `AircraftWaypoints.dll` to `[Game Folder]\BepInEx\plugins\`

## Usage

1. Open the Mission Editor and place AI aircraft
2. Press **F5** to open the Waypoint UI
3. Go to **Routes** tab → create a new route
4. Add waypoints via **Click Map**, **At Camera**, or **Record Camera**
5. Go to **Aircraft** tab → select an aircraft → assign the route
6. Click **Play** to see the aircraft follow the route

## Configuration

Edit `BepInEx/config/com.noms.aircraftwaypoints.cfg`:
- `ToggleKey` — UI toggle key (default: F5)
- `ArrivalRadius` — Distance to waypoint before advancing (default: 500m)
- `TurnSmoothness` — Flight smoothness, 0.2 = cinematic, 1.0 = aggressive (default: 0.5)

## Notes

- **Singleplayer / Host only** — AI logic runs on the server (host)
- **Waypoint tracking is not perfect** — AI autopilot has its own flight characteristics and may not follow paths exactly. Use wider turns and higher altitudes for more reliable results
- **Raw mode + low altitude = crash risk** — Terrain avoidance is disabled, ensure your path clears all obstacles
- Routes persist in memory across Play→Return and editor sessions. Use Save/Load tab for permanent storage

## License

MIT
