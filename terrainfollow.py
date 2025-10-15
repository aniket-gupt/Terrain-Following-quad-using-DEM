import asyncio
import json
import time
import math

import numpy as np
import rasterio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from pyproj import Transformer

# Configuration
DEM_FILE = "output_be.tif"
MISSION_PLAN = "old5.plan"
TARGET_AGL = 40.0  # Fly 40 meters above ground
CRUISE_SPEED = 12.0
CHECK_INTERVAL = 3.0  # Check every 3 seconds
MIN_UPDATE_GAP = 10.0  # Wait 10s between mission updates

class TerrainMap:
    def __init__(self, dem_file):
        """Load and prepare the digital elevation model"""
        self.dataset = rasterio.open(dem_file)
        self.coord_converter = Transformer.from_crs(
            "epsg:4326", self.dataset.crs, always_xy=True
        )
        self.elevation_data = self.dataset.read(1)
        print(f"Terrain map loaded: {self.elevation_data.shape}")

    def get_ground_height(self, lat, lon):
        """Get ground elevation at GPS coordinates"""
        try:
            x, y = self.coord_converter.transform(lon, lat)
            row, col = self.dataset.index(x, y)
            row = np.clip(row, 0, self.elevation_data.shape[0] - 1)
            col = np.clip(col, 0, self.elevation_data.shape[1] - 1)
            return float(self.elevation_data[row, col])
        except:
            return 0.0

def load_mission_points(plan_file):
    """Read mission waypoints from file"""
    with open(plan_file, "r") as f:
        mission_data = json.load(f)

    points = []
    for item in mission_data["mission"]["items"]:
        if item["type"] != "SimpleItem":
            continue
        lat = item["params"][4]
        lon = item["params"][5]
        points.append((lat, lon))

    print(f"Mission loaded: {len(points)} waypoints")
    return points

def create_waypoint(lat, lon, rel_alt):
    """Create a mission waypoint"""
    return MissionItem(
        latitude_deg=lat,
        longitude_deg=lon,
        relative_altitude_m=rel_alt,
        speed_m_s=CRUISE_SPEED,
        is_fly_through=True,
        gimbal_pitch_deg=0.0,
        gimbal_yaw_deg=0.0,
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=0.0,
        camera_photo_interval_s=0.0,
        acceptance_radius_m=60.0,
        yaw_deg=float("nan"),
        camera_photo_distance_m=0.0,
        vehicle_action=MissionItem.VehicleAction.NONE,
    )

async def create_mission_with_terrain_alts(terrain_map, mission_points, home_alt):
    """Create mission with terrain-based altitudes"""
    print("Creating mission with terrain altitudes...")
    
    mission_waypoints = []
    
    for i, (lat, lon) in enumerate(mission_points):
        ground_height = terrain_map.get_ground_height(lat, lon)
        required_amsl = ground_height + TARGET_AGL
        rel_alt = max(required_amsl - home_alt, 15.0)
        
        mission_waypoints.append(
            create_waypoint(lat, lon, rel_alt)
        )
        
        if i < 5:  # Show first few waypoints
            print(f"  WP{i}: Ground {ground_height:.1f}m -> Alt {rel_alt:.1f}m")

    return MissionPlan(mission_waypoints)

async def update_mission_altitudes(drone, terrain_map, mission_points, home_alt, current_wp):
    """Update mission waypoint altitudes only"""
    try:
        # Create new mission with updated altitudes
        mission_waypoints = []
        
        for i, (lat, lon) in enumerate(mission_points):
            ground_height = terrain_map.get_ground_height(lat, lon)
            required_amsl = ground_height + TARGET_AGL
            rel_alt = max(required_amsl - home_alt, 15.0)
            
            mission_waypoints.append(
                create_waypoint(lat, lon, rel_alt)
            )

        new_mission = MissionPlan(mission_waypoints)
        
        # Pause mission briefly
        await drone.mission.pause_mission()
        await asyncio.sleep(2)
        
        # Clear and upload new mission
        await drone.mission.clear_mission()
        await asyncio.sleep(1)
        await drone.mission.upload_mission(new_mission)
        await asyncio.sleep(2)
        
        # Resume from current position
        await drone.mission.set_current_mission_item(current_wp)
        await asyncio.sleep(1)
        
        # Continue mission
        await drone.mission.start_mission()
        
        print(f"Mission altitudes updated! Continuing from WP{current_wp}")
        return True
        
    except Exception as e:
        print(f"Mission update failed: {e}")
        # Try to resume mission
        try:
            await drone.mission.start_mission()
        except:
            pass
        return False

async def get_current_mission_progress(drone):
    """Get current mission progress"""
    try:
        async for progress in drone.mission.mission_progress():
            return progress.current
    except:
        return 0

async def check_current_altitude(drone, terrain_map, home_alt):
    """Check current altitude relative to terrain"""
    try:
        position = await drone.telemetry.position().__anext__()
        current_lat = position.latitude_deg
        current_lon = position.longitude_deg
        current_alt = position.absolute_altitude_m
        
        # Get terrain below current position
        current_ground = terrain_map.get_ground_height(current_lat, current_lon)
        current_agl = current_alt - current_ground
        
        # Calculate target altitude for current position
        target_amsl = current_ground + TARGET_AGL
        alt_error = target_amsl - current_alt
        
        return {
            'current_agl': current_agl,
            'target_amsl': target_amsl,
            'alt_error': alt_error,
            'needs_update': abs(alt_error) > 5.0  # Update if >5m error
        }
    except:
        return None

async def simple_terrain_follower(drone, terrain_map, mission_points, home_alt):
    """Simple terrain following - only update waypoint altitudes"""
    print("Simple terrain follower active")
    print("Following mission, updating waypoint altitudes only")

    current_waypoint = 0
    mission_complete = False
    last_update_time = 0
    last_check_time = 0

    while not mission_complete and current_waypoint < len(mission_points):
        try:
            current_time = time.time()
            
            # Get current mission progress
            current_waypoint = await get_current_mission_progress(drone)
            
            # Check altitude periodically
            if current_time - last_check_time > CHECK_INTERVAL:
                last_check_time = current_time
                
                alt_info = await check_current_altitude(drone, terrain_map, home_alt)
                
                if alt_info:
                    status = "OK" if abs(alt_info['alt_error']) <= 5 else "ADJUST"
                    print(f"AGL: {alt_info['current_agl']:5.1f}m | "
                          f"Error: {alt_info['alt_error']:+5.1f}m | "
                          f"Status: {status} | "
                          f"WP: {current_waypoint + 1}/{len(mission_points)}")
                    
                    # Update mission if altitude error is >5m and enough time passed
                    if (alt_info['needs_update'] and 
                        current_time - last_update_time > MIN_UPDATE_GAP and
                        current_waypoint < len(mission_points) - 1):
                        
                        print(f"Updating mission altitudes (error: {alt_info['alt_error']:.1f}m)")
                        success = await update_mission_altitudes(
                            drone, terrain_map, mission_points, home_alt, current_waypoint
                        )
                        if success:
                            last_update_time = current_time
                            print("Waypoint altitudes updated")

            # Check mission completion
            if current_waypoint >= len(mission_points) - 1:
                position = await drone.telemetry.position().__anext__()
                current_lat = position.latitude_deg
                current_lon = position.longitude_deg
                
                target_lat, target_lon = mission_points[-1]
                distance_to_end = (
                    math.sqrt((current_lat - target_lat) ** 2 + (current_lon - target_lon) ** 2)
                    * 111320
                )
                
                if distance_to_end < 50:
                    mission_complete = True
                    print("Mission complete!")

            await asyncio.sleep(1)

        except Exception as e:
            print(f"System error: {e}")
            await asyncio.sleep(2)

async def main():
    print("UAV Simple Terrain Following System")
    print("Follow mission waypoints, update altitudes on 5m error")

    # Load terrain data
    terrain_map = TerrainMap(DEM_FILE)

    # Load mission
    mission_points = load_mission_points(MISSION_PLAN)

    # Connect to aircraft
    drone = System()
    await drone.connect(system_address="udp://:14550")

    print("Connecting to aircraft...")
    async for connection_status in drone.core.connection_state():
        if connection_status.is_connected:
            print("Aircraft connected!")
            break

    # Get home position
    print("Getting position data...")
    home_altitude = None
    async for home_position in drone.telemetry.home():
        if home_position.absolute_altitude_m is not None:
            home_altitude = float(home_position.absolute_altitude_m)
            print(f"Home altitude: {home_altitude:.2f}m")
            break
    await asyncio.sleep(1)

    if home_altitude is None:
        print("No home position available")
        return

    # Create mission with terrain altitudes
    print("Preparing mission with terrain altitudes...")
    initial_mission = await create_mission_with_terrain_alts(terrain_map, mission_points, home_altitude)
    
    # Upload and start mission
    await drone.mission.upload_mission(initial_mission)
    await asyncio.sleep(1)
    
    # Takeoff sequence
    print("Starting takeoff...")
    await drone.action.arm()
    await asyncio.sleep(1)
    await drone.action.takeoff()
    await asyncio.sleep(10)
    
    # Begin mission
    await drone.mission.start_mission()
    print("Mission started!")

    # Start simple terrain following
    await simple_terrain_follower(drone, terrain_map, mission_points, home_altitude)

    print("Mission completed!")

if __name__ == "__main__":
    asyncio.run(main())
