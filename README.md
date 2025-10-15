Package
pip install mavsdk numpy rasterio pyproj


This project implements a real-time terrain-following mission system for PX4-based UAVs using MAVSDK and Digital Elevation Model (DEM) data. The UAV dynamically adjusts its mission waypoint altitudes based on the terrain below, ensuring a constant Altitude Above Ground Level (AGL) throughout the flight ideal for autonomous missions over uneven terrain.Dynamic Altitude Adjustment: Continuously monitors UAV altitude relative to terrain and updates mission waypoints if deviation exceeds ±5 m.

Dynamic Altitude Adjustment: Continuously monitors UAV altitude relative to terrain and updates mission waypoints if deviation exceeds ±5 m.
DEM-Based Ground Elevation: Uses rasterio to read ground elevation data from a GeoTIFF (DEM) file.
