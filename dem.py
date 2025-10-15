import rasterio
from rasterio.plot import show
from pyproj import Transformer
import matplotlib.pyplot as plt

# Load DEM file
dem_path = "output_be.tif"  # replace with your DEM file
dem = rasterio.open(dem_path)

# Plot DEM
show(dem, cmap="terrain")

# Function to get elevation from DEM at lat/lon
def get_elevation(lat, lon):
    # Convert lat/lon -> DEM coordinate system
    transformer = Transformer.from_crs("EPSG:4326", dem.crs, always_xy=True)
    x, y = transformer.transform(lon, lat)
    row, col = dem.index(x, y)
    elevation = dem.read(1)[row, col]
    return elevation

# Example: get elevation at Kathmandu
lat, lon = 27.7172, 85.3240
print("Elevation at Kathmandu:", get_elevation(lat, lon), "m")
