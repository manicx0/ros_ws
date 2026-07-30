import math
import yaml

EARTH_RADIUS = 6371000.0


class WaypointLoader:
    def __init__(self, config_path):
        self.waypoints = {}
        self._display_name_map = {}
        self.gps_origin = None
        self._load(config_path)

    def _load(self, path):
        if not path:
            return
        try:
            with open(path, 'r') as f:
                config = yaml.safe_load(f)

            origin = config.get('gps_origin')
            if origin:
                self.gps_origin = {
                    'lat': math.radians(origin.get('lat', 0.0)),
                    'lon': math.radians(origin.get('lon', 0.0))
                }

            for name, wp in config.get('waypoints', {}).items():
                self.waypoints[name] = wp
                display_name = wp.get('name', '')
                if display_name:
                    self._display_name_map[display_name.lower()] = name

        except Exception as e:
            print(f'Failed to load waypoints: {e}')

    def get_waypoint(self, name):
        wp = self.waypoints.get(name)
        if not wp:
            wp = self.waypoints.get(self._display_name_map.get(name.lower(), ''))
        if not wp:
            return None

        if 'odom' in wp:
            return wp['odom']

        if 'gps' in wp and self.gps_origin:
            gps = wp['gps']
            return self._gps_to_odom(gps['lat'], gps['lon'])

        if 'gps' in wp:
            print(f'Waypoint "{name}" has GPS but no gps_origin configured')
            return None

        return None

    def _gps_to_odom(self, lat_deg, lon_deg):
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)

        x = (lon - self.gps_origin['lon']) * math.cos(self.gps_origin['lat']) * EARTH_RADIUS
        y = (lat - self.gps_origin['lat']) * EARTH_RADIUS

        return {'x': x, 'y': y}

    def get_available_names(self):
        return list(self.waypoints.keys())

    def get_formatted_waypoints(self):
        """Return a formatted string with all waypoints and their coordinates."""
        if not self.waypoints:
            return "No waypoints defined"
        
        lines = []
        for name, wp in self.waypoints.items():
            display_name = wp.get('name', name)
            
            if 'odom' in wp:
                x = wp['odom'].get('x', 0.0)
                y = wp['odom'].get('y', 0.0)
                lines.append(f"  {name}: x={x:.2f}, y={y:.2f} ({display_name})")
            elif 'gps' in wp:
                if self.gps_origin:
                    gps = wp['gps']
                    odom = self._gps_to_odom(gps['lat'], gps['lon'])
                    x = odom.get('x', 0.0)
                    y = odom.get('y', 0.0)
                    lines.append(f"  {name}: x={x:.2f}, y={y:.2f} ({display_name})")
                else:
                    lat = wp['gps'].get('lat', 0.0)
                    lon = wp['gps'].get('lon', 0.0)
                    lines.append(f"  {name}: lat={lat:.6f}, lon={lon:.6f} ({display_name})")
        
        return '\n'.join(lines) if lines else "No waypoints defined"
