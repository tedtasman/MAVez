# coordinate.py
# version: 2.0.0
# Authors: Theodore Tasman
# Creation Date: 2025-01-30
# Last Modified: 2026-04-16
# Organization: PSU UAS

"""
Represents a geographic coordinate with latitude, longitude, and altitude.
"""

import math
from geographiclib.geodesic import Geodesic
class Coordinate:
    def __init__(
            self, 
            latitude_deg: float, 
            longitude_deg: float, 
            altitude_m: float = 0, 
            heading_deg: float = 0, 
            timestamp_ms: int | float = 0
    ):
        """Coordinate object represents a geographic coordinate as a specific WGS84 global position, with both integer and float representations

        Args:
            latitude_deg (float): Latitude of position in degrees.
            longitude_deg (float): Longitude of position in degrees.
            altitude_m (float, optional): Altitude of position in meters. Defaults to 0.
            heading_deg (float, optional): Heading of position in degrees clockwise from North. Defaults to 0.
            timestamp_ms (int, optional): Timestamp for position in milliseconds. Defaults to 0.
        """
        # float representations
        self.latitude_deg = latitude_deg
        self.longitude_deg = longitude_deg
        self.altitude_m = altitude_m
        self.heading_deg = heading_deg

        self.timestamp_ms = timestamp_ms

    @classmethod
    def from_int(
        cls, 
        latitude_degE7: int, 
        longitude_degE7: int,
        altitude_mm: int = 0,
        heading_cdeg: int = 0,
        timestamp_ms: int | float = 0
    ):
        """Create a coordinate object from using integer representation of position

        Args:
            latitude_degE7 (int): Latitude of position in degrees * 10^7.
            longitude_degE7 (int): Longitude of position in degrees * 10^7.
            altitude_mm (int, optional): Altitude of position in millimeters. Defaults to 0.
            heading_cdeg (int, optional): Heading of position in centidegrees clockwise from north. Defaults to 0.
            timestamp_ms (int | float, optional): Timestamp for the position in milliseconds. Defaults to 0.

        Returns:
            Coordinate: The created Coordinate object
        """
        return Coordinate(
            latitude_deg=latitude_degE7 / 1e7,
            longitude_deg=longitude_degE7 / 1e7,
            altitude_m=altitude_mm / 1000,
            heading_deg=heading_cdeg / 100,
            timestamp_ms=timestamp_ms
        )
    
    @classmethod
    def from_rad(
        cls,
        latitude_rad: float,
        longitude_rad: float,
        altitude_m: float = 0,
        heading_rad: float = 0,
        timestamp_ms: int | float = 0
    ):
        """Create a coordinate object from using radian representation of position

        Args:
            latitude_rad (float): Latitude of position in radians.
            longitude_rad (float): Longitude of position in radians.
            altitude_m (float, optional): Altitude of position in meters. Defaults to 0.
            heading_rad (float, optional): Heading of position in radians clockwise from north. Defaults to 0.
            timestamp_ms (int | float, optional): Timestamp for the position in milliseconds. Defaults to 0.

        Returns:
            Coordinate: The created Coordinate object
        """
        return Coordinate(
            latitude_deg=math.degrees(latitude_rad),
            longitude_deg=math.degrees(longitude_rad),
            altitude_m=altitude_m,
            heading_deg=math.degrees(heading_rad),
            timestamp_ms=timestamp_ms
        )

    def __str__(self):
        out = f"{self.latitude_deg}°, {self.longitude_deg}°; {self.altitude_m}m"

        return out

    __repr__ = __str__

    @property
    def latitude_degE7(self) -> int:
        return int(self.latitude_deg * 1e7)
    
    @property
    def longitude_degE7(self) -> int:
        return int(self.longitude_deg * 1e7)
    
    @property
    def altitude_mm(self) -> int:
        return int(self.altitude_m * 1000)
    
    @property
    def heading_cdeg(self) -> int:
        return int(self.heading_deg * 100)
    
    @property
    def latitude_rad(self) -> float:
        return math.radians(self.latitude_deg)

    @property
    def longitude_rad(self) -> float:
        return math.radians(self.longitude_deg)
    
    @property
    def heading_rad(self) -> float:
        return math.radians(self.heading_deg)

    def offset_coordinate(self, distance_m: float, azimuth_deg: float) -> "Coordinate":
        """
        Offset the coordinate by a given distance and heading.

        Args:
            distance_m (float): The distance to offset in meters.
            azimuth_deg (float): The bearing from the first coordinate in degrees.

        Returns:
            Coordinate: A new Coordinate object with the offset applied.
        """

        # use geodesic model to offset coordinates
        geo = Geodesic.WGS84.Direct(
            lat1=self.latitude_deg,
            lon1=self.longitude_deg,
            azi1=azimuth_deg,
            s12=distance_m,
            outmask=Geodesic.LATITUDE | Geodesic.LONGITUDE # only compute lat/lon
        )

        return Coordinate(
            latitude_deg=geo["lat2"],
            longitude_deg=geo["lon2"],
            altitude_m=self.altitude_m,
            heading_deg=self.heading_deg,
            timestamp_ms=self.timestamp_ms
        )

    def __eq__(self, other: "Coordinate | object") -> bool:
        if not isinstance(other, Coordinate):
            return False

        return (
            self.latitude_deg == other.latitude_deg
            and self.longitude_deg == other.longitude_deg
            and self.altitude_m == other.altitude_m
            and self.heading_deg == other.heading_deg
        )

    def distance_to(self, other: "Coordinate", include_alt: bool = False) -> float:
        """Calculate the distance between two coordinates in meters.

        Args:
            other (Coordinate): The other coordinate to calculate the distance to.
            include_alt (bool, optional): Flag to consider difference in altitude in distance. Defaults to false.

        Returns:
            float: The distance in meters between the two coordinates. 
        """

        geo = Geodesic.WGS84.Inverse(
            lat1=self.latitude_deg,
            lat2=other.latitude_deg,
            lon1=self.longitude_deg,
            lon2=other.longitude_deg,
            outmask=Geodesic.DISTANCE # only compute distance
        )
        if include_alt:
            alt_diff = self.altitude_m - other.altitude_m
            return math.sqrt(geo["s12"] ** 2 + alt_diff ** 2)
        else:
            return geo["s12"]


    def azimuth_to(self, other: "Coordinate") -> float:
        """Compute the azimuth to another coordinate clockwise from north

        Args:
            other (Coordinate): The other coordinate to calculate the azimuth to.

        Returns:
            float: The azimuth relative to north in degrees
        """
        geo = Geodesic.WGS84.Inverse(
            lat1=self.latitude_deg,
            lat2=other.latitude_deg,
            lon1=self.longitude_deg,
            lon2=other.longitude_deg,
            outmask=Geodesic.AZIMUTH # only compute azimuths
        )

        return geo["azi1"]
    

    def bearing_to(self, other: "Coordinate") -> float:
        """Compute the bearing to another coordinate clockwise from `self.heading_deg`

        Args:
            other (Coordinate): The other coordinate to calculate the bearing to.

        Returns:
            float: The bearing relative to heading in degrees
        """
        return self.azimuth_to(other) - self.heading_deg
    

    def path_to(self, other: "Coordinate") -> dict[str, float]:
        """
        Calculate the azimuth, bearing, and distance between two coordinates in degrees.

        Args:
            other (Coordinate): The other coordinate to calculate the path to.

        Returns:
            tuple[float, float]: A dict with keys for calculated "distance" in meters, "azimuth" in degrees, and "bearing" in degrees
        """
        geo = Geodesic.WGS84.Inverse(
            lat1=self.latitude_deg,
            lat2=other.latitude_deg,
            lon1=self.longitude_deg,
            lon2=other.longitude_deg,
            outmask=Geodesic.AZIMUTH | Geodesic.DISTANCE # compute azimuths and distance
        )

        return {
            "distance": geo["s12"],
            "azimuth": geo["azi1"],
            "bearing": geo["azi1"] - self.heading_deg
        }



def main():
    coord1 = Coordinate(-35.3632623, 149.1652377, 784.09, 20)
    coord2 = Coordinate.from_int(-353_624_199, 1_491_655_403, 50000)

    print(f"Coordinate 1: {coord1}")
    print(f"Coordinate 2: {coord2}")

    distance = coord1.distance_to(coord2)
    bearing = coord1.azimuth_to(coord2)

    print(f"Distance between coordinates: {distance:.2f} meters")
    print(f"Azimuth from coord1 to coord2: {bearing:.2f} degrees")

    offset_coord = coord1.offset_coordinate(1000, 45)
    print(f"Offset Coordinate (1000m at 45°): {offset_coord}")

    distance = coord1.distance_to(offset_coord)
    azimuth = coord1.azimuth_to(offset_coord)
    bearing = coord1.bearing_to(offset_coord)
    path = coord1.path_to(offset_coord)


    print(f"Distance to Offset Coordinate: {distance:.2f} meters")
    print(f"Azimuth to Offset Coordinate: {azimuth:.2f} degrees")
    print(f"Bearing to Offset coordinate: {bearing:.2f} degrees")
    print(path)

if __name__ == "__main__":
    main()