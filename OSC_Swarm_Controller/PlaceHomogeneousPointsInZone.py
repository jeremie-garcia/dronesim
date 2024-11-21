import numpy as np
from scipy.spatial import Voronoi
from shapely.geometry import Polygon, Point
import ast
import re

def generate_relaxed_points(data_string, num_points=10, iterations=1000):
    """
    Generates relaxed points within a polygon defined by data_string.

    Parameters:
    - data_string: str
        String representation of a list of coordinate pairs.
        Example: '[[x1, y1], [x2, y2], ..., [xn, yn]]'
    - num_points: int, optional
        Number of points to generate and relax within the polygon (default is 10).
    - iterations: int, optional
        Number of iterations for Lloyd's algorithm (default is 1000).

    Returns:
    - relaxed_coords: list of [x, y]
        List of relaxed points as coordinate pairs.
    """
   # Replace commas in numbers with dots
    def replace_commas_in_numbers(match):
        number_with_commas = match.group(0)
        number_with_dots = number_with_commas.replace(',', '.')
        return number_with_dots

    data_string_processed = re.sub(r'\d+(?:,\d+)*', replace_commas_in_numbers, data_string)
    
    # Parse the data_string to get polygon_coords
    try:
        polygon_coords = ast.literal_eval(data_string_processed) # Convert string to list
        if not isinstance(polygon_coords, list):
            raise ValueError("Parsed data is not a list.")
        # Ensure each coordinate is a list or tuple of length 2
        for coords in polygon_coords:
            print(coords)
            if not (isinstance(coords, list) or isinstance(coords, tuple)) or len(coords) != 2:
                raise ValueError("Each coordinate should be a list or tuple of length 2.")
    except (SyntaxError, ValueError) as e:
        raise ValueError(f"Invalid coordinate data: {e}")

    # Create a Shapely Polygon
    polygon = Polygon(polygon_coords)
    if not polygon.is_valid:
        polygon = polygon.buffer(0)
        if not polygon.is_valid:
            raise ValueError("The provided coordinates do not form a valid polygon.")

    # Generate initial random points inside the polygon
    minx, miny, maxx, maxy = polygon.bounds
    coords = []
    attempts = 0
    max_attempts = num_points * 10  # To prevent infinite loops
    while len(coords) < num_points and attempts < max_attempts:
        random_point = (
            np.random.uniform(minx, maxx),
            np.random.uniform(miny, maxy)
        )
        if polygon.contains(Point(random_point)):
            coords.append(random_point)
        attempts += 1
    if len(coords) < num_points:
        raise ValueError("Could not generate enough points inside the polygon.")
    coords = np.array(coords)

    #print chaque coords
    for coord in coords:
        print(coord)

    # Apply Lloyd's algorithm
    for _ in range(iterations):
        vor = Voronoi(coords)
        new_coords = []
        for i, region_index in enumerate(vor.point_region):
            region = vor.regions[region_index]
            if -1 in region or len(region) == 0:
                # Infinite region, keep the original point
                new_coords.append(coords[i])
                continue
            # Get the vertices of the Voronoi cell
            polygon_vertices = [vor.vertices[j] for j in region]
            cell = Polygon(polygon_vertices)
            # Intersection of cell with original polygon
            intersection = cell.intersection(polygon)
            if not intersection.is_empty and intersection.geom_type in ['Polygon', 'MultiPolygon']:
                # Handle MultiPolygon by taking the largest area polygon
                if intersection.geom_type == 'MultiPolygon':
                    intersection = max(intersection.geoms, key=lambda a: a.area)
                centroid = np.array(intersection.centroid.coords[0])
                new_coords.append(centroid)
            else:
                # If intersection is empty or not a polygon, keep the original point
                new_coords.append(coords[i])
        coords = np.array(new_coords)

    relaxed_coords = coords.tolist()
    return relaxed_coords
