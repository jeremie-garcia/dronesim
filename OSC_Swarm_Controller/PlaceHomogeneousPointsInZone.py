import re
import ast
import numpy as np
from shapely.geometry import Polygon, Point
from shapely.validation import explain_validity
from geovoronoi import voronoi_regions_from_coords

def generate_relaxed_points(data_string, num_points=10, max_iterations=30, tol=1e-1):
    """
    Generates relaxed points within a polygon defined by data_string using Lloyd's algorithm.

    Parameters:
    - data_string: str
        String representation of a list of coordinate pairs.
        Example: '[[x1, y1], [x2, y2], ..., [xn, yn]]'
    - num_points: int, optional
        Number of points to generate and relax within the polygon (default is 10).
    - max_iterations: int, optional
        Maximum number of iterations for Lloyd's algorithm (default is 100).
    - tol: float, optional
        Tolerance for convergence in Lloyd's algorithm (default is 1e-3).

    Returns:
    - relaxed_coords: list of [x, y]
        List of relaxed points as coordinate pairs.
    """

    # Replace commas in numbers with dots (if needed)
    def replace_commas_in_numbers(match):
        number_with_commas = match.group(0)
        number_with_dots = number_with_commas.replace(',', '.')
        return number_with_dots

    data_string_processed = re.sub(r'\d+(?:,\d+)*', replace_commas_in_numbers, data_string)
    
    # Parse the data_string to get polygon_coords
    try:
        polygon_coords = ast.literal_eval(data_string_processed)  # Convert string to list
        if not isinstance(polygon_coords, list):
            raise ValueError("Parsed data is not a list.")
        # Ensure each coordinate is a list or tuple of length 2
        for coords in polygon_coords:
            if not (isinstance(coords, list) or isinstance(coords, tuple)) or len(coords) != 2:
                raise ValueError("Each coordinate should be a list or tuple of length 2.")
    except (SyntaxError, ValueError) as e:
        raise ValueError(f"Invalid coordinate data: {e}")

    # Create a Shapely Polygon
    polygon = Polygon(polygon_coords)
    if not polygon.is_valid:
        print("Invalid polygon:", explain_validity(polygon))
        polygon = polygon.buffer(0)  # Attempt to fix the polygon
        if not polygon.is_valid:
            raise ValueError("The provided coordinates do not form a valid polygon.")
    
    if num_points == 1:
        # Get the centroid of the polygon
        centroid = polygon.centroid
        return [[centroid.x, centroid.y]]

    # Generate initial random points inside the polygon
    minx, miny, maxx, maxy = polygon.bounds
    coords = []
    attempts = 0
    max_attempts = num_points * 100  # Increase max_attempts for higher chance
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

    # Convert initial coords to Point objects
    seed_points = [Point(xy) for xy in coords]

    # Apply Lloyd's algorithm using geovoronoi
    for iteration in range(max_iterations):
        coords_array = np.array([[p.x, p.y] for p in seed_points])
        try:
            region_polys, _ = voronoi_regions_from_coords(coords_array, polygon)
        except Exception as e:
            raise ValueError(f"Error during Voronoi computation: {e}")
        max_displacement = 0
        new_seed_points = []
        for idx in range(len(seed_points)):
            if idx in region_polys:
                region_poly = region_polys[idx]
                centroid = region_poly.centroid
                displacement = seed_points[idx].distance(centroid)
                max_displacement = max(max_displacement, displacement)
                new_seed_points.append(centroid)
            else:
                # If no region for this point, keep the original point
                new_seed_points.append(seed_points[idx])
        seed_points = new_seed_points
        if max_displacement < tol:
            # Converged
            print(f"Lloyd's algorithm converged after {iteration+1} iterations.")
            break

    # Prepare the relaxed coordinates to return
    relaxed_coords = [[point.x, point.y] for point in seed_points]
    return relaxed_coords



# if __name__ == "__main__":
#     data_string = '[[0, 0], [0, 10], [10, 10], [10, 0]]'
#     num_points = 1
#     relaxed_points = generate_relaxed_points(data_string, num_points=num_points)
#     print(f"Relaxed points: {relaxed_points}")