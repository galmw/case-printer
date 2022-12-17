from scipy.spatial import ConvexHull
import numpy as np
from numpy import asarray
from stl import mesh


class CasePrinter(object):
    def __init__(self) -> None:
        self.stl_mesh = None

    def load_stl_from_file(self, filename):
        # Load the STL file
        self.stl_mesh = mesh.Mesh.from_file(filename)

    def create_case(self, output_path):
        points = np.array(self.stl_mesh.vectors)
        # Compute the convex hull
        print("Comupting convex hull")
        hull = ConvexHull(points)
        # Extract the vertices of the convex hull
        vertices = points[hull.vertices]

        # Define the vertices of the mesh
        vertices = asarray(vertices)

        # Create the mesh
        print("Creating new mesh")
        convex_hull_mesh = mesh.Mesh(np.zeros(vertices.shape[0], dtype=mesh.Mesh.dtype))
        for i, point in enumerate(vertices):
            convex_hull_mesh.vectors[i] = point

        # Export the mesh to an STL file
        convex_hull_mesh.save(output_path)
