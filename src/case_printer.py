import pymesh
import pyvista


class CasePrinter(object):
    def __init__(self) -> None:
        self._mesh = None

    def load_stl_from_file(self, filename):
        # Load the STL file
        self._mesh = pymesh.load_mesh(filename)

    def create_case(self, output_path):
        # Compute the convex hull
        print("Comupting convex hull")
        hull = pymesh.convex_hull(self._mesh)
        # Create the mesh

        output_case = hull
        return output_case

    @staticmethod
    def save_mesh_to_stl(mesh, output_path):
        print("Saving mesh")
        pymesh.save_mesh(output_path, mesh)

    @staticmethod
    def display_stl(path):
        # Load the STL file
        mesh = pyvista.read(path)
        # Show the plot
        mesh.plot()
        pyvista.read
