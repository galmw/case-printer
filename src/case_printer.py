import pymesh


class CasePrinter(object):
    def __init__(self) -> None:
        self._mesh = None

    def load_stl_from_file(self, filename):
        # Load the STL file
        #self.stl_mesh = mesh.Mesh.from_file(filename)
        self._mesh = pymesh.load_mesh(filename)

    def create_case(self, output_path):
        # Compute the convex hull
        print("Comupting convex hull")
        #all_vertices = np.concatenate((self.stl_mesh.v0, self.stl_mesh.v1, self.stl_mesh.v2))
        hull = pymesh.convex_hull(self._mesh)
        # Create the mesh
        print("Saving new mesh")
        pymesh.save_mesh(output_path, hull)

