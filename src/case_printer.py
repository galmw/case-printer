import pymesh
import pyvista
import numpy as np
from numpy.linalg import norm
import os
import math
from gravity_simulator import get_case_gravity_orientation
from scipy.spatial.transform import Rotation


class CasePrinter(object):
    _REFINEMENT_ORDER = 3
    
    def __init__(self, mesh_path, output_dir='') -> None:
        self._mesh = pymesh.load_mesh(mesh_path)
        self._output_dir = output_dir
        self._mesh = self.fix_mesh(self._mesh, detail='low')

    def create_case(self, thickness=0.1, gravity_rotate=True) -> pymesh.Mesh:
        print("Comupting convex hull")
        # This might be useful someday: pymesh.compute_outer_hull(mesh)
        hull = pymesh.convex_hull(self._mesh)

        # Find the correct rotation via a physics simulation
        if gravity_rotate:
            hull = self.gravity_rotate_mesh(hull)

        bigger_hull = self.get_outer_case(hull, thickness=thickness)

        diff = pymesh.boolean(bigger_hull, hull, operation='difference')
        top_half, bottom_half = self.split_mesh_in_two(diff)

        return top_half, bottom_half

    def gravity_rotate_mesh(self, mesh: pymesh.Mesh):
        temp_mesh_path = os.path.join(self._output_dir, 'temp_mesh.obj')
        self.save_mesh_to_file(mesh, temp_mesh_path)
        orientation = get_case_gravity_orientation(temp_mesh_path)
        os.remove(temp_mesh_path)

        r = Rotation.from_quat(orientation)
        rotated_vertices = r.apply(mesh.vertices)
        rotated_mesh = pymesh.form_mesh(rotated_vertices, mesh.faces)

        return rotated_mesh

    def get_outer_case(self, mesh, thickness):
        center = (mesh.bbox[0] + mesh.bbox[1]) / 2
        new_vertices = mesh.vertices * (1 + thickness) - (center * thickness)
        return pymesh.form_mesh(new_vertices, mesh.faces)

    def split_mesh_in_two(self, mesh: pymesh.Mesh):
        """
        1. Split the bouding box into two bounding boxes on top of each other.
        2. Intersect given mesh with a each half of the bouding box
        """
        bottom, top = mesh.bbox[0], mesh.bbox[1]
        mid_height = (bottom[2] + top[2]) / 2

        top_box = pymesh.generate_box_mesh(bottom, (top[0], top[1], mid_height))
        bottom_box = pymesh.generate_box_mesh((bottom[0], bottom[1], mid_height), top)
        
        top_half = pymesh.boolean(mesh, top_box, operation='intersection')
        bottom_half = pymesh.boolean(mesh, bottom_box, operation='intersection')

        return top_half, bottom_half
    
    @staticmethod
    def save_mesh_to_file(mesh, output_path):
        print("Saving mesh")
        pymesh.save_mesh(output_path, mesh)

    @staticmethod
    def _pymesh_to_pyvista(mesh: pymesh.Mesh):
        tmp_path = "temp.obj"
        pymesh.save_mesh(tmp_path, mesh)
        mesh = pyvista.read(tmp_path)
        os.remove(tmp_path)
        return mesh
    
    @staticmethod
    def display_stl(path):
        # Load the STL file
        mesh = pyvista.read(path)
        # Show the plot
        mesh.plot()

    def display_two_meshes(self, mesh1, mesh2, show_edges=False):
        plotter = pyvista.Plotter(shape=(1, 2))

        plotter.add_text("Top Half", font_size=30)
        plotter.add_mesh(self._pymesh_to_pyvista(mesh1), show_edges=show_edges)

        plotter.subplot(0, 1)
        plotter.add_text("Bottom half\n", font_size=30)
        plotter.add_mesh(self._pymesh_to_pyvista(mesh2), show_edges=show_edges)
        # Optional - plotter.link_views()
        plotter.show()

    @staticmethod
    def fix_mesh(mesh, detail="normal"):
        bbox_min, bbox_max = mesh.bbox
        diag_len = norm(bbox_max - bbox_min)
        if detail == "normal":
            target_len = diag_len * 5e-3
        elif detail == "high":
            target_len = diag_len * 2.5e-3
        elif detail == "low":
            target_len = diag_len * 1e-2
        print("Target resolution: {} mm".format(target_len))

        count = 0
        mesh, __ = pymesh.remove_degenerated_triangles(mesh, 100)
        mesh, __ = pymesh.split_long_edges(mesh, target_len)
        num_vertices = mesh.num_vertices
        while True:
            mesh, __ = pymesh.collapse_short_edges(mesh, 1e-6)
            mesh, __ = pymesh.collapse_short_edges(mesh, target_len,
                                                preserve_feature=True)
            mesh, __ = pymesh.remove_obtuse_triangles(mesh, 150.0, 100)
            if mesh.num_vertices == num_vertices:
                break

            num_vertices = mesh.num_vertices
            print("#v: {}".format(num_vertices))
            count += 1
            if count > 10: break

        mesh = pymesh.resolve_self_intersection(mesh)
        mesh, __ = pymesh.remove_duplicated_faces(mesh)
        mesh = pymesh.compute_outer_hull(mesh)
        mesh, __ = pymesh.remove_duplicated_faces(mesh)
        mesh, __ = pymesh.remove_obtuse_triangles(mesh, 179.0, 5)
        mesh, __ = pymesh.remove_isolated_vertices(mesh)

        return mesh

    def get_sphere_polyline(self, radius, points=6):
        """
        This code is meant for using a minkowski sum with a sphere polyline.
        sphere = pymesh.generate_icosphere(radius=1,
                                           center=(0, 0, 0),
                                           refinement_order=self._REFINEMENT_ORDER)
        sphere_polyline = self.get_sphere_polyline(radius=thickness)
        """
        n = points
        x, y, z = [], [], []
        # Calculate the points on the sphere
        for i in range(n):
            for j in range(n):
                # Calculate the coordinates
                x.append(radius * math.sin(math.pi * i / n) * math.cos(2 * math.pi * j / n))
                y.append(radius * math.sin(math.pi * i / n) * math.sin(2 * math.pi * j / n))
                z.append(radius * math.cos(math.pi * i / n))

        polyline = np.column_stack((x, y, z))
        return polyline