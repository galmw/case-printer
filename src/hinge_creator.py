import pymesh
import os
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation


class HingeCreator(object):
    SOCK = pymesh.load_mesh(os.path.join('src', 'socket.stl'))
    HINGE = pymesh.load_mesh(os.path.join('src', 'hinge.stl'))

    def __init__(self, bottom_half, top_half, bottom_interior, top_interior) -> None:
        """
        Contains all the logic to transform the halves into halves with a hinge and socket that will connect them.
        """
        self.bottom_half = bottom_half
        self.top_half = top_half
        self.bottom_interior = bottom_interior
        self.top_interior = top_interior

    def rotate_meshes_hinge(self):
        """
        Find the correct rotation to apply the hinge to.
        """
        slice = pymesh.slice_mesh(self.bottom_half, np.array([0.0, 0.0, 1.0]), 1)[0]

        points_2d = np.ndarray(shape=(len(slice.vertices), 2))
        for i, vertex in enumerate(slice.vertices):
            points_2d[i] = vertex[0:2]
    
        pi2 = np.pi/2.

        # get the convex hull for the points
        hull_points = points_2d[ConvexHull(points_2d).vertices]

        # calculate edge angles
        edges = np.zeros((len(hull_points)-1, 2))
        edges = hull_points[1:] - hull_points[:-1]

        angles = np.zeros((len(edges)))
        angles = np.arctan2(edges[:, 1], edges[:, 0])

        angles = np.abs(np.mod(angles, pi2))
        angles = np.unique(angles)

        # find rotation matrices
        rotations = np.vstack([
            np.cos(angles),
            np.cos(angles-pi2),
            np.cos(angles+pi2),
            np.cos(angles)]).T

        rotations = rotations.reshape((-1, 2, 2))

        # apply rotations to the hull
        rot_points = np.dot(rotations, hull_points.T)

        # find the bounding points
        min_x = np.nanmin(rot_points[:, 0], axis=1)
        max_x = np.nanmax(rot_points[:, 0], axis=1)
        min_y = np.nanmin(rot_points[:, 1], axis=1)
        max_y = np.nanmax(rot_points[:, 1], axis=1)

        # find the box with the best area
        areas = (max_x - min_x) * (max_y - min_y)
        best_idx = np.argmin(areas)

        # return the best box
        x1 = max_x[best_idx]
        x2 = min_x[best_idx]
        y1 = max_y[best_idx]
        y2 = min_y[best_idx]
        r = rotations[best_idx]        

        # Rotate to align to larger edge of the bounding box
        if abs(y1 - y2) > abs(x1 - x2):
            print("Rotating the boudning box to align hinge")
            rotate_90_degrees = np.array([[0, -1], [1, 0]])
            r = np.matmul(rotate_90_degrees, r)

        spatial_rotation_matrix = np.eye(3)
        spatial_rotation_matrix[0][0] = r[0][0]
        spatial_rotation_matrix[0][1] = r[0][1]
        spatial_rotation_matrix[1][0] = r[1][0]
        spatial_rotation_matrix[1][1] = r[1][1]

        self.bottom_half = self.rotate_mesh_by_matrix(self.bottom_half, spatial_rotation_matrix)
        self.top_half = self.rotate_mesh_by_matrix(self.top_half, spatial_rotation_matrix)
        self.bottom_interior = self.rotate_mesh_by_matrix(self.bottom_interior, spatial_rotation_matrix)
        self.top_interior = self.rotate_mesh_by_matrix(self.top_interior, spatial_rotation_matrix)
    
    @staticmethod
    def rotate_mesh_by_matrix(mesh, spatial_rotation_matrix):
        r = Rotation.from_matrix(spatial_rotation_matrix)
        rotated_vertices = r.apply(mesh.vertices)
        return pymesh.form_mesh(rotated_vertices, mesh.faces)

    def connect_sock(self):
        mesh = self.bottom_half
        
        # For the x axis - place in the middle
        # For the y axis - place outside of the inner hull
        # For the z axis - place inside the edge
        """
        "Outer" option:
        mesh_placement_point = np.array([(mesh.bbox[0][0] + mesh.bbox[1][0]) / 2,  mesh.bbox[1][1], mesh.bbox[1][2]])
        sock_connection_point = np.array([(sock.bbox[0][0] + sock.bbox[1][0]) / 2, sock.bbox[0][1], sock.bbox[0][2]])
        """
        mesh_placement_point = np.array([(mesh.bbox[0][0] + mesh.bbox[1][0]) / 2,  self.bottom_interior.bbox[1][1], mesh.bbox[1][2]])
        sock_connection_point = np.array([(self.SOCK.bbox[0][0] + self.SOCK.bbox[1][0]) / 2, self.SOCK.bbox[0][1], self.SOCK.bbox[1][2]])
        
        sock = pymesh.form_mesh(self.SOCK.vertices + (mesh_placement_point - sock_connection_point), self.SOCK.faces)

        # Remove the convex hull of the sock, to create room for the hinge
        width = (sock.bbox[1][1] - sock.bbox[0][1]) / 2

        sock_hull = pymesh.convex_hull(sock)
        mesh = pymesh.boolean(mesh, sock_hull, operation='difference')

        
        sock_box = pymesh.generate_box_mesh(sock.bbox[0], sock.bbox[1])
        sock_box_inside = pymesh.form_mesh(sock_box.vertices - np.array([0, width, 0]), sock_box.faces)
        mesh = pymesh.boolean(mesh, sock_box_inside, operation='difference')
        #mesh = pymesh.boolean(mesh, sock_hull, operation='difference')
        self.bottom_half = pymesh.boolean(mesh, sock, operation='union')

    def connect_hinge(self):
        mesh = self.top_half

        mesh_placement_point = np.array(
            [(mesh.bbox[0][0] + mesh.bbox[1][0]) / 2, self.bottom_interior.bbox[1][1], mesh.bbox[0][2]])
        hinge_connection_point = np.array(
            [(self.HINGE.bbox[0][0] + self.HINGE.bbox[1][0]) / 2, self.HINGE.bbox[0][1], self.HINGE.bbox[1][2]])

        hinge = pymesh.form_mesh(self.HINGE.vertices + (mesh_placement_point - hinge_connection_point), self.HINGE.faces)

        self.top_half = pymesh.boolean(mesh, hinge, operation='union')


    def create_hinge(self):
        radius = 1
        bottom_center = np.ndarray([0,0,0])
        top_center = np.ndarray([0,0,5])
        cylinder = pymesh.generate_cylinder(bottom_center, top_center, radius, radius, num_segments=50)
        bottom_sphere = pymesh.generate_icosphere(radius * 0.8, bottom_center)
        top_sphere = pymesh.generate_icosphere(radius * 0.8, top_center)
        hinge = pymesh.boolean(cylinder, pymesh.boolean(top_sphere, bottom_sphere, operation='union'))
        return hinge