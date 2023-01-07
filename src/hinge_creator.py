import pymesh
import os
import numpy as np
import math
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation


class HingeCreator(object):
    SOCK = pymesh.load_mesh(os.path.join('src', 'socket.stl'))
    HINGE = pymesh.load_mesh(os.path.join('src', 'hinge.stl'))

    MAX_SCALE = 0.6

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
        min_x, min_y = np.nanmin(rot_points[:, 0], axis=1), np.nanmin(rot_points[:, 1], axis=1)
        max_x, max_y = np.nanmax(rot_points[:, 0], axis=1), np.nanmax(rot_points[:, 1], axis=1)

        # find the box with the best area
        areas = (max_x - min_x) * (max_y - min_y)
        best_idx = np.argmin(areas)

        # return the best box
        x1, y1 = max_x[best_idx], max_y[best_idx]
        x2, y2 = min_x[best_idx], min_y[best_idx]
        r = rotations[best_idx]        

        # Rotate to align to larger edge of the bounding box
        if abs(y1 - y2) > abs(x1 - x2):
            print("Rotating the boudning box to align hinge")
            rotate_90_degrees = np.array([[0, -1], [1, 0]])
            r = np.matmul(rotate_90_degrees, r)

        rotate_matrix = np.eye(3)
        rotate_matrix[0][0], rotate_matrix[0][1] = r[0][0], r[0][1]
        rotate_matrix[1][0], rotate_matrix[1][1] = r[1][0], r[1][1]

        self.bottom_half = self.rotate_mesh_by_matrix(self.bottom_half, rotate_matrix)
        self.top_half = self.rotate_mesh_by_matrix(self.top_half, rotate_matrix)
        self.bottom_interior = self.rotate_mesh_by_matrix(self.bottom_interior, rotate_matrix)
        self.top_interior = self.rotate_mesh_by_matrix(self.top_interior, rotate_matrix)
    
    @staticmethod
    def rotate_mesh_by_matrix(mesh, spatial_rotation_matrix):
        r = Rotation.from_matrix(spatial_rotation_matrix)
        rotated_vertices = r.apply(mesh.vertices)
        return pymesh.form_mesh(rotated_vertices, mesh.faces)

    def scale_hinge(self):
        # Scale by the X axis of the bounding box of the case

        max_case_width = (self.bottom_half.bbox[1][0] - self.bottom_half.bbox[0][0]) * self.MAX_SCALE
        print(f'Case width: {max_case_width}')
        sock_width = self.SOCK.bbox[1][0] - self.SOCK.bbox[0][0]
        if sock_width > max_case_width:
            print("Scaling hinge to match case")
            scale = max_case_width / sock_width
            self.SOCK = pymesh.form_mesh(self.SOCK.vertices * scale, self.SOCK.faces)
            self.HINGE = pymesh.form_mesh(self.HINGE.vertices * scale, self.HINGE.faces)
            sock_width *= scale

        print(f'Hinge width: {sock_width}')

    @property
    def hinge_connection_point(self):
        return self._part_connection_point(self.HINGE)

    @property
    def sock_connection_point(self):
        return self._part_connection_point(self.SOCK)
    
    def _part_connection_point(self, part):
        return np.array([(part.bbox[0][0] + part.bbox[1][0]) / 2,
                          part.bbox[0][1], part.bbox[1][2]])
    
    def part_height(self, part):
        return part.bbox[1][2] - part.bbox[0][2]

    def get_slice_at_z_values(self, mesh, bottom_z, top_z):
        MAX = 1000
        box_min = np.array([-MAX, -MAX, bottom_z])
        box_max = np.array([MAX, MAX, top_z])
        slice = pymesh.generate_box_mesh(box_min, box_max)
        return pymesh.boolean(mesh, slice, 'intersection')

    @property
    def placement_point(self):
        """
        For the x axis - place in the middle of the case
        For the y axis - place outside of the inner hull of the part which is parallel to the hinge and sock.
        For the z axis - place right below the top
        """
        x_val = (self.bottom_half.bbox[0][0] + self.bottom_half.bbox[1][0]) / 2
        z_val = self.bottom_half.bbox[1][2] # Also equals self.top_half.bbox[0][2]

        # For the Y value, create a slice:
        slice = self.get_slice_at_z_values(self.bottom_interior,
                                           z_val - self.part_height(self.SOCK), z_val)
        y_val = slice.bbox[1][1]

        return np.array([x_val, y_val, z_val])

    def get_hinge(self):
        hinge = pymesh.form_mesh(self.HINGE.vertices + (self.placement_point - self.hinge_connection_point), self.HINGE.faces)
        return hinge
    
    def get_sock(self):                
        sock = pymesh.form_mesh(self.SOCK.vertices + (self.placement_point - self.sock_connection_point), self.SOCK.faces)
        return sock

    def connect_sock(self):
        # First we must clear the path for the socket to exist on the edge of the case.
        hinge_bbox = self.get_hinge().bbox
        inner_y_value = (self.bottom_interior.bbox[0][1] + self.bottom_interior.bbox[1][1]) / 2
        extra_bbox_x = np.array([hinge_bbox[0][0], inner_y_value, hinge_bbox[0][2]])
        hinge_box = pymesh.generate_box_mesh(extra_bbox_x, hinge_bbox[1])

        self.bottom_half = pymesh.boolean(self.bottom_half, hinge_box, operation='difference')
        self.bottom_half = pymesh.boolean(self.bottom_half, self.get_sock(), operation='union')

    def connect_hinge(self):
        sock = self.get_sock()
        height = self.part_height(sock) / 3
        
        # Create some space in the upper case to allow movement around the sockets.
        sock1, sock2 = pymesh.separate_mesh(sock)
        sock1 = pymesh.convex_hull(sock1)
        sock2 = pymesh.convex_hull(sock2)
        upper_sock = pymesh.merge_meshes([sock1, sock2])
        upper_sock = pymesh.form_mesh(upper_sock.vertices + np.array([0, 0, height]), upper_sock.faces)
        self.top_half = pymesh.boolean(self.top_half, upper_sock, operation='difference')

        # Connect the hinge
        self.top_half = pymesh.boolean(self.top_half, self.get_hinge(), operation='union')
