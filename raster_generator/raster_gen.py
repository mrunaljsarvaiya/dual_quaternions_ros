"""
Raster Generator

Author: Mrunal Sarvaiya
License: Peanut Robotics

"""

from dual_quaternions_ros import DualQuaternion
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Raster(object):

    def __init__(self):
        
        self.p1 = DualQuaternion.identity()
        self.p2 = DualQuaternion.identity()
        self.p3 = DualQuaternion.identity()
        self.num_row_raster = 0
        self.vec_normal = np.array([0, 0, 0])
        self.proj_p31_on_p21 = np.array([0, 0, 0])

        # For base raster
        self.raster_shape_rows = 5
        self.raster_shape_columns = 5

    def gen_raster_shape(self):
        """
        Standard xy axis and orientation
        Raster box is (0,0), (1,0), (0, -1), (1, -1)

        """
        self.base_raster_pts = []

        for i in range(0,self.raster_shape_rows + 1):
            for j in range(0, self.raster_shape_columns + 1):
                self.base_raster_pts.append([float(j)/self.raster_shape_columns, -1*float(i)/self.raster_shape_rows])

    def transform_raster_pts(self):

        # Get scaling factor
        x_scale = self.p2.translation[0] - self.p1.translation[0]
        y_scale = self.proj_p31_on_p21[1] - self.p3.translation[1]
        translation_offset = np.array(self.p1.translation)
        scaling_trans_matrix = np.array([
                                [   x_scale,    0,          0,  translation_offset[0]],
                                [   0,          y_scale,    0,  translation_offset[1]],
                                [   0,          0,          1,  0                    ],
                                [   0,          0,          0,  1                    ]]
                                )

        # Build matrix of ptsrint
        pts_matrix = np.zeros([4, len(self.base_raster_pts)])
        for i in range(0, len(self.base_raster_pts)):
            pts_matrix[:2,i] = self.base_raster_pts[i]
            pts_matrix[2:,i] = [0,1]
        
        # Get rotation matrix
        rotation_matrix = self.get_axis_rotation(np.array([0,-1,0]), self.vec_normal)

        # Apply Trasform
        transformation_matrix = np.matmul(rotation_matrix,scaling_trans_matrix)
        transformed_pts = np.matmul(transformation_matrix, pts_matrix)

        # Get raster points
        self.raster_pts = []
        for i in range(0, np.size(transformed_pts,1)):
            self.raster_pts.append(transformed_pts[:3])

    def find_normal(self):
        """
        Call this after setting p1, p2 , p3
         
        """

        # Find vectors
        vec31 = np.array(self.p3.translation, dtype = np.float64) - np.array(self.p1.translation, dtype = np.float64)
        vec21 = np.array(self.p2.translation, dtype = np.float64)  - np.array(self.p1.translation, dtype = np.float64)
        
        self.proj_p31_on_p21 = np.dot(vec31, vec21)/(np.linalg.norm(vec21)**2) * vec21
        self.vec_normal = np.array(self.p3.translation) -  ( self.p1.translation + np.array(self.proj_p31_on_p21) ) 
      
    def plot_raster(self):
        
        fig = plt.figure()

        # Plot base raster
        ax = fig.add_subplot(211, projection='3d')
        for pts in self.base_raster_pts:
            ax.scatter(pts[0],pts[1], c ='r',s = 5)
        ax.scatter(self.p1.translation[0],self.p1.translation[1],self.p1.translation[2], c ='b',s = 25)
        ax.scatter(self.p2.translation[0],self.p2.translation[1],self.p2.translation[2], c ='b',s = 25)
        ax.scatter(self.p3.translation[0],self.p3.translation[1],self.p3.translation[2], c ='b',s = 25)

        plt.title("Base Raster")   
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax = fig.add_subplot(212, projection='3d')
        for pts in self.raster_pts:
            ax.scatter(pts[0],pts[1],pts[2], c = (0,1, 0), s = 5)
        ax.scatter(self.p1.translation[0],self.p1.translation[1],self.p1.translation[2], c ='b',s = 25)
        ax.scatter(self.p2.translation[0],self.p2.translation[1],self.p2.translation[2], c ='b',s = 25)
        ax.scatter(self.p3.translation[0],self.p3.translation[1],self.p3.translation[2], c ='b',s = 25)
        
        plt.title("Transformed Raster")
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        plt.show()

    def get_axis_rotation(self,x,y):

        axis = np.array(np.cross(x,y))
        if np.absolute(np.linalg.norm(axis)) < 0.001:
            print("Norm close to zero")
            return np.identity(4)
        axis = axis/np.linalg.norm(axis)
        angle = np.arccos(np.dot(x,y)/(np.linalg.norm(x)*np.linalg.norm(y)))

        C = np.cos(angle/2)
        S = np.sin(angle/2)
        quat =  np.quaternion(C, axis[0]*S, axis[1]*S, axis[2]*S)

        dualq = DualQuaternion.from_quat_pose_array([C, axis[0]*S, axis[1]*S, axis[2]*S,0,0,0])
        return dualq.homogeneous_matrix

if __name__ == "__main__":
    myRaster = Raster()
    myRaster.p1 = DualQuaternion.from_translation_vector([0.5, 0 , 0])
    myRaster.p2 = DualQuaternion.from_translation_vector([1.5, -0.2 , 0])
    myRaster.p3 = DualQuaternion.from_translation_vector([0, -1 , 0])
    myRaster.num_row_raster = 5

    myRaster.find_normal()
    myRaster.gen_raster_shape()
    myRaster.transform_raster_pts()
    myRaster.plot_raster()