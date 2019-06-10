"""
Raster Generator

Author: Mrunal Sarvaiya
License: Peanut Robotics

"""

from dual_quaternions_ros import DualQuaternion
import numpy as np
import matplotlib.pyplot as plt

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
        Standard x and y axis and orientation
        Raster box is (0,0), (1,0), (0, -1), (1, -1)

        """
        self.base_raster_pts = []

        for i in range(0,self.raster_shape_columns + 1):
            for j in range(0, self.raster_shape_rows + 1):
                self.base_raster_pts.append([float(j)/self.raster_shape_columns, -1*float(i)/self.raster_shape_rows])

    def transform_raster_pts(self):

        # Get scaling factor
        x_scale = self.p2.translation[0] - self.p1.translation[0]
        y_scale = self.proj_p31_on_p21[1] - self.p3.translation[1]
        translation_offset = np.array(self.p1.translation)
        #import pdb; pdb.set_trace()
        trans_matrix = np.array([
                                [   x_scale,    0,          0,  translation_offset[0]],
                                [   0,          y_scale,    0,  translation_offset[1]],
                                [   0,          0,          1,  0                    ],
                                [   0,          0,          0,  1                    ]]
                                )

        # Build matrix of pts
        pts_matrix = np.zeros([4, len(self.base_raster_pts)])
        for i in range(0, len(self.base_raster_pts)):
            pts_matrix[:2,i] = self.base_raster_pts[i]
            pts_matrix[2:,i] = [0,1]
        
        # Transform points
        transformed_pts = np.matmul(trans_matrix, pts_matrix)

        # Get raster points
        self.raster_pts = []
        for i in range(0, np.size(transformed_pts,1)):
            self.raster_pts.append(transformed_pts[:3])

    def find_normal(self):
        """
        Call this after setting p1, p2 , p3
         
        """

        # Find vectors
        vec31 = np.array(self.p3.translation) - np.array(self.p1.translation)
        vec21 = np.array(self.p2.translation)  - np.array(self.p1.translation)
        
        self.proj_p31_on_p21 = np.dot(vec31, vec21)/(np.linalg.norm(vec21)**2) * vec21
        self.vec_normal = (np.array(self.p3.translation) - np.array(self.proj_p31_on_p21))/ np.linalg.norm(np.array(self.p3.translation) - np.array(self.proj_p31_on_p21))

    def gen_raster(self):

        #self.raster_pts = []

        for curr_row in range(0,self.num_row_raster):
            
            # Find new end points
            curr_p1 = np.array(self.p1.translation) + curr_row*self.vec_normal 
            curr_p2 = np.array(self.p2.translation) + curr_row*self.vec_normal 
            
            # Interpolate between new points
            old_p1 = self.p1.quat_pose_array
            translated_vec = np.array(old_p1[4:]) + curr_row*self.vec_normal
            new_p1 = DualQuaternion.from_quat_pose_array( np.concatenate((old_p1[0:4], translated_vec), axis = 0) )
            
            self.raster_pts.append(curr_p1)
            self.raster_pts.append(curr_p2)
            
    def plot_raster(self):

        # plt.scatter(*self.p1.translation[:2], c = (1,0, 0))
        # plt.scatter(*self.p2.translation[:2], c = (1,0, 0))
        # plt.scatter(*self.p3.translation[:2], c = (1,0, 0))

        plt.figure(1)
        for pts in self.base_raster_pts:
            plt.scatter(pts[0],pts[1], c = (0,1, 0))
        plt.title("Base Raster")
        plt.show()

        plt.figure(2)
        for pts in self.raster_pts:
            plt.scatter(pts[0],pts[1], c = (0,1, 0))
        plt.title("Transformed Raster")
        plt.show()

if __name__ == "__main__":
    myRaster = Raster()
    myRaster.p1 = DualQuaternion.from_translation_vector([0, 0 , 0])
    myRaster.p2 = DualQuaternion.from_translation_vector([1, 0 , 0])
    myRaster.p3 = DualQuaternion.from_translation_vector([0.5, -1 , 0])
    myRaster.num_row_raster = 5

    myRaster.find_normal()
    myRaster.gen_raster_shape()
    myRaster.transform_raster_pts()
    myRaster.plot_raster()