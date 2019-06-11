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
        self.vec_perp = np.array([0, 0, 0])
        self.proj_p31_on_p21 = np.array([0, 0, 0])

        # Base raster params
        self.raster_shape_rows = 5
        self.raster_shape_columns = 5

    def gen_raster_shape(self):
        """
        Generates base raster shape. 
        Raster box is (0,0), (1,0), (0, -1), (1, -1) and 
        obeys standard xy axis and orientation
        """
        
        self.base_raster_rows = []

        for i in range(0,self.raster_shape_rows + 1):
            self.base_raster_rows.append([])
            for j in range(0, self.raster_shape_columns + 1):
                self.base_raster_rows[i].append([float(j)/self.raster_shape_columns, -1*float(i)/self.raster_shape_rows])
        
        self.base_raster_pts = []
        for i in range(0, len(self.base_raster_rows)):
            if i%2 == 0:
                curr_row = self.base_raster_rows[i]
            else:
                curr_row = self.base_raster_rows[i].reverse()
            for j in range(0, len(self.base_raster_rows[i])):
                self.base_raster_pts.append(self.base_raster_rows[i][j])

    def transform_raster_pts(self):
        """
        Transforms base raster shape by scaling, translatin and rotating it 
        """

        # Get scaling factor
        x_scale = np.linalg.norm(np.array(self.p2.translation) - np.array(self.p1.translation))
        y_scale = np.linalg.norm(np.array(self.proj_p31_on_p21) - np.array(self.p3.translation))
      
        translation_offset = np.array(self.p1.translation)
        scaling_trans_matrix = np.array([
                                [   x_scale,    0,          0,  translation_offset[0]],
                                [   0,          y_scale,    0,  translation_offset[1]],
                                [   0,          0,          1,  0                    ],
                                [   0,          0,          0,  1                    ]]
                                )

        # Build matrix of pts
        pts_matrix = np.zeros([4, len(self.base_raster_pts)])
        for i in range(0, len(self.base_raster_pts)):
            pts_matrix[:2,i] = self.base_raster_pts[i]
            pts_matrix[2:,i] = [0,1] # z =0, since base shape is on xy plane
        
        # Get rotation matrix
        rotation_matrix = self.get_axis_rotation(np.array([0,0,1]), self.vec_perp)
        for i in range(0, np.size(rotation_matrix,0)):
            for j in range(0, np.size(rotation_matrix,1)):
                if np.absolute(rotation_matrix[i][j]) < 10**-5:
                    rotation_matrix[i][j] = 0
        # Apply Trasform
        transformation_matrix = np.matmul(scaling_trans_matrix,rotation_matrix)
        new_rotation_matrix = np.array([
                                    [1, 0, 0, 0],
                                    [0,0,1,0],
                                    [0,-1,0,0],
                                    [0,0,0,1]

                                    ])
        print("R : {}".format(new_rotation_matrix))
        transformed_pts = np.matmul(transformation_matrix, pts_matrix)

        # Get raster points
        self.raster_pts = []
        for i in range(0, np.size(transformed_pts,1)):
            self.raster_pts.append(transformed_pts[:3])

    def set_perp(self, p1, p2, p3):
        """
        Finds vector perpendicular to line p1p2 towards p3
        and projection of p3 into line p1p2
         
        """

        # Find vectors
        vec31 = np.array(self.p3.translation, dtype = np.float64) - np.array(self.p1.translation, dtype = np.float64)
        vec21 = np.array(self.p2.translation, dtype = np.float64)  - np.array(self.p1.translation, dtype = np.float64)
        
        self.proj_p31_on_p21 = np.dot(vec31, vec21)/(np.linalg.norm(vec21)**2) * vec21
        self.vec_perp = np.array(self.p3.translation) -  ( self.p1.translation + np.array(self.proj_p31_on_p21) ) 

        self.vec_perp = np.cross(vec31, vec21)
    def plot_raster(self):
        
        fig = plt.figure()

        # Plot base raster
        ax = fig.add_subplot(211, projection='3d')
        for idx, pts in enumerate(self.base_raster_pts):
            ax.scatter(pts[0],pts[1], c ='r',s = 5)
            ax.text3D(pts[0],pts[1],0, str(idx), zdir = None)
            
        ax.scatter(self.p1.translation[0],self.p1.translation[1],self.p1.translation[2], c ='b',s = 25)
        ax.scatter(self.p2.translation[0],self.p2.translation[1],self.p2.translation[2], c ='b',s = 25)
        ax.scatter(self.p3.translation[0],self.p3.translation[1],self.p3.translation[2], c ='b',s = 25)
        ax.text3D(self.p1.translation[0],self.p1.translation[1],self.p1.translation[2], "P1", zdir = None)
        ax.text3D(self.p2.translation[0],self.p2.translation[1],self.p2.translation[2], "P2", zdir = None)
        ax.text3D(self.p3.translation[0],self.p3.translation[1],self.p3.translation[2], "P3", zdir = None)

        plt.title("Base Raster")   
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax = fig.add_subplot(212, projection='3d')
        for idx, pts in enumerate(self.raster_pts):
            ax.scatter(pts[0],pts[1],pts[2], c = (0,1, 0), s = 5)
        ax.scatter(self.p1.translation[0],self.p1.translation[1],self.p1.translation[2], c ='b',s = 25)
        ax.scatter(self.p2.translation[0],self.p2.translation[1],self.p2.translation[2], c ='b',s = 25)
        ax.scatter(self.p3.translation[0],self.p3.translation[1],self.p3.translation[2], c ='b',s = 25)
        ax.text3D(self.p1.translation[0],self.p1.translation[1],self.p1.translation[2], "P1", zdir = None)
        ax.text3D(self.p2.translation[0],self.p2.translation[1],self.p2.translation[2], "P2", zdir = None)
        ax.text3D(self.p3.translation[0],self.p3.translation[1],self.p3.translation[2], "P3", zdir = None)


        plt.title("Transformed Raster")
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        plt.show()

    def get_axis_rotation(self,x,y):
        """
        Returns a homogenous transformation matrix that rotates x to y 
        """
        EPS = 10**-4
        dotProd = np.dot(x,y)/(np.linalg.norm(x)*np.linalg.norm(y))
        print(dotProd)
        if dotProd < -1 + EPS and dotProd > -1 - EPS:
            print("Anti Parallel Vectors")
            axis = np.array([y[1], -y[0],0])
            angle = np.pi

        elif dotProd < 1 + EPS and dotProd > 1 - EPS:
            print("Parallel Vectors")
            return np.identity(4)
        else:
            axis = np.array(np.cross(x,y))
            axis = axis/np.linalg.norm(axis)
            angle = np.arccos(np.dot(x,y)/(np.linalg.norm(x)*np.linalg.norm(y)))
        print("Rotate about axis: {}".format(axis))
        print("Perp vec: {}".format(self.vec_perp))
        C = np.cos(angle/2.0)
        S = np.sin(angle/2.0)
        quat =  np.quaternion(C, axis[0]*S, axis[1]*S, axis[2]*S)

        dualq = DualQuaternion.from_quat_pose_array([C, axis[0]*S, axis[1]*S, axis[2]*S,0,0,0])
        return dualq.homogeneous_matrix

if __name__ == "__main__":
    myRaster = Raster()
    myRaster.p1 = DualQuaternion.from_translation_vector([-1, 0 , 0])
    myRaster.p2 = DualQuaternion.from_translation_vector([1, 0 , 0])
    myRaster.p3 = DualQuaternion.from_translation_vector([0, -1 , 0])

    myRaster.set_perp(myRaster.p1, myRaster.p2, myRaster.p3)
    myRaster.gen_raster_shape()
    myRaster.transform_raster_pts()
    myRaster.plot_raster()