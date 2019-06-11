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

    def __init__(self, p1, p2, p3):
        
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.proj_p31_on_p21 = [1,1,1]

        # Base raster params
        self.raster_shape_rows = 5
        self.raster_shape_columns = 5

        # Find helper vectors and projected point
        vec31 = np.array(self.p3, dtype = np.float64) - np.array(self.p1, dtype = np.float64)
        vec21 = np.array(self.p2, dtype = np.float64)  - np.array(self.p1, dtype = np.float64)
        self.proj_p31_on_p21 = self.p1 + np.dot(vec31, vec21)/(np.linalg.norm(vec21)**2) * vec21

        # Generates base raster shape. Raster box is (0,0), (1,0), (0, -1), (1, -1) and 
        # obeys standard xy axis and orientation
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
                curr_row = self.base_raster_rows[i][::-1]
            for j in range(0, len(curr_row)):
                self.base_raster_pts.append(curr_row[j])
        
    def transform_raster_pts(self):
        """
        Transforms base raster shape by scaling, translatin and rotating it 
        """
        
        # Get rotation matrix
        base_x =  np.array([1,0,0])
        base_y =  np.array([0,1,0])
        base_z =  np.array([0,0,1])

        self.desired_x =  np.array(self.p2) - np.array(self.p1)
        self.desired_y =  np.array(self.proj_p31_on_p21) - np.array(self.p3) 
        self.desired_z = np.cross(self.desired_x, self.desired_y)

        rotation_matrix = self.get_rotation_matrix (base_x, base_y, base_z, self.desired_x, self.desired_y, self.desired_z)
        for i in range(0, np.size(rotation_matrix,0)):
            for j in range(0, np.size(rotation_matrix,1)):
                if np.absolute(rotation_matrix[i][j]) < 10**-5:
                    rotation_matrix[i][j] = 0
        print("Det R {}".format(np.linalg.det(rotation_matrix)))
        print("R*R-1: {}".format(np.matmul(rotation_matrix, np.linalg.inv(rotation_matrix))))

        # Get scaling factor
        x_scale = np.linalg.norm(np.array(self.p2) - np.array(self.p1))
        y_scale = np.linalg.norm(np.array(self.proj_p31_on_p21) - np.array(self.p3))
        translation_offset = np.array([self.p1[0], self.p1[1], self.p1[2], 0])
        scaling_trans_matrix = np.array([
                                [   x_scale,    0,          0,  0],
                                [   0,          y_scale,    0,  0],
                                [   0,          0,          1,  0],
                                [   0,          0,          0,  1]]
                                )
        
        traslation_matrix = np.array([
                                    [1, 0, 0, translation_offset[0]],
                                    [0, 1, 0, translation_offset[1]],
                                    [0, 0, 1, translation_offset[2]],
                                    [0, 0, 0 ,1]])

             
                            
        # Build matrix of pts
        pts_matrix = np.zeros([4, len(self.base_raster_pts)])
        for i in range(0, len(self.base_raster_pts)):
            pts_matrix[:2,i] = self.base_raster_pts[i]
            pts_matrix[2:,i] = [0,1] # z =0, since base shape is on xy plane

        # Apply Transform
        transformation_matrix = np.matmul(rotation_matrix, scaling_trans_matrix)
        transformation_matrix = np.matmul(traslation_matrix, transformation_matrix)
        transformed_pts = np.matmul(transformation_matrix, pts_matrix)
        print("scaling_trans_matrix \n {}".format(scaling_trans_matrix))
        print("rotation_matrix \n {}".format(rotation_matrix))
        print("1st row , last column {}".format(transformed_pts[:,5]))

        # Get raster points
        self.raster_pts = []
        for i in range(0, np.size(transformed_pts,1)):
            self.raster_pts.append(transformed_pts[:3])
    
    def get_rotation_matrix (self, base_x, base_y, base_z, desired_x, desired_y, desired_z):

        # Every Column : Project desired frame axeses onto base frame
        
        # Normalize vectors
        base_x = base_x/np.linalg.norm(base_x)
        base_y = base_y/np.linalg.norm(base_y)
        base_z = base_z/np.linalg.norm(base_z)
        desired_x = desired_x/np.linalg.norm(desired_x)
        desired_y = desired_y/np.linalg.norm(desired_y)
        desired_z = desired_z/np.linalg.norm(desired_z)
        
        rotation_matrix = np.array  ([
                                    [self.project_vec(desired_x,base_x), self.project_vec(desired_y,base_x), self.project_vec(desired_z,base_x), 0],
                                    [self.project_vec(desired_x,base_y), self.project_vec(desired_y,base_y), self.project_vec(desired_z,base_y), 0],
                                    [self.project_vec(desired_x,base_z), self.project_vec(desired_y,base_z), self.project_vec(desired_z,base_z), 0],
                                    [0, 0, 0, 1]
                                    ])

        return rotation_matrix

    def project_vec(self, x, y):
        """
        Project vector x onto vector y
        Returns component of x in y
        """

        return np.dot(x,y)/(np.linalg.norm(y)**2)

    def plot_raster(self):
        
        fig = plt.figure( figsize=(10, 10))

        # Plot base raster
        ax = fig.add_subplot(211, projection='3d')
        for idx, pts in enumerate(self.base_raster_pts):
            ax.scatter(pts[0],pts[1], c ='r',s = 5)
            ax.text3D(pts[0],pts[1],0, str(idx), zdir = None)
            
        ax.scatter(self.p1[0],self.p1[1],self.p1[2], c ='b',s = 25)
        ax.scatter(self.p2[0],self.p2[1],self.p2[2], c ='b',s = 25)
        ax.scatter(self.p3[0],self.p3[1],self.p3[2], c ='b',s = 25)
        ax.text3D(self.p1[0],self.p1[1],self.p1[2], "P1", zdir = None)
        ax.text3D(self.p2[0],self.p2[1],self.p2[2], "P2", zdir = None)
        ax.text3D(self.p3[0],self.p3[1],self.p3[2], "P3", zdir = None)

        ax.scatter(self.proj_p31_on_p21[0],self.proj_p31_on_p21[1],self.proj_p31_on_p21[2], c = (0, 0, 1), s = 25)
        ax.text3D(self.proj_p31_on_p21[0],self.proj_p31_on_p21[1],self.proj_p31_on_p21[2], "Proj", zdir = None)
        
        plt.title("Base Raster")   
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        #ax.set_aspect('equal')
        #ax.view_init(azim=270, elev=90)

         # Plot Trasnformed raster
        ax = fig.add_subplot(212, projection='3d')
        for idx, pts in enumerate(self.raster_pts):
            ax.scatter(pts[0],pts[1],pts[2], c = (0,1, 0), s = 5)
        ax.scatter(self.p1[0],self.p1[1],self.p1[2], c ='b',s = 25)
        ax.scatter(self.p2[0],self.p2[1],self.p2[2], c ='b',s = 25)
        ax.scatter(self.p3[0],self.p3[1],self.p3[2], c ='b',s = 25)
        ax.text3D(self.p1[0],self.p1[1],self.p1[2], "P1", zdir = None)
        ax.text3D(self.p2[0],self.p2[1],self.p2[2], "P2", zdir = None)
        ax.text3D(self.p3[0],self.p3[1],self.p3[2], "P3", zdir = None)
        
        ax.scatter(self.proj_p31_on_p21[0],self.proj_p31_on_p21[1],self.proj_p31_on_p21[2], c = (0, 0, 1), s = 25)
        ax.text3D(self.proj_p31_on_p21[0],self.proj_p31_on_p21[1],self.proj_p31_on_p21[2], "Proj", zdir = None)
        
        ax.scatter(self.desired_x[0],self.desired_x[1],self.desired_x[2], c = (0, 0, 1), s = 25)
        ax.text3D(self.desired_x[0],self.desired_x[1],self.desired_x[2], "d_x", zdir = None)
        ax.scatter(self.desired_y[0],self.desired_y[1],self.desired_y[2], c = (0, 0, 1), s = 25)
        ax.text3D(self.desired_y[0],self.desired_y[1],self.desired_y[2], "d_y", zdir = None)

        plt.title("Transformed Raster")
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        #ax.view_init(azim=270, elev=90)
        #ax.set_aspect('equal')
        plt.show()

if __name__ == "__main__":

    theta = 45
    myRaster = Raster([0, 0 , 0], [np.cos(theta), np.sin(theta) , 0], [np.sin(theta), -np.cos(theta) , 0])
    #myRaster = Raster([0, 0 , 0],  [0, 0 , -1], [0, -1 , 0])

    myRaster.transform_raster_pts()
    myRaster.plot_raster()