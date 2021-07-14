#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from roslib import message
import pcl
from rospy.numpy_msg import numpy_msg
import cv2
import numpy as np
from projection_helper import birds_eye_point_cloud
import copy
import pyproj as pj
from numpy.polynomial import polynomial as P
from scipy import stats
from sklearn.cluster import KMeans, DBSCAN
import matplotlib
# For the error: Exception ignored in: <bound method Image.del of <tkinter.PhotoImage object at 0x7f1b5f86a710>> Traceback (most recent call last):
matplotlib.use('Agg')
#$DISPLAY error, solution:- echo "backend: Agg" > ~/.config/matplotlib/matplotlibrc
from matplotlib import pyplot as plt
from matplotlib import colors as clr
from mpl_toolkits.mplot3d import Axes3D
import tf2_ros
from PIL import Image

class CheckTopic:
    counter = 0
    def __init__(self):
        rospy.init_node("Starting the CheckTopic code")
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)
        #rospy.Subscriber('/ibeo/lidar/static', numpy_msg(PointCloud2), self.check_topic) 
        rospy.Subscriber('/lidar_pointcloud/top', numpy_msg(PointCloud2), self.check_topic) 
        #rospy.Subscriber('/pointcloud_transformer/output_pcl2', numpy_msg(PointCloud2), self.check_topic)
        rospy.spin()
        
    def check_topic(self, data): 
        print("**********************count: {}*********************".format(self.counter))
        pc = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z","intensity"))
        cloud_points = []
        for p in pc:
            cloud_points.append(p)
        xyzi_data = np.array(cloud_points)        
        xyzi_data = self.elevation_filter(xyzi_data)
        
        # Projecting point clouds into bird-eye view image
        xyzi_data = birds_eye_point_cloud(xyzi_data, side_range=(-10, 10), fwd_range=(0, 30), res=0.1, _type='intensity', saveto="/constraint_model/images/bird_eye_{}.png".format(self.counter))
        
        
        '''
        
        road_xyz = pcl.PointCloud_PointXYZI()
        road_xyz.from_array(xyzi_data.astype('float32'))
        
        print("Min X:{:15.2f}, Max X:{:15.2f}".format(np.min(xyzi_data[:,0]), np.max(xyzi_data[:,0])))
        print("Min Y:{:15.2f}, Max Y:{:15.2f}".format(np.min(xyzi_data[:,1]), np.max(xyzi_data[:,1])))
        print("Min Z:{:15.2f}, Max Z:{:15.2f}".format(np.min(xyzi_data[:,2]), np.max(xyzi_data[:,2])))
        print("Min I:{:15.2f}, Max I:{:15.2f}".format(np.min(xyzi_data[:,3]), np.max(xyzi_data[:,3])))

        # find road plane using pcl call
        road_plane, road_plane_idx = self.find_road_plane(xyzi_data)
        road_plane_flatten = road_plane[:,0:2]

        # cluster road plane, and find the road segment
        db = DBSCAN(eps=0.5, min_samples=5).fit_predict(road_plane_flatten)

        largest_cluster_label = stats.mode(db).mode[0]
        largest_cluster_points_idx = np.array(np.where(db == largest_cluster_label)).ravel()

        road_plane_seg_idx = road_plane_idx[largest_cluster_points_idx]
        road_plane_seg = copy.deepcopy(xyzi_data[road_plane_seg_idx, :])
        
        print("Road plane segment point count:{}".format(road_plane_seg.shape))

        road_plane_seg_cloud = pcl.PointCloud_PointXYZI()
        road_plane_seg_cloud.from_array(road_plane_seg.astype('float32'))
        
        #self.plot_points(road_plane_seg, skip=100, name="/constraint_model/images/road_seg_{}".format(self.counter))
        
        #line = self.thresholding(road_plane_seg)
        line = road_plane_seg
        # plot_points(line, skip=1)

        #print(line)

        self.line_clustering(line, xyzi_data, name="/constraint_model/images/lane_{}".format(self.counter))'''

        self.counter += 1
        
        
    def elevation_filter(self, pts):
        # filter with elevation
        
        print('Before filter, shape {}'.format(np.array(pts).shape))
        result = []
        mean = np.mean(pts[:, 2])
        std = np.std(pts[:, 2])
        threshold = mean
        for point in pts:
            if point[2] < threshold:
                result.append(point)
                
        print('After filter, shape {}'.format(np.array(result).shape))
        
        return np.array(result)
    
    def thresholding(self, points):
        #min = 10%
        #Max = 30%
        #threshold_min = abs((max(points[:,3])*15)/100)
        #threshold_max = abs((max(points[:,3])*100)/100)
        i_min = np.mean(points[:,3])+1*np.std(points[:,3])
        i_max = np.mean(points[:,3])+7*np.std(points[:,3])
        
        return points[np.logical_and(points[:,3]>i_min, points[:,3]<1)]

    def find_road_plane(self, xyz_data):

        # cloud = pcl.PointCloud()
        # cloud.from_array(xyz_data[:,0:3].astype('float32'))

        cloud = pcl.PointCloud_PointXYZI()
        cloud.from_array(xyz_data.astype('float32'))

        # fitler statistical outlier
        # fil_stat = cloud.make_statistical_outlier_filter()
        # fil_stat = cloud.make_statistical_outlier_filter()
        # fil_stat.set_mean_k(50)
        # fil_stat.set_std_dev_mul_thresh(1.0)
        # cloud_filtered = fil_stat.filter()

        # print "Statistical Inlier Number:", cloud_filtered.size

        # find normal plane
        seg = cloud.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
        seg.set_normal_distance_weight(0.001) # 0.001
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_max_iterations(100)
        seg.set_distance_threshold(0.3) #0.3
        indices, model = seg.segment()

        #print ("Road Plane Model:", model)

        cloud_plane = cloud.extract(indices, negative=False)

        # NG : const char* not str
        # cloud_plane.to_file('table_scene_mug_stereo_textured_plane.pcd')
        #pcl.save(cloud_plane, 'road_plane.pcd')
        #print "Road plane point cloud file road_plane.pcd saved."

        return cloud_plane.to_array(), np.array(indices)
    
    def plot_points(self, points, skip=100, point_size=1, name=None):
        
        points = points - np.min(points, axis=0)
        # plt.ion()
        norm = clr.Normalize(vmin=0,vmax=255)

        fig = plt.figure()
        ax = Axes3D(fig)

        max_range = np.array([points[:,0].max()-points[:,0].min(), points[:,1].max()-points[:,1].min(), points[:,2].max()-points[:,2].min()]).max() / 2.0
        mid_x = (points[:,0].max() + points[:,0].min()) * 0.5
        mid_y = (points[:,1].max() + points[:,1].min()) * 0.5
        mid_z = (points[:,2].max() + points[:,2].min()) * 0.5

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
    
        num = points.shape[0]
        sc = ax.scatter(points[0:num:skip,0], points[0:num:skip,1], \
                    points[0:num:skip,2], c=points[0:num:skip,3], norm=norm, s=point_size)

        plt.colorbar(sc)
        plt.savefig(name)

    def line_clustering(self, points, orig_points, name=None):
        # 2d clustering based on line structure in x-y plane first
        line = points
        fig2=plt.figure()
        ax2 = fig2.add_subplot(111)
        xy = points[:, 0:2]
        xy = xy - np.min(xy, axis=0)
        ax2.scatter(xy[:,0], xy[:,1])

        # initialization of m and c
        A = np.vstack([xy[:,0], np.ones(len(xy))]).T
        m, c = np.linalg.lstsq(A, xy[:,1])[0]
        c1 = c-20# + np.random.normal(0, 10)
        c2 = c-10
        c3 = c# + np.random.normal(0, 10)
        c4 = c+10
        c5 = c+20
        plt.plot(xy[:,0], m*xy[:,0] + c1, 'r')
        plt.plot(xy[:,0], m*xy[:,0] + c2, 'g')
        plt.plot(xy[:,0], m*xy[:,0] + c3, 'b')
        plt.plot(xy[:,0], m*xy[:,0] + c4, 'y')
        plt.plot(xy[:,0], m*xy[:,0] + c5, 'k')


        for i in range(10):
            # classify
            y1 = np.absolute(A.dot([[m],[c1]]) - xy[:,[1]])
            y2 = np.absolute(A.dot([[m],[c2]]) - xy[:,[1]])
            y3 = np.absolute(A.dot([[m],[c3]]) - xy[:,[1]])
            y4 = np.absolute(A.dot([[m],[c4]]) - xy[:,[1]])
            y5 = np.absolute(A.dot([[m],[c5]]) - xy[:,[1]])

            k1 = np.squeeze(np.all([y1<y2,y1<y3,y1<y4,y1<y5],axis=0))
            k2 = np.squeeze(np.all([y2<y1,y2<y3,y2<y4,y2<y5],axis=0))
            k3 = np.squeeze(np.all([y3<y1,y3<y2,y3<y4,y3<y5],axis=0))
            k4 = np.squeeze(np.all([y4<y1,y4<y2,y4<y3,y4<y5],axis=0))
            k5 = np.squeeze(np.all([y5<y1,y5<y2,y5<y3,y5<y4],axis=0))
            #k2 = np.squeeze(np.logical_and(np.logical_and(y2<y1, y2<y3),y2<10))
            # k3 = np.squeeze(np.logical_and(np.logical_and(y3<y1, y3<y2),y3<10))
            # k4 = np.squeeze(np.logical_and(np.logical_and(y4<y1, y3<y2),y3<10))
            # k5 = np.squeeze(np.logical_and(np.logical_and(y3<y1, y3<y2),y3<10))
            # update
            A1 = np.vstack([xy[k1,0], np.ones(len(xy[k1]))]).T
            A2 = np.vstack([xy[k2,0], np.ones(len(xy[k2]))]).T
            A3 = np.vstack([xy[k3,0], np.ones(len(xy[k3]))]).T
            A4 = np.vstack([xy[k4,0], np.ones(len(xy[k4]))]).T
            A5 = np.vstack([xy[k5,0], np.ones(len(xy[k5]))]).T

            m1, c1 = np.linalg.lstsq(A1, xy[k1,1])[0]
            m2, c2 = np.linalg.lstsq(A2, xy[k2,1])[0]
            m3, c3 = np.linalg.lstsq(A3, xy[k3,1])[0]
            m4, c4 = np.linalg.lstsq(A4, xy[k4,1])[0]
            m5, c5 = np.linalg.lstsq(A5, xy[k5,1])[0]

            m = (m1+m2+m3+m4+m5)/5.0
            # replot
            ax2.clear()
            plt.scatter(xy[:,0], xy[:,1])
            plt.plot(xy[:,0], m*xy[:,0] + c1, 'r')
            plt.plot(xy[:,0], m*xy[:,0] + c2, 'g')
            plt.plot(xy[:,0], m*xy[:,0] + c3, 'b')
            plt.plot(xy[:,0], m*xy[:,0] + c4, 'y')
            plt.plot(xy[:,0], m*xy[:,0] + c5, 'k')

        # 3d polyline fitting based on 2d clusters
        xyz = copy.deepcopy(line[:, 0:3])
        xyz = xyz - np.min(xyz, axis=0)

        max_range = np.array([xyz[:,0].max()-xyz[:,0].min(), xyz[:,1].max()-xyz[:,1].min(), xyz[:,2].max()-xyz[:,2].min()]).max() / 2.0
        mid_x = (xyz[:,0].max() + xyz[:,0].min()) * 0.5
        mid_y = (xyz[:,1].max() + xyz[:,1].min()) * 0.5
        mid_z = (xyz[:,2].max() + xyz[:,2].min()) * 0.5

        # divide into K clusters
        K1 = xyz[k1,:]
        K2 = xyz[k2,:]
        K3 = xyz[k3,:]
        K4 = xyz[k4,:]
        K5 = xyz[k5,:]
        K=[K1,K2,K3,K4,K5]

        # polyline fitting
        from numpy.polynomial import polynomial as P
        xs=np.arange(100)

        # parallel straight lines
        fig4=plt.figure()
        ax4 = Axes3D(fig4)
        ax4.set_xlim(mid_x - max_range, mid_x + max_range)
        ax4.set_ylim(mid_y - max_range, mid_y + max_range)
        ax4.set_zlim(mid_z - max_range, mid_z + max_range)
        ax4.scatter(xyz[:,0], xyz[:,1], xyz[:,2])
        pave = np.zeros((2,2))
        for i in range(len(K)):
            # Dhanoop modified
            if len(K[i]) > 0:
                X = K[i][:,0]
                Y = K[i][:,1:3]
                pave= pave + P.polyfit(X,Y,1)
        pave = pave / len(K)
        for i in range(len(K)):
            # Dhanoop modified
            if len(K[i]) > 0:
                X = K[i][:,0]
                Y = K[i][:,1:3]
                p1=P.polyfit(X,Y,1)
                ax4.plot(xs, p1[0,0] + pave[1,0]*xs, p1[0,1] + pave[1,1]*xs, 'y')

        # unparallel straight lines
        fig3=plt.figure()
        ax3 = Axes3D(fig3)
        ax3.set_xlim(mid_x - max_range, mid_x + max_range)
        ax3.set_ylim(mid_y - max_range, mid_y + max_range)
        ax3.set_zlim(mid_z - max_range, mid_z + max_range)
        ax3.scatter(xyz[:,0], xyz[:,1], xyz[:,2])
        for i in range(len(K)):
            
            # Dhanoop modified
            if len(K[i]) > 0:
                X = K[i][:,0]
                Y = K[i][:,1:3]
                p1=P.polyfit(X,Y,1)
                ax3.plot(xs, p1[0,0] + p1[1,0]*xs, p1[0,1] + p1[1,1]*xs, 'r')

        # parallel poly lines of degree 2
        fig6=plt.figure()
        ax6 = Axes3D(fig6)
        ax6.set_xlim(mid_x - max_range, mid_x + max_range)
        ax6.set_ylim(mid_y - max_range, mid_y + max_range)
        ax6.set_zlim(mid_z - max_range, mid_z + max_range)
        ax6.scatter(xyz[:,0], xyz[:,1], xyz[:,2])
        pave = np.zeros((3,2))
        for i in range(len(K)):
            
            # Dhanoop modified
            if len(K[i]) > 0:
                X = K[i][:,0]
                Y = K[i][:,1:3]
                pave= pave + P.polyfit(X,Y,2)
        pave = pave / len(K)
        for i in range(len(K)):
            
            # Dhanoop modified
            if len(K[i]) > 0:
                X = K[i][:,0]
                Y = K[i][:,1:3]
                p=P.polyfit(X,Y,2)
                ax6.plot(xs, p[0,0] + pave[1,0]*xs + pave[2,0]*xs**2, \
                    p[0,1] + pave[1,1]*xs + pave[2,1]*xs**2)

        # unparallel poly lines of degree 2
        fig5=plt.figure()
        ax5 = Axes3D(fig5)
        ax5.set_xlim(mid_x - max_range, mid_x + max_range)
        ax5.set_ylim(mid_y - max_range, mid_y + max_range)
        ax5.set_zlim(mid_z - max_range, mid_z + max_range)
        ax5.scatter(xyz[:,0], xyz[:,1], xyz[:,2])
        '''
        print ''
        print ''
        print ''
        print 'Best fit: polylines of degree 2:\n'
        '''

        for i in range(len(K)):
            
            # Dhanoop modified
            if len(K[i]) > 0:
                X = K[i][:,0]
                Y = K[i][:,1:3]
                p=P.polyfit(X,Y,2)
                ax5.plot(xs, p[0,0] + p[1,0]*xs + p[2,0]*xs**2, \
                    p[0,1] + p[1,1]*xs + p[2,1]*xs**2)

            '''
            print 'Y = ' + str(p[2,0]) + ' * X^2 + ' + str(p[1,0]) + ' * X + ' + str(p[0,0])
            print 'Z = ' + str(p[2,1]) + ' * X^2 + ' + str(p[1,1]) + ' * X + ' + str(p[1,0])
            print 'Y + Z = ' + str(p[2,0] + p[2,1]) + ' * X^2 + ' + str(p[1,0] + p[1,1]) + ' * X + ' + str(p[0,0] + p[1,0])
            print ''
            '''

        ### plot final result

        intensity = copy.deepcopy(orig_points[:, 3])
        orig_points[intensity > 30, 3] = 255
        points = copy.deepcopy(orig_points)

        skip = 10
        points = points - np.min(orig_points, axis=0)
        line_xyz = line - np.min(orig_points, axis=0)
        min_line_xyz = np.min(line_xyz, axis=0)

        # plt.ion()

        # print "Min X:{:15.2f}, Max X:{:15.2f}".format(np.min(points[:,0]), np.max(points[:,0]))
        # print "Min Y:{:15.2f}, Max Y:{:15.2f}".format(np.min(points[:,1]), np.max(points[:,1]))
        # print "Min Z:{:15.2f}, Max Z:{:15.2f}".format(np.min(points[:,2]), np.max(points[:,2]))

        # print "Min X:{:15.2f}, Max X:{:15.2f}".format(np.min(line_xyz[:,0]), np.max(line_xyz[:,0]))
        # print "Min Y:{:15.2f}, Max Y:{:15.2f}".format(np.min(line_xyz[:,1]), np.max(line_xyz[:,1]))
        # print "Min Z:{:15.2f}, Max Z:{:15.2f}".format(np.min(line_xyz[:,2]), np.max(line_xyz[:,2]))

        norm = clr.Normalize(vmin=0,vmax=255)

        fig = plt.figure()
        ax_f = Axes3D(fig)

        max_range = np.array([points[:,0].max()-points[:,0].min(), points[:,1].max()-points[:,1].min(), points[:,2].max()-points[:,2].min()]).max() / 2.0
        mid_x = (points[:,0].max() + points[:,0].min()) * 0.5
        mid_y = (points[:,1].max() + points[:,1].min()) * 0.5
        mid_z = (points[:,2].max() + points[:,2].min()) * 0.5

        ax_f.set_xlim(mid_x - max_range, mid_x + max_range)
        ax_f.set_ylim(mid_y - max_range, mid_y + max_range)
        ax_f.set_zlim(mid_z - max_range, mid_z + max_range)

        ax_f.set_xlabel('X')
        ax_f.set_ylabel('Y')
        ax_f.set_zlabel('Z')

        num = points.shape[0]
        sc = ax_f.scatter(points[0:num:skip,0], points[0:num:skip,1], \
                        points[0:num:skip,2], c=points[0:num:skip,3], norm=norm, s=4)

        for i in range(len(K)):
            
            # Dhanoop modified
            if len(K[i]) > 0:
                X = K[i][:,0]
                Y = K[i][:,1:3]
                p=P.polyfit(X,Y,2)

                ax_f.plot(xs + min_line_xyz[0], p[0,0] + p[1,0]*(xs) + p[2,0]*(xs)**2 + min_line_xyz[1], \
                    p[0,1] + p[1,1]*xs + p[2,1]*xs**2 + min_line_xyz[2])

        plt.colorbar(sc)
        plt.savefig(name)
        
    
if __name__=='__main__':
    try:
        CheckTopic()
    except rospy.ROSInterruptException:
	    rospy.logerr('Could not start Extract node.')
        
        
        
