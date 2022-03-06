import os
from utils import *
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from baseline import Baseline
import math

import vis_gui


class SFM:
    """Represents the main reconstruction loop"""

    def __init__(self, views, matches, K):

        self.views = views  # list of views
        self.matches = matches  # dictionary of matches
        self.names = []  # image names
        self.done = []  # list of views that have been reconstructed
        self.K = K  # intrinsic matrix
        self.points_3D = np.zeros((views[0].led_count, 3))  # reconstructed 3D points
        self.points_3D_all = []
        for i in range(len(self.points_3D)):
            self.points_3D_all.append([])
        self.point_counter = 0  # keeps track of the reconstructed points
        self.point_map = {}  # a dictionary of the 2D points that contributed to a given 3D point
        self.errors = []  # list of mean reprojection errors taken at the end of every new view being added

        for view in self.views:
            self.names.append(view.name)

        if not os.path.exists(self.views[0].root_path + '/points'):
            os.makedirs(self.views[0].root_path + '/points')

        # store results in a root_path/points
        self.results_path = os.path.join(self.views[0].root_path, 'points')

    def get_index_of_view(self, view):
        """Extracts the position of a view in the list of views"""

        return self.names.index(view.name)

    def remove_mapped_points(self, match_object, image_idx):
        """Removes points that have already been reconstructed in the completed views"""

        inliers1 = []
        inliers2 = []

        for i in range(len(match_object.inliers1)):
            if (image_idx, match_object.inliers1[i]) not in self.point_map:
                inliers1.append(match_object.inliers1[i])
                inliers2.append(match_object.inliers2[i])

        match_object.inliers1 = inliers1
        match_object.inliers2 = inliers2

    def compute_pose(self, view1, view2=None, is_baseline=False):
        """Computes the pose of the new view"""

        # procedure for baseline pose estimation
        if is_baseline and view2:

            match_object = self.matches[(view1.name, view2.name)]
            baseline_pose = Baseline(view1, view2, match_object)
            view2.R, view2.t = baseline_pose.get_pose(self.K)

            rpe1, rpe2 = self.triangulate(view1, view2)
            self.errors.append(np.mean(rpe1))
            self.errors.append(np.mean(rpe2))

            self.done.append(view1)
            self.done.append(view2)

        # procedure for estimating the pose of all other views
        else:

            view1.R, view1.t = self.compute_pose_PNP(view1)
            errors = []

            # reconstruct unreconstructed points from all of the previous views
            for i, old_view in enumerate(self.done):
                match_object = self.matches[(old_view.name, view1.name)]
                try:
                    _ = remove_outliers_using_F(old_view, view1, match_object)
                    self.remove_mapped_points(match_object, i)
                    _, rpe = self.triangulate(old_view, view1)
                    errors += rpe
                except:
                  pass

            self.done.append(view1)
            self.errors.append(np.mean(errors))

    def triangulate(self, view1, view2):
        """Triangulates 3D points from two views whose poses have been recovered. Also updates the point_map dictionary"""

        K_inv = np.linalg.inv(self.K)
        P1 = np.hstack((view1.R, view1.t))
        P2 = np.hstack((view2.R, view2.t))

        match_object = self.matches[(view1.name, view2.name)]

        pixel_points1 = np.array([kp.pt for kp in view1.keypoints])[match_object.inliers1]
        if len(pixel_points1) == 0:
            return [], []
        descriptors1 = np.array([desc for desc in view1.descriptors])[match_object.inliers1]
        pixel_points2 = np.array([kp.pt for kp in view2.keypoints])[match_object.inliers2]

        pixel_points1 = cv2.convertPointsToHomogeneous(pixel_points1)[:, 0, :]
        pixel_points2 = cv2.convertPointsToHomogeneous(pixel_points2)[:, 0, :]
        reprojection_error1 = []
        reprojection_error2 = []

        for i in range(len(pixel_points1)):

            u1 = pixel_points1[i, :]
            u2 = pixel_points2[i, :]

            u1_normalized = K_inv.dot(u1)
            u2_normalized = K_inv.dot(u2)

            point_3D = get_3D_point(u1_normalized, P1, u2_normalized, P2)
            self.points_3D_all[descriptors1[i]].append(point_3D.T)
            self.points_3D[descriptors1[i]] = np.median(self.points_3D_all[descriptors1[i]], axis=0)

            error1 = calculate_reprojection_error(point_3D, u1[0:2], self.K, view1.R, view1.t)
            reprojection_error1.append(error1)
            error2 = calculate_reprojection_error(point_3D, u2[0:2], self.K, view2.R, view2.t)
            reprojection_error2.append(error2)

            self.point_map[(self.get_index_of_view(view1), match_object.inliers1[i])] = descriptors1[i]
            self.point_map[(self.get_index_of_view(view2), match_object.inliers2[i])] = descriptors1[i]
            self.point_counter += 1

        return reprojection_error1, reprojection_error2

    def compute_pose_PNP(self, view):
        """Computes pose of new view using perspective n-point"""

        matches = []

        # collects all the descriptors of the reconstructed views
        for old_view in self.done:
            imgIdx = self.get_index_of_view(old_view)
            for descriptor in old_view.descriptors:
                try:
                    match = []
                    match.append(imgIdx)
                    match.append(view.descriptors.index(descriptor))
                    match.append(old_view.descriptors.index(descriptor))
                    matches.append(match)
                except:
                    pass

        # match old descriptors against the descriptors in the new view
        points_3D, points_2D = np.zeros((0, 3)), np.zeros((0, 2))

        # build corresponding array of 2D points and 3D points
        for match in matches:
            old_image_idx, new_image_kp_idx, old_image_kp_idx = match[0], match[1], match[2]

            if (old_image_idx, old_image_kp_idx) in self.point_map:

                # obtain the 2D point from match
                point_2D = np.array(view.keypoints[new_image_kp_idx].pt).T.reshape((1, 2))
                points_2D = np.concatenate((points_2D, point_2D), axis=0)

                # obtain the 3D point from the point_map
                point_3D = self.points_3D[self.point_map[(old_image_idx, old_image_kp_idx)], :].T.reshape((1, 3))
                points_3D = np.concatenate((points_3D, point_3D), axis=0)

        # compute new pose using solvePnPRansac
        _, R, t, _ = cv2.solvePnPRansac(points_3D[:, np.newaxis], points_2D[:, np.newaxis], self.K, None,
                                        confidence=0.99, reprojectionError=8.0, flags=cv2.SOLVEPNP_DLS)
        R, _ = cv2.Rodrigues(R)
        return R, t

    def fix_points(self):
        points_3D = self.points_3D.copy()

        #fix zero points
        for idx_0, point in enumerate(self.points_3D[1:-1]):
            idx = idx_0 + 1
            if (point[0] == 0 and point[1] == 0 and point[2] == 0):
                prev_idx = idx - 1
                while prev_idx > 0:
                    if (points_3D[prev_idx][0] != 0 or points_3D[prev_idx][1] != 0 or points_3D[prev_idx][2] != 0):
                        break
                    prev_idx -= 1
                if prev_idx == -1:
                    #no previous point found
                    continue
                next_idx = idx + 1
                while next_idx < len(points_3D):
                    if (points_3D[next_idx][0] != 0 or points_3D[next_idx][1] != 0 or points_3D[next_idx][2] != 0):
                        break
                    next_idx += 1
                if next_idx == len(points_3D):
                    #no next point found
                    continue

                differs = [j-i for i,j in zip(points_3D[prev_idx], points_3D[next_idx])]
                step = 1
                while prev_idx + step != next_idx:
                    points_3D[prev_idx+step] = [i+j for i,j in zip(points_3D[prev_idx],[k*step/(next_idx-prev_idx) for k in differs])]
                    step += 1

        midpoint = np.mean(points_3D, axis=0)
        points_3D = points_3D - midpoint

        return points_3D

    def plot_points(self, points_3D = None, name = None, display = False):
        """Saves the reconstructed 3D points to ply files using Open3D"""

        if points_3D is None:
            points_3D = self.points_3D.copy()
        

        if name == None:
            name = str(len(self.done))

        lines = []
        colors = []
        for i in range(len(self.points_3D) - 1):
            lines.append([i, i+1])
            colors.append([i / len(self.points_3D), 1-(i / len(self.points_3D)), 0])

        filename = os.path.join(self.results_path, name + '_images.ply')
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_3D)
        pcd.colors = o3d.utility.Vector3dVector(np.zeros((self.views[0].led_count, 3)))
        o3d.io.write_point_cloud(filename, pcd)

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points_3D)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors)
        filename = os.path.join(self.results_path, name + '_images_lines.ply')
        o3d.io.write_line_set(filename, line_set, True)

        if display:
            gui.Application.instance.initialize()
            w = vis_gui.AppWindow(1024, 768)
            w.set_points(points_3D)
            gui.Application.instance.run()

            R = np.matrix([
                [ math.cos(w.rot_y) * math.cos(w.rot_z), -math.sin(w.rot_z), math.sin(w.rot_y)],
                [ math.sin(w.rot_z), math.cos(w.rot_x) * math.cos(w.rot_z), -math.sin(w.rot_x)],
                [ -math.sin(w.rot_y), math.sin(w.rot_x), math.cos(w.rot_x) * math.cos(w.rot_y)]])
            new_points = np.array(points_3D * R)
            axis_min = np.amin(new_points, axis=0)
            axis_min_abs = axis_min * -1
            new_points += [0, 0, axis_min_abs[2]]
            axis_max = np.amax(new_points, axis=0)
            axis_max_abs = np.maximum(axis_min_abs, axis_max)
            max_val = np.maximum(axis_max_abs[0], axis_max_abs[1])
            new_points /= max_val

            #w = vis_gui.AppWindow(1024, 768)
            #w.set_points(new_points)
            #gui.Application.instance.run()

            filename = os.path.join(self.results_path, 'coords_gift.csv')
            np.savetxt(filename, new_points, fmt='%.9f', delimiter=",")
            filename = os.path.join(self.results_path, 'coords.npy')
            np.save(filename, new_points, allow_pickle=False)

    def create_reprojected_images(self, points_3D = None):
        if points_3D is None:
            points_3D = self.points_3D

        for view in self.views:
            print('Reprojecting: ', view.name)
            image = cv2.imread(os.path.join(view.root_path, 'features', view.name + '.jpg'))

            reprojected_points, _ = cv2.projectPoints(points_3D, view.R, view.t, self.K, None)
            for idx, point_2D in enumerate(reprojected_points):
                x = int(point_2D[0][0])
                y = int(point_2D[0][1])
                if np.array_equal(points_3D[idx], self.points_3D[idx]):
                    color = (255, 255, 255)
                else:
                    color = (0, 0, 255)
                cv2.circle(image, (x, y), 5, color, 2)
                cv2.putText(image, "#{}".format(idx), (x, y - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)
                if idx > 0:
                    cv2.line(image, (x, y), (prev_x, prev_y), (128, 128, 128))
                
                prev_x = x
                prev_y = y
            
            cv2.imwrite(os.path.join(view.root_path, 'features', view.name + '_reprojected.jpg'), image)
            
    def reconstruct(self):
        """Starts the main reconstruction loop for a given set of views and matches"""

        # compute baseline pose
        baseline_view1, baseline_view2 = self.views[0], self.views[1]
        logging.info("Computing baseline pose and reconstructing points")
        self.compute_pose(view1=baseline_view1, view2=baseline_view2, is_baseline=True)
        logging.info("Mean reprojection error for 1 image is %f", self.errors[0])
        logging.info("Mean reprojection error for 2 images is %f", self.errors[1])
        self.plot_points()
        logging.info("Points plotted for %d views", len(self.done))

        for i in range(2, len(self.views)):

            logging.info("Computing pose and reconstructing points for view %d - %s", i+1, self.views[i].name)
            self.compute_pose(view1=self.views[i])
            logging.info("Mean reprojection error for %d images is %f", i+1, self.errors[i])
            self.plot_points()
            logging.info("Points plotted for %d views", i+1)
        points_3D_final = self.fix_points()
        self.create_reprojected_images(points_3D_final)
        print("fix rotation")
        self.plot_points(points_3D_final, name="final", display=True)
