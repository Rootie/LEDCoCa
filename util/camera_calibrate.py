import cv2 as cv
import numpy as np
import argparse
import os

class VideoCalibrate:
    def __init__(self):
        self.chessboardSize = (9,6)
        # termination criteria
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((self.chessboardSize[0] * self.chessboardSize[1], 3), np.float32)
        objp[:,:2] = np.mgrid[0:self.chessboardSize[0],0:self.chessboardSize[1]].T.reshape(-1,2)

        self.size_of_chessboard_squares_mm = 41
        self.objp = objp * self.size_of_chessboard_squares_mm


        # Arrays to store object points and image points from all the images.
        self.objpoints = [] # 3d point in real world space
        self.imgpoints = [] # 2d points in image plane.

    def calibrate(self, pathIn):
        count = 0
        vidcap = cv.VideoCapture(pathIn)
        while True:
            vidcap.set(cv.CAP_PROP_POS_MSEC, (count*1000))
            success,image = vidcap.read()
            if not success:
                break
            print('Read a new frame: ', count)

            orig = image.copy()
            gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
            self.frameSize = gray.shape[::-1]

            # Find the chess board corners
            ret, corners = cv.findChessboardCorners(gray, self.chessboardSize, None)

            # If found, add object points, image points (after refining them)
            if ret == True:
                self.objpoints.append(self.objp)
                corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), self.criteria)
                self.imgpoints.append(corners)

                # Draw and display the corners
                cv.drawChessboardCorners(orig, self.chessboardSize, corners2, ret)

            cv.imshow('img', orig)
            key = cv.waitKey(1000)
            if key == 27:
                break
            count = count + 1
        
        ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(self.objpoints, self.imgpoints, self.frameSize, None, None)
        self.cameraMatrix = cameraMatrix
        print('cameraMatrix: ')
        print(cameraMatrix)

        # Reprojection Error
        mean_error = 0

        for i in range(len(self.objpoints)):
            imgpoints2, _ = cv.projectPoints(self.objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
            error = cv.norm(self.imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
            mean_error += error
        print( "total error: {}".format(mean_error/len(self.objpoints)) )


if __name__=="__main__":
    a = argparse.ArgumentParser()
    a.add_argument("--file", help="path to calibration video")
    args = a.parse_args()

    vc = VideoCalibrate()
    vc.calibrate(args.file)
    dir = os.path.dirname(os.path.abspath(args.file))
    K = np.savetxt(os.path.join(dir, 'K.txt'), vc.cameraMatrix)
