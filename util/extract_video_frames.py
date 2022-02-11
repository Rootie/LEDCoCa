import argparse
import os
import glob

import cv2

def extractImages(root_path, pathIn):
    name = pathIn[pathIn.rfind('/') + 1:-4]
    pathOut = os.path.join(root_path, 'images_scaled', name)
    if not os.path.exists(pathOut):
        os.makedirs(pathOut)

    vidcap = cv2.VideoCapture(pathIn)
    for idx in range(150):
        vidcap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        success,image = vidcap.read()
        if not success:
            break

        print('Read a new frame: ', idx)
        scaled = cv2.resize(image, None, fx=0.5, fy=0.5)
        cv2.imwrite( os.path.join(pathOut, "frame%05d.jpg" % idx), scaled)


if __name__ == "__main__":
    a = argparse.ArgumentParser()
    a.add_argument('--root_dir', action='store', type=str, dest='root_dir',
                        help='root directory containing the images/ folder')
    a.add_argument('--video_format', action='store', type=str, dest='video_format', default='mp4',
                        help='extension of the images in the images/ folder')
    args = a.parse_args()

    video_names = glob.glob(os.path.join(args.root_dir, 'images', '*.' + args.video_format))
    for video_name in video_names:
        extractImages(args.root_dir, video_name)
