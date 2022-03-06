from enum import Enum, auto
import os
import pickle
import cv2
import numpy as np
import glob
import logging


class View:
    """Represents an image used in the reconstruction"""

    def __init__(self, image_path, root_path, feature_path, led_count):

        self.name = image_path[image_path.rfind('/') + 1:-4]  # image name without extension
        self.image = cv2.VideoCapture(image_path)
        self.keypoints = []  # list of keypoints obtained from feature extraction
        self.descriptors = []  # list of descriptors obtained from feature extraction, equals the led nr
        self.led_count = led_count
        self.root_path = root_path  # root directory containing the image folder
        self.R = np.zeros((3, 3), dtype=float)  # rotation matrix for the view
        self.t = np.zeros((3, 1), dtype=float)  # translation vector for the view

        if not feature_path:
            self.extract_features()
        else:
            self.read_features()

    def get_image_tint(self, frame_nr):
        self.image.set(cv2.CAP_PROP_POS_FRAMES, frame_nr)
        success,image = self.image.read()
        if not success:
            raise Exception()

        hist_b = cv2.calcHist([image],[0],None,[256],[0,256])
        b_median = np.median(hist_b)
        hist_g = cv2.calcHist([image],[1],None,[256],[0,256])
        g_median = np.median(hist_g)
        hist_r = cv2.calcHist([image],[2],None,[256],[0,256])
        r_median = np.median(hist_r)

        if r_median > g_median * 1.24 and r_median > b_median * 1.24:
            tint = 'r'
        elif g_median > r_median * 1.24 and g_median > b_median * 1.24:
            tint = 'g'
        elif b_median > r_median * 1.24 and b_median > g_median * 1.24:
            tint = 'b'
        else:
            tint = 'g'
        return tint

    def extract_info_from_video(self):
        class States(Enum):
            FIND_RED_BEGINNING = auto()
            FIND_BLUE_END = auto()
        state = States.FIND_RED_BEGINNING
        start_frame = 0
        end_frame = int(self.image.get(cv2.CAP_PROP_FRAME_COUNT)) - 1

        red_beginning_frame = None

        while True:
            for frame_nr in range(start_frame, end_frame, 1 if start_frame < end_frame else -1):
                tint = self.get_image_tint(frame_nr)
                if state is States.FIND_RED_BEGINNING:
                    if tint == 'r':
                        red_beginning_frame = frame_nr

                        state = States.FIND_BLUE_END
                        start_frame = end_frame
                        end_frame = frame_nr + 1
                        break
                elif state is States.FIND_BLUE_END:
                    if tint == 'b':
                        fpl = (frame_nr + 1 - red_beginning_frame) / (self.led_count + 7)
                        offset = red_beginning_frame + 4 * fpl
                        return offset, fpl

    def extract_features(self):
        """Extracts features from the image"""

        if not os.path.exists(os.path.join(self.root_path, 'features')):
            os.makedirs(os.path.join(self.root_path, 'features'))


        offset, lit_frames = self.extract_info_from_video()
        logging.info("First LED offset: %d", offset)
        logging.info("LEDs are lit for %f frames", lit_frames)

        offset += int(lit_frames / 2)
        all_LED_image = None
        for count in range(self.led_count):
            self.image.set(cv2.CAP_PROP_POS_FRAMES, offset + (count * lit_frames))
            success,image = self.image.read()
            if not success:
                break

            #print('Read a new frame: ', count)
            view_path = os.path.join(self.root_path, 'features', self.name)
            if not os.path.exists(view_path):
                os.makedirs(view_path)
            cv2.imwrite( os.path.join(view_path, "frame%05d.jpg" % count), image)     # save frame as JPEG file
            
            if all_LED_image is not None:
                all_LED_image = all_LED_image + image
            else:
                all_LED_image = image.copy()

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # median filter
            median = cv2.medianBlur(gray, 3)

            # do canny edge detection
            canny = cv2.Canny(median, 100, 250)

            # transpose canny image to compensate for following numpy points as y,x
            canny_t = cv2.transpose(canny)

            # get canny points
            # numpy points are (y,x)
            points = np.argwhere(canny_t>0)

            if len(points) >= 5:
                # fit ellipse and get ellipse center, minor and major diameters and angle in degree
                ellipse = cv2.fitEllipse(points)
                (x,y), (d1,d2), angle = ellipse

                self.keypoints.append(cv2.KeyPoint(x, y, 1))
                self.descriptors.append(count)

        cv2.imwrite(os.path.join(self.root_path, 'features', self.name + '.jpg'), all_LED_image)
        for idx, kp in enumerate(self.keypoints):
            cv2.circle(all_LED_image, (int(kp.pt[0]), int(kp.pt[1])), 5, (255, 255, 255), 2)
            cv2.putText(all_LED_image, "#{}".format(self.descriptors[idx]), (int(kp.pt[0]), int(kp.pt[1]) - 15),
		        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 2)
            if idx > 0:
                prev_pt = self.keypoints[idx-1].pt
                cv2.line(all_LED_image, (int(kp.pt[0]), int(kp.pt[1])), (int(prev_pt[0]), int(prev_pt[1])), (128, 128, 128))
            
        cv2.imwrite(os.path.join(self.root_path, 'features', self.name + '_keypoints.jpg'), all_LED_image)

        logging.info("Computed features for image %s", self.name)

        self.write_features()

    def read_features(self):
        """Reads features stored in files. Feature files have filenames corresponding to image names without extensions"""

        # logic to compute features for images that don't have pkl files
        try:
            features = pickle.load(open(os.path.join(self.root_path, 'features', self.name + '.pkl'), "rb"))
            logging.info("Read features from file for image %s", self.name)

            keypoints = []
            descriptors = []

            for point in features:
                keypoint = cv2.KeyPoint(point[0][0], point[0][1], 1)
                descriptor = point[6]
                keypoints.append(keypoint)
                descriptors.append(descriptor)

            self.keypoints = keypoints
            self.descriptors = descriptors

        except FileNotFoundError:
            logging.error("Pkl file not found for image %s. Computing from scratch", self.name)
            self.extract_features()

    def write_features(self):
        """Stores computed features to pkl files. The files are written inside a features directory inside the root directory"""

        if not os.path.exists(os.path.join(self.root_path, 'features')):
            os.makedirs(os.path.join(self.root_path, 'features'))

        temp_array = []
        for idx, point in enumerate(self.keypoints):
            temp = (point.pt, point.size, point.angle, point.response, point.octave, point.class_id,
                    self.descriptors[idx])
            temp_array.append(temp)

        features_file = open(os.path.join(self.root_path, 'features', self.name + '.pkl'), 'wb')
        pickle.dump(temp_array, features_file)
        features_file.close()


def create_views(root_path, video_format, led_count):
    """Loops through the images and creates an array of views"""

    feature_path = False

    # if features directory exists, the feature files are read from there
    logging.info("Created features directory")
    if os.path.exists(os.path.join(root_path, 'features')):
        feature_path = True

    image_names = sorted(glob.glob(os.path.join(root_path, 'images', '*.' + video_format)))

    logging.info("Computing features")
    views = []
    for image_name in image_names:
        views.append(View(image_name, root_path, feature_path, led_count))

    return views
