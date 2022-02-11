import os
import pickle
import logging


class Match:
    """Represents a feature matches between two views"""

    def __init__(self, view1, view2, match_path):

        self.indices1 = []  # indices of the matched keypoints in the first view
        self.indices2 = []  # indices of the matched keypoints in the second view
        self.distances = []  # distance between the matched keypoints in the first view
        self.image_name1 = view1.name  # name of the first view
        self.image_name2 = view2.name  # name of the second view
        self.root_path = view1.root_path  # root directory containing the image folder
        self.inliers1 = []  # list to store the indices of the keypoints from the first view not removed using the fundamental matrix
        self.inliers2 = []  # list to store the indices of the keypoints from the second view not removed using the fundamental matrix
        self.view1 = view1
        self.view2 = view2

        if not match_path:
            self.get_matches(view1, view2)
        else:
            self.read_matches()

    def get_matches(self, view1, view2):
        """Extracts feature matches between two views"""

        # store match components in their respective lists
        for i in range(view1.led_count): #TODO LED Count
            try:
                idx1 = view1.descriptors.index(i)
                idx2 = view2.descriptors.index(i)
                self.indices1.append(idx1)
                self.indices2.append(idx2)
                self.distances.append(0)
            except:
              pass

        logging.info("Computed matches between view %s and view %s", self.image_name1, self.image_name2)

        self.write_matches()

    def write_matches(self):
        """Writes a match to a pkl file in the root_path/matches directory"""

        if not os.path.exists(os.path.join(self.root_path, 'matches')):
            os.makedirs(os.path.join(self.root_path, 'matches'))

        temp_array = []
        for i in range(len(self.indices1)):
            temp = (self.distances[i], self.indices1[i], self.indices2[i])
            temp_array.append(temp)

        matches_file = open(os.path.join(self.root_path, 'matches', self.image_name1 + '_' + self.image_name2 + '.pkl'), 'wb')
        pickle.dump(temp_array, matches_file)
        matches_file.close()

    def read_matches(self):
        """Reads matches from file"""

        try:
            matches = pickle.load(
                open(
                    os.path.join(self.root_path, 'matches', self.image_name1 + '_' + self.image_name2 + '.pkl'),
                    "rb"
                )
            )
            logging.info("Read matches from file for view pair pair %s %s", self.image_name1, self.image_name2)

            for point in matches:
                self.distances.append(point[0])
                self.indices1.append(point[1])
                self.indices2.append(point[2])

        except FileNotFoundError:
            logging.error("Pkl file not found for match %s_%s. Computing from scratch", self.image_name1, self.image_name2)
            self.get_matches(self.view1, self.view2)


def create_matches(views):
    """Computes matches between every possible pair of views and stores in a dictionary"""

    match_path = False

    root_path = views[0].root_path

    if os.path.exists(os.path.join(root_path, 'matches')):
        match_path = True

    matches = {}
    for i in range(0, len(views) - 1):
        for j in range(i+1, len(views)):
            matches[(views[i].name, views[j].name)] = Match(views[i], views[j], match_path)

    return matches
