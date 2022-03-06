from traceback import format_exc
from view import *
from match import *
from sfm import *
import numpy as np
import logging
import argparse


def run(args):

    logging.basicConfig(level=logging.INFO)
    views = create_views(args.root_dir, args.video_format, args.led_count)
    matches = create_matches(views)

    led_in_view_count = np.zeros((args.led_count,1), dtype=int)
    for view in views:
        for descriptor in view.descriptors:
            led_in_view_count[descriptor] += 1

    led_match_count = np.zeros((args.led_count,1), dtype=int)
    for idx, match in matches.items():
        for indice in match.indices1:
            led_match_count[match.view1.descriptors[indice]] += 1
    
    print("LED Statistics: (nr, view count, match count)")
    for i in range(args.led_count):
        print(f"LED {i:>3}: {led_in_view_count[i][0]:>3}, {led_match_count[i][0]:>3}")

    K = np.loadtxt(os.path.join(args.root_dir, 'K.txt'))
    sfm = SFM(views, matches, K)
    sfm.reconstruct()


def set_args(parser):

    parser.add_argument('--root_dir', action='store', type=str, dest='root_dir',
                        help='root directory containing the images/ folder')
    parser.add_argument('--video_format', action='store', type=str, dest='video_format', default='mp4',
                        help='extension of the images in the images/ folder')
    parser.add_argument('--led_count', action='store', type=int, dest='led_count', default=100,
                        help='amount of LEDs')


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    set_args(parser)
    args = parser.parse_args()
    run(args)
