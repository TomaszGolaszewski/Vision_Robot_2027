"""Camera intrinsics calibration with a ChArUco board, straight from the camera (no ROS needed).

Live preview: detected board corners in green, corners of the saved views as blue dots (aim to cover the whole image).
Keys: SPACE save the current view, c calibrate and write the YAML, q quit without saving.
Views with an error above 2x the median are dropped; the YAML is not written when the RMS stays above 1 px.
With --save-dir the saved views are also written as PNG; --from-dir calibrates from such PNGs without the camera.
"""
import argparse
import glob
import os

import cv2
import numpy as np

from robot_machining_controller.pose_math import (calibrate_charuco_robust, charuco_board, find_charuco_corners,
                                                  save_camera_info_yaml)

CAMERA = '/dev/v4l/by-id/usb-FLH-201016_Integrated_Camera-video-index0'
MIN_CORNERS = 12  # of 24 on the 7x5 board: views with only a few corners add little
MIN_VIEWS = 15
MAX_RMS_PX = 1.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--device', default=CAMERA, help='video device path or index')
    ap.add_argument('--width', type=int, default=640, help='must match the resolution used by cam2image')
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--squares', type=int, nargs=2, default=[7, 5], metavar=('X', 'Y'), help='board squares')
    ap.add_argument('--square', type=float, default=0.025, help='square side [m], measured on the print')
    ap.add_argument('--marker', type=float, default=0.018, help='marker side [m]')
    ap.add_argument('--dictionary', default='DICT_6X6_250')
    ap.add_argument('--output', default='camera_calibration.yaml', help='ROS camera_info YAML file')
    ap.add_argument('--save-dir', help='also write the saved views as PNG into this directory')
    ap.add_argument('--from-dir', help='calibrate from PNG views in this directory instead of the camera')
    ap.add_argument('--force', action='store_true', help=f'write the YAML even when RMS > {MAX_RMS_PX} px')
    args = ap.parse_args()

    board = charuco_board(*args.squares, args.square, args.marker, args.dictionary)
    if args.from_dir:
        views = []
        for path in sorted(glob.glob(os.path.join(args.from_dir, '*.png'))):
            # same grayscale conversion as live: imread(GRAYSCALE) differs slightly and loses borderline views
            view = find_charuco_corners(cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2GRAY), board, MIN_CORNERS)
            print(f'{len(views)}: {path}: {"no board" if view is None else len(view[1])}')
            if view is not None:
                views.append(view)
        calibrate(views, board, args)
        return
    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
    device = int(args.device) if args.device.isdigit() else args.device
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    views = []
    while True:
        ok, frame = cap.read()
        if not ok:
            raise SystemExit(f'no image from {args.device} (camera disconnected?)')
        if frame.shape[1::-1] != (args.width, args.height):
            raise SystemExit(f'camera gives {frame.shape[1]}x{frame.shape[0]}, not {args.width}x{args.height}')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        view = find_charuco_corners(gray, board, MIN_CORNERS)

        img = frame.copy()
        for corners, _ in views:
            for x, y in corners.reshape(-1, 2):
                cv2.circle(img, (int(x), int(y)), 2, (255, 0, 0), -1)
        if view is not None:
            cv2.aruco.drawDetectedCornersCharuco(img, *view, (0, 255, 0))
        text = f'views {len(views)}/{MIN_VIEWS}  SPACE save  c calibrate  q quit'
        cv2.putText(img, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(img, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow('calibrate_camera', img)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' ') and view is not None:
            if args.save_dir:
                cv2.imwrite(os.path.join(args.save_dir, f'view_{len(views):03d}.png'), frame)
            print(f'view {len(views)}: {len(view[1])} corners')
            views.append(view)
        elif key == ord('c'):
            if calibrate(views, board, args):
                break
        elif key == ord('q'):
            print('quit, nothing written')
            break
    cap.release()
    cv2.destroyAllWindows()


def calibrate(views, board, args):
    """Calibrate, print the result and write the YAML when it is good enough. Returns True when written."""
    if len(views) < MIN_VIEWS:
        print(f'need at least {MIN_VIEWS} views, have {len(views)}')
        return False
    K, dist, rms, dropped = calibrate_charuco_robust(views, board, (args.width, args.height))
    np.set_printoptions(precision=3, suppress=True)
    print(f'dropped outlier views (blur, bent board?): {dropped}; used {len(views) - len(dropped)}')
    print(f'RMS reprojection error: {rms:.3f} px (good: < 0.5, max {MAX_RMS_PX})')
    print(f'K:\n{K}\ndist (k1, k2, p1, p2, k3): {dist}')
    if rms > MAX_RMS_PX and not args.force:
        print('NOT written: RMS too high. Flat rigid board, hold it still, more light; or --force')
        return False
    save_camera_info_yaml(args.output, K, dist, (args.width, args.height))
    print(f'written {args.output}')
    return True


if __name__ == '__main__':
    main()
