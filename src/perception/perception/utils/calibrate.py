#!/usr/bin/env python3

import depthai as dai
import numpy as np
import sys
from pathlib import Path

# Connect Device
with dai.Device() as device:
    calibFile = str(
        (Path(__file__).parent / Path(f"calib_{device.getMxId()}.json"))
        .resolve()
        .absolute()
    )
    if len(sys.argv) > 1:
        calibFile = sys.argv[1]

    calibData = device.readCalibration()
    calibData.eepromToJsonFile(calibFile)

    M_rgb, width, height = calibData.getDefaultIntrinsics(dai.CameraBoardSocket.CAM_A)
    print("RGB Camera Default intrinsics...")
    print(M_rgb)
    print(width)
    print(height)

    if (
        "OAK-1" in calibData.getEepromData().boardName
        or "BW1093OAK" in calibData.getEepromData().boardName
    ):
        M_rgb = np.array(
            calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, 1280, 720)
        )
        print("RGB Camera resized intrinsics...")
        print(M_rgb)

        D_rgb = np.array(
            calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)
        )
        print("RGB Distortion Coefficients...")
        [
            print(name + ": " + value)
            for (name, value) in zip(
                [
                    "k1",
                    "k2",
                    "p1",
                    "p2",
                    "k3",
                    "k4",
                    "k5",
                    "k6",
                    "s1",
                    "s2",
                    "s3",
                    "s4",
                    "τx",
                    "τy",
                ],
                [str(data) for data in D_rgb],
            )
        ]

        print(f"RGB FOV {calibData.getFov(dai.CameraBoardSocket.CAM_A)}")

    else:
        M_rgb, width, height = calibData.getDefaultIntrinsics(
            dai.CameraBoardSocket.CAM_A
        )
        print("RGB Camera Default intrinsics...")
        print(M_rgb)
        print(width)
        print(height)

        M_rgb = np.array(
            calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, 3840, 2160)
        )
        print("RGB Camera resized intrinsics... 3840 x 2160 ")
        print(M_rgb)

        M_rgb = np.array(
            calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, 4056, 3040)
        )
        print("RGB Camera resized intrinsics... 4056 x 3040 ")
        print(M_rgb)

        M_left, width, height = calibData.getDefaultIntrinsics(
            dai.CameraBoardSocket.CAM_B
        )
        print("LEFT Camera Default intrinsics...")
        print(M_left)
        print(width)
        print(height)

        M_left = np.array(
            calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, 1280, 720)
        )
        print("LEFT Camera resized intrinsics...  1280 x 720")
        print(M_left)

        M_right = np.array(
            calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C, 1280, 720)
        )
        print("RIGHT Camera resized intrinsics... 1280 x 720")
        print(M_right)

        D_left = np.array(
            calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B)
        )
        print("LEFT Distortion Coefficients...")
        [
            print(name + ": " + value)
            for (name, value) in zip(
                [
                    "k1",
                    "k2",
                    "p1",
                    "p2",
                    "k3",
                    "k4",
                    "k5",
                    "k6",
                    "s1",
                    "s2",
                    "s3",
                    "s4",
                    "τx",
                    "τy",
                ],
                [str(data) for data in D_left],
            )
        ]

        D_right = np.array(
            calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_C)
        )
        print("RIGHT Distortion Coefficients...")
        [
            print(name + ": " + value)
            for (name, value) in zip(
                [
                    "k1",
                    "k2",
                    "p1",
                    "p2",
                    "k3",
                    "k4",
                    "k5",
                    "k6",
                    "s1",
                    "s2",
                    "s3",
                    "s4",
                    "τx",
                    "τy",
                ],
                [str(data) for data in D_right],
            )
        ]

        print(
            f"RGB FOV {calibData.getFov(dai.CameraBoardSocket.CAM_A)}, Mono FOV {calibData.getFov(dai.CameraBoardSocket.CAM_B)}"
        )

        R1 = np.array(calibData.getStereoLeftRectificationRotation())
        R2 = np.array(calibData.getStereoRightRectificationRotation())
        M_right = np.array(
            calibData.getCameraIntrinsics(calibData.getStereoRightCameraId(), 1280, 720)
        )

        H_left = np.matmul(np.matmul(M_right, R1), np.linalg.inv(M_left))
        print("LEFT Camera stereo rectification matrix...")
        print(H_left)

        H_right = np.matmul(np.matmul(M_right, R1), np.linalg.inv(M_right))
        print("RIGHT Camera stereo rectification matrix...")
        print(H_right)

        lr_extrinsics = np.array(
            calibData.getCameraExtrinsics(
                dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C
            )
        )
        print(
            "Transformation matrix of where left Camera is W.R.T right Camera's optical center"
        )
        print(lr_extrinsics)

        l_rgb_extrinsics = np.array(
            calibData.getCameraExtrinsics(
                dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_A
            )
        )
        print(
            "Transformation matrix of where left Camera is W.R.T RGB Camera's optical center"
        )
        print(l_rgb_extrinsics)

# RGB Camera Default intrinsics...
# [[3050.27978515625, 0.0, 1993.59228515625], [0.0, 3050.27978515625, 1087.78369140625], [0.0, 0.0, 1.0]]
# 3840
# 2160
# RGB Camera Default intrinsics...
# [[3050.27978515625, 0.0, 1993.59228515625], [0.0, 3050.27978515625, 1087.78369140625], [0.0, 0.0, 1.0]]
# 3840
# 2160
# RGB Camera resized intrinsics... 3840 x 2160
# [[3.05027979e+03 0.00000000e+00 1.99359229e+03]
#  [0.00000000e+00 3.05027979e+03 1.08778369e+03]
#  [0.00000000e+00 0.00000000e+00 1.00000000e+00]]
# RGB Camera resized intrinsics... 4056 x 3040
# [[3.22185791e+03 0.00000000e+00 2.10573169e+03]
#  [0.00000000e+00 3.22185791e+03 1.52822156e+03]
#  [0.00000000e+00 0.00000000e+00 1.00000000e+00]]
# LEFT Camera Default intrinsics...
# [[794.5697631835938, 0.0, 653.6582641601562], [0.0, 794.5697631835938, 405.7890625], [0.0, 0.0, 1.0]]
# 1280
# 800
# LEFT Camera resized intrinsics...  1280 x 720
# [[794.56976318   0.         653.65826416]
#  [  0.         794.56976318 365.7890625 ]
#  [  0.           0.           1.        ]]
# RIGHT Camera resized intrinsics... 1280 x 720
# [[798.95349121   0.         607.94848633]
#  [  0.         798.95349121 331.31161499]
#  [  0.           0.           1.        ]]
# LEFT Distortion Coefficients...
# k1: -9.049858093261719
# k2: 86.63890838623047
# p1: 0.0008891293546184897
# p2: 0.0007972149178385735
# k3: -19.350984573364258
# k4: -9.081456184387207
# k5: 85.96934509277344
# k6: -17.258333206176758
# s1: 0.0
# s2: 0.0
# s3: 0.0
# s4: 0.0
# τx: 0.0
# τy: 0.0
# RIGHT Distortion Coefficients...
# k1: -8.26738452911377
# k2: 76.22952270507812
# p1: 0.0030772723257541656
# p2: 0.0007752112578600645
# k3: -45.7986946105957
# k4: -8.31797981262207
# k5: 75.9485092163086
# k6: -44.45697021484375
# s1: 0.0
# s2: 0.0
# s3: 0.0
# s4: 0.0
# τx: 0.0
# τy: 0.0
# RGB FOV 68.7938003540039, Mono FOV 71.86000061035156
# LEFT Camera stereo rectification matrix...
# [[ 1.00743875e+00 -3.88409170e-03 -5.11711457e+01]
#  [ 3.05868032e-03  1.00448660e+00 -3.61591027e+01]
#  [ 3.16944379e-06 -3.09518393e-06  9.99054252e-01]]
# RIGHT Camera stereo rectification matrix...
# [[ 1.00191110e+00 -3.86278032e-03 -1.90179022e+00]
#  [ 3.04189783e-03  9.98975149e-01  4.49054955e-01]
#  [ 3.15205357e-06 -3.07820116e-06  9.99097359e-01]]
# Transformation matrix of where left Camera is W.R.T right Camera's optical center
# [[ 9.99991655e-01  4.08306159e-03  1.81495154e-04 -7.48413801e+00]
#  [-4.08390490e-03  9.99979556e-01  4.91825957e-03  4.55617905e-02]
#  [-1.61409887e-04 -4.91895946e-03  9.99987900e-01  2.00192183e-02]
#  [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
# Transformation matrix of where left Camera is W.R.T RGB Camera's optical center
# [[ 9.99913752e-01  1.21513158e-02 -4.98648547e-03 -3.70458198e+00]
#  [-1.21574895e-02  9.99925375e-01 -1.20961794e-03  6.85514435e-02]
#  [ 4.97141480e-03  1.27013691e-03  9.99986887e-01 -1.74082652e-01]
#  [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
