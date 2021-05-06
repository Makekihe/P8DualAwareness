#!/usr/bin/env python
import math
import statistics
import tensorflow as tf
import cv2
import pyrealsense2 as rs
import numpy as np
import time
import argparse

import geometry_msgs.msg
from geometry_msgs.msg import Transform
import rospy
import posenet

parser = argparse.ArgumentParser()
parser.add_argument('--model', type=int, default=101)
parser.add_argument('--cam_id', type=int, default=0)
parser.add_argument('--cam_width', type=int, default=1280)
parser.add_argument('--cam_height', type=int, default=720)
parser.add_argument('--scale_factor', type=float, default=0.715)  #0.7125 original value
parser.add_argument('--notxt', action='store_true')
parser.add_argument('--file', type=str, default=None, help="Optionally use a video file instead of a live camera")
args = parser.parse_args()

amin_part_score = 0.08
amin_pose_score = 0.1

def stand_sit(side):
    shoulder2hip = math.sqrt(pow(side[0][0]-side[1][0], 2)+pow(side[0][1]-side[1][1], 2))
    shoulder2knee = math.sqrt(pow(side[0][0]-side[2][0], 2)+pow(side[0][1]-side[2][1], 2))
    #print(shoulder2hip)
    #print(shoulder2knee)

    if shoulder2hip/shoulder2knee < 0.65: #if standing
        return 1
    else:
        return 0

def check_side(side):
    # good_thresh = 0.1
    for i in range(3):
        if side[i][0] < amin_part_score:
            return 0
    return 1


def main():

    xcent = 0
    ycent = 0

    xstart = 0
    xend = 0

    ystart = 0

    yend = 0

    StanSmith = 0

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))


    config.enable_stream(rs.stream.color, args.cam_width, args.cam_height, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    with tf.Session() as sess:
        model_cfg, model_outputs = posenet.load_model(args.model, sess)
        output_stride = model_cfg['output_stride']

       # ROS exstra
        pub = rospy.Publisher('body_pose', Transform, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(100)  # 10hz

        count_stand = 0
        count_sit = 0
        count_unknown = 0

        pose = "unknown"
        color1 = (255, 255, 255)

        queSize = 1
        q1 = []
        for i in range(queSize):
            q1.append(0)

        start = time.time()
        frame_count = 0
        while not rospy.is_shutdown():

            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            input_image, display_image, output_scale = posenet.read_cap(
                color_image, scale_factor=args.scale_factor, output_stride=output_stride)

            heatmaps_result, offsets_result, displacement_fwd_result, displacement_bwd_result = sess.run(
                model_outputs,
                feed_dict={'image:0': input_image}
            )

            pose_scores, keypoint_scores, keypoint_coords = posenet.decode_multi.decode_multiple_poses(
                heatmaps_result.squeeze(axis=0),
                offsets_result.squeeze(axis=0),
                displacement_fwd_result.squeeze(axis=0),
                displacement_bwd_result.squeeze(axis=0),
                output_stride=output_stride,
                max_pose_detections=10,
                min_pose_score=amin_pose_score)

            keypoint_coords *= output_scale

            if not args.notxt:
                print()
                print("Results for image:")
                for pi in range(len(pose_scores)):
                    if pose_scores[pi] == 0.:
                        break
                    print('Pose #%d, score = %f' % (pi, pose_scores[pi]))

                    poses = [keypoint_scores[pi, :], keypoint_coords[pi, :, :]]
                    left_index = [5, 11, 13]
                    right_index = [6, 12, 14]
                    left_side = []
                    right_side = []

                    for i in left_index:
                        left_side.append([poses[0][i], round(poses[1][i][0]), round(poses[1][i][1])])

                    for i in right_index:
                        right_side.append([poses[0][i], round(poses[1][i][0]), round(poses[1][i][1])])


                    if check_side(right_side):
                        if stand_sit(right_side) == 1:
                            q1.pop(0)
                            q1.append(1)
                        else:
                            q1.pop(0)
                            q1.append(-1)
                    elif check_side(left_side):
                        if stand_sit(left_side) == 1:
                            q1.pop(0)
                            q1.append(1)
                        else:
                            q1.pop(0)
                            q1.append(-1)
                    else:
                        q1.pop(0)
                        q1.append(0)
                    print(q1)
                    mean_pose = statistics.mean(q1)

                    if mean_pose > 0:
                        pose = "stand"
                        color1 = (255,0,0)
                        count_stand = count_stand + 1
                        StanSmith = 1
                    elif mean_pose < 0:
                        pose = "sit"
                        color1 = (0, 255, 0)
                        count_sit = count_sit + 1
                        StanSmith = 2
                    else:
                        pose = "unknown"
                        color1 = (0, 0, 255)
                        count_unknown = count_unknown + 1
                        StanSmith = 0


                    print("meanpose: %f" % mean_pose)
                    print(pose)

                    lSs = poses[0][5]  # lShoulder score
                    rSs = poses[0][6]  # rShoulder score
                    lHs = poses[0][11]  # lHip score
                    rHs = poses[0][12]  # rHip score
                    lSc = poses[1][5]  # lShoulder coordinates
                    rSc = poses[1][6]  # rShoulder coordinates
                    lHc = poses[1][11]  # lHip coordinates
                    rHc = poses[1][12]  # rHip coordinates

                    if lSs > amin_part_score:  # Check if left shoulder has been spotted
                        lS = 1
                    else:
                        lS = 0

                    if rSs > amin_part_score:  # Check if right shoulder has been spotted
                        rS = 1
                    else:
                        rS = 0

                    if lHs > amin_part_score:  # Check if left hip has been spotted
                        lH = 1
                    else:
                        lH = 0

                    if rHs > amin_part_score:  # Check if right hip has been spotted
                        rH = 1
                    else:
                        rH = 0

                    if lS != 0 or rS != 0:  # At least one shoulder visible
                        if lS == 1 and rS == 1:  # Both shoulders visible
                            if lH == 1 and rH == 1:  # Both shoulders, both hips
                                print("Both shoulders, both hips")
                                ycent = (lSc[0] + rSc[0] + lHc[0] + rHc[0]) / 4
                                xcent = (lSc[1] + rSc[1] + lHc[1] + rHc[1]) / 4

                                # Box creator
                                buffer = min(abs(lSc[1] - rSc[1]), abs(lHc[1] - rHc[1])) / 2
                                buffer *= 0.80
                                xstart = xcent - buffer
                                xend = xcent + buffer
                                ystart = ycent - buffer
                                yend = ycent + buffer

                            elif lH == 0 and rH == 0:
                                print("Only shoulders")
                                # Do coordinates from both shoulders only
                                ycent = (lSc[0] + rSc[0]) / 2
                                xcent = (lSc[1] + rSc[1]) / 2

                                buffer = abs(lSc[1] - rSc[1]) / 2
                                buffer *= 0.40
                                xstart = xcent - buffer
                                xend = xcent + buffer
                                ystart = ycent - buffer
                                yend = ycent + buffer

                            elif lH == 1:
                                print("Both shoulders, left hip")
                                # Do triangle, left hip both shoulders
                                ycent = (lSc[0] + rSc[0] + lHc[0]) / 3
                                xcent = (lSc[1] + rSc[1] + lHc[1]) / 3

                                buffer = abs(lSc[1] - rSc[1]) / 2
                                buffer *= 0.80
                                xstart = xcent - buffer
                                xend = xcent + buffer
                                ystart = ycent - buffer
                                yend = ycent + buffer

                            else:
                                print("Both shoulders, right hip")
                                # Do triangle right hip both shoulders
                                ycent = (lSc[0] + rSc[0] + rHc[0]) / 3
                                xcent = (lSc[1] + rSc[1] + rHc[1]) / 3

                                buffer = abs(lSc[1] - rSc[1]) / 2
                                buffer *= 0.80
                                xstart = xcent - buffer
                                xend = xcent + buffer
                                ystart = ycent - buffer
                                yend = ycent + buffer

                        else:  # Only one shoulder visible
                            if lS == 1:  # left Shoulder
                                if lH == 1 and rH == 1:
                                    print("Left shoulder, both hips")
                                    # Do lower triangle, left shoulder
                                    ycent = (lSc[0] + lHc[0] + rHc[0]) / 3
                                    xcent = (lSc[1] + lHc[1] + rHc[1]) / 3

                                    buffer = abs(lHc[1] - rHc[1]) / 2
                                    buffer *= 0.75
                                    xstart = xcent - buffer
                                    xend = xcent + buffer
                                    ystart = ycent - buffer
                                    yend = ycent + buffer

                                elif rH == 1:
                                    print("Left shoulder, right hip")
                                    #  Do center between left shoulder and right hip
                                    ycent = (lSc[0] + rHc[0]) / 2
                                    xcent = (lSc[1] + rHc[1]) / 2

                                    buffer = abs(lSc[1] - rHc[1]) / 2
                                    buffer *= 0.80
                                    xstart = xcent - buffer
                                    xend = xcent + buffer
                                    ystart = ycent - buffer
                                    yend = ycent + buffer

                            else:  # right shoulder
                                if lH == 1 and rH == 1:
                                    print("Right shoulder, both hips")
                                    # Do lower triangle, right shoulder
                                    ycent = (rSc[0] + lHc[0] + rHc[0]) / 3
                                    xcent = (rSc[1] + lHc[1] + rHc[1]) / 3

                                    buffer = abs(lHc[1] - rHc[1]) / 2
                                    buffer *= 0.75
                                    xstart = xcent - buffer
                                    xend = xcent + buffer
                                    ystart = ycent - buffer
                                    yend = ycent + buffer

                                elif lH == 1:
                                    print("Right shoulder, left hip")
                                    #  Do center between right shoulder and left hip
                                    ycent = (rSc[0] + lHc[0]) / 2
                                    xcent = (rSc[1] + lHc[1]) / 2

                                    buffer = abs(lHc[1] - rSc[1]) / 2
                                    buffer *= 0.80
                                    xstart = xcent - buffer
                                    xend = xcent + buffer
                                    ystart = ycent - buffer
                                    yend = ycent + buffer
                    else:
                        print('no torso spotted')
                        xcent = 0
                        ycent = 0


                    cv2.circle(display_image, (round(xcent), round(ycent)), 5, color1,
                               2)  # Draw a circle
                    cv2.rectangle(display_image, (round(xend), round(yend)),
                                  (round(xstart), round(ystart)), color1, 2)



                    # ROS HERE PLS
                    rosmsg = geometry_msgs.msg.Transform()

                    rosmsg.translation.x = pi
                    rosmsg.translation.y = StanSmith
                    rosmsg.translation.z = 0

                    rosmsg.rotation.x = round(xstart)
                    rosmsg.rotation.y = round(ystart)
                    rosmsg.rotation.z = round(xend)
                    rosmsg.rotation.w = round(yend)

                    print(pi)
                    rospy.loginfo(rosmsg)
                    pub.publish(rosmsg)
                    rate.sleep()

            # TODO this isn't particularly fast, use GL for drawing and display someday...
            overlay_image = posenet.draw_skel_and_kp(
                display_image, pose_scores, keypoint_scores, keypoint_coords,
                min_pose_score=amin_pose_score, min_part_score=amin_part_score)

            #cv2.putText(overlay_image, "Unknown: %f" % count_unknown, (50,50), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            #cv2.putText(overlay_image, "Sitting: %f" % count_sit, (50,100), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            #cv2.putText(overlay_image, "Standing: %f" % count_stand, (50,150), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            cv2.imshow('posenet', overlay_image)
            frame_count += 1
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            print("Unknown: %f" % count_unknown)
            print("Sitting: %f" % count_sit)
            print("Standing: %f" % count_stand)



        print('Average FPS: ', frame_count / (time.time() - start))


if __name__ == "__main__":
    main()
