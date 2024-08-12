import cv2 as cv
from cv2 import aruco
import numpy as np
import socket
from time import sleep, time
import math
#import calibration
import json
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd

# Replace with the IP address of your ESP32 and the port number
target1_IP = '192.168.4.10'
target_PORT = 12345
x,y,z,theta,phi,psi = [0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0]
x_test = []
# Define the current and target positions and orientations
#x1, y1, z1, theta1, phi1, psi1 = 0, 0, 0 , 0, 0, 0 # Current position and orientation (in radians)
#x2, y2, z2, theta2 , phi2, psi2 = 0, 0, 0 , 0, 0, 0 # Target position and orientation (in radians)
#x3, y3, z3, theta3 , phi3, psi3 = 0, 0, 0 , 0, 0, 0 # Current position and orientation (in radians)
#x4, y4, z4, theta4, phi4, psi4 = 0, 0, 0 , 0, 0, 0 # Target position and orientation (in radians)
#x5, y5, z5, theta5,phi5, psi5 = 0, 0, 0 , 0, 0, 0 # Target position and orientation (in radians)
#x6, y6, z6, theta6,phi6, psi6 = 0, 0, 0 , 0, 0, 0 # Target position and orientation (in radians)
#x7, y7, z7, theta7,phi7, psi7 = 0, 0, 0 , 0, 0, 0 # Target position and orientation (in radians)
#x_robot, y_robot,z_robot, theta_robot,phi_robot, psi_robot = 0,0,0,0,0,0            

def send_command(command):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((target1_IP, target_PORT))
        s.sendall((command + '\n').encode('utf-8'))
        print(f"Command '{command}' sent")
        s.close()

#def command_to_robot(command, robot_name):
def receive_data_from_robot(robot_name):
    host = robot_name
    port = 80
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    #print(f'Sent_Command: {command}')
    #s.sendall(bytes(str(command), 'utf-8'))
    #sleep(5)
    #s.close()
    #with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
     #   s.connect((target1_IP, target_PORT))
    #try:
        # Create a socket object
     #   client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect to the server
     #   client_socket.connect((host, port))
     #   print(f"Connected to server at {host}:{port}")

        # Send a message to the server
        #message = "Hello from Python client!"
        #client_socket.sendall(message.encode('utf-8'))

        # Continuously read data from the server
    #while True:
            # Receive data from the server
    data = s.recv(1024)  # Buffer size of 1024 bytes
            #if not data:
            #    break  # Exit loop if no more data is received

            # Decode the received bytes to a string
    received_message = data.decode('utf-8')
    print(f"Received from server: {received_message}")
    sleep(1)
            #break
    s.close()

    #except Exception as e:
     #   print(f"Error: {e}")
    #finally:
        # Ensure the client socket is closed
       # if 'client_socket' in locals():
        #    client_socket.close()
        #    print("Disconnected from server")

def read_frame():
    ret, frame = cap.read()
    
    if not ret:
        #break
        return
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )
    if marker_corners:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef
        )
        total_markers = range(0, marker_IDs.size)
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
            )
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()

            # Calculating the distance
            distance = np.sqrt(
                tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
            )

            # Draw the pose of the marker
            point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)

            # Get the rotation matrix and convert it to Euler angles
            rmat = cv.Rodrigues(rVec[i])[0]
            sy = np.sqrt(rmat[0, 0] ** 2 + rmat[1, 0] ** 2)
            singular = sy < 1e-6
            if not singular:
                x_angle = np.arctan2(rmat[2, 1], rmat[2, 2])
                y_angle = np.arctan2(-rmat[2, 0], sy)
                z_angle = np.arctan2(rmat[1, 0], rmat[0, 0])
            else:
                x_angle = np.arctan2(-rmat[1, 2], rmat[1, 1])
                y_angle = np.arctan2(-rmat[2, 0], sy)
                z_angle = 0

            # θ is the yaw angle (rotation around Z-axis)
            theta_rot = np.degrees(x_angle)
            phi_rot = np.degrees(y_angle)
            psi_rot = np.degrees(z_angle)



            cv.putText(
                frame,
                f"id: {ids[0]} Dist: {round(distance, 2)}",
                top_right,
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (0, 0, 255),
                2,
                cv.LINE_AA,
            )
                         # Assign current and target positions and orientations
            phi1, phi2, phi6, phi7 = 0,0,0,0
           # x,y,z,theta,phi,psi = [0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0]
            
            if ids[0] == 3:
                x3, y3, z3 ,theta3, phi3, psi3 = (tVec[i][0][0]), tVec[i][0][1]  , tVec[i][0][2] , theta_rot, phi_rot, psi_rot
               # x3, y3, z3 = transform_point(x3, y3, z3, transformation_matrix)
                putText(x3, y3, z3, theta3, phi3, psi3, frame, bottom_right)
                #x.append(x3)
                #y.append(y3)
                #z.append(z3)
                #theta.append(theta3)
                #phi.append(phi3)
                #psi.append(psi3)
                x[2] = x3
                y[2] = y3
                z[2] = z3
                theta[2] = theta3
                phi[2] = phi3
                psi[2] = psi3
               # prin            # Assign current and target positions and orientations
            elif ids[0] == 1:
                x1, y1, z1,  theta1, phi1, psi1 = (tVec[i][0][0] ) , tVec[i][0][1]  , tVec[i][0][2] , theta_rot, -phi_rot + 90 , psi_rot
                #x1, y1, z1,  theta1 = (tVec[i][0][0]) , tVec[i][0][1] , tVec[i][0][2] , theta
              
                #putText(x1, y1, z1, theta1, phi1, psi1)
               # print(f"Position: x1, y1 ({x1:.2f}, {y1:.2f})")
                #x.append(x1)
                #y.append(y1)
                #z.append(z1)
                #theta.append(theta1)
                #phi.append(phi1)
                #psi.append(psi1)
                x[0] = x1
                y[0] = y1
                z[0] = z1
                theta[0] = theta1
                phi[0] = phi1
                psi[0] = psi1
                x_test.append(x1)

                phi_robot = phi[0]
                theta_robot = theta[0]
                psi_robot = psi[0]
                x_robot = x[0]
                y_robot = y[0]
                z_robot = z[0]
                if phi_robot <0:
                    phi_robot = phi_robot + 360
                putText(x_robot, y_robot, z_robot, 0, phi_robot, 0, frame, bottom_right)

                x[7] = x_robot
                y[7] = y_robot
                z[7] = z_robot
                theta[7] = theta_robot
                phi[7] = phi_robot
                psi[7] = psi_robot
            elif ids[0] == 2:
                x[1], y[1], z[1], theta[1], phi[1], psi[1] = tVec[i][0][0] , tVec[i][0][1] , tVec[i][0][2], theta_rot, -phi_rot  , psi_rot
                #putText(x2, y2, z2,theta2, phi2, psi2) 
               # print(f"Position: x2, y2 ({x2:.2f}, {y2:.2f})")
                #x.append(x2)
                #y.append(y2)
                #z.append(z2)
                #theta.append(theta2)
                #phi.append(phi2)
                #psi.append(psi2)
                x_test.append(x[1])
                phi_robot = phi[1]
                theta_robot = theta[1]
                psi_robot = psi[1]
                x_robot = x[1]
                y_robot = y[1]
                z_robot = z[1]

                if phi_robot <0:
                    phi_robot = phi_robot + 360
                putText(x_robot, y_robot, z_robot, 0, phi_robot, 0, frame, bottom_right)
                x[7] = x_robot
                y[7] = y_robot
                z[7] = z_robot
                theta[7] = theta_robot
                phi[7] = phi_robot
                psi[7] = psi_robot

            elif ids[0] == 4:
                x[3], y[3], z[3], theta[3], phi[3], psi[3] = (tVec[i][0][0]), tVec[i][0][1] , tVec[i][0][2] , theta_rot, phi_rot, psi_rot
                #x4, y4, z4, theta2 = (tVec[i][0][0] - x2 + 0.05*tVec[i][0][2]), tVec[i][0][1] - y2, tVec[i][0][2] - z2, theta#(f"Position: x3, y3 ({x3:.2f}, {y3:.2f})")
                #x4, y4, z4 = transform_point(x4, y4, z4, transformation_matrix)
                putText(x[3], y[3], z[3], theta[3], phi[3], psi[3], frame, bottom_right)
                #x.append(x4)
                #y.append(y4)
                #z.append(z4)
                #theta.append(theta4)
                #phi.append(phi4)
                #psi.append(psi4)
            elif ids[0] == 5:
                x[4], y[4], z[4], theta[4], phi[4],psi[4] = (tVec[i][0][0] ), tVec[i][0][1] , tVec[i][0][2] , theta_rot, phi_rot, psi_rot#(f"Position: x3, y3 ({x3:.2f}, {y3:.2f})")
                #x5, y5, z5 = transform_point(x5, y5, z5, transformation_matrix)
                putText(x[4],y[4], z[4], theta[4], phi[4], psi[4], frame, bottom_right)
                #x.append(x5)
                #y.append(y5)
                #z.append(z5)
                #theta.append(theta5)
                #phi.append(phi5)
                #psi.append(psi5)
            elif ids[0] == 6:
                x[5], y[5], z[5], theta[5], phi[5],psi[5] = (tVec[i][0][0] ), tVec[i][0][1] , tVec[i][0][2] , theta_rot, -phi_rot - 90, psi_rot#(f"Position: x3, y3 ({x3:.2f}, {y3:.2f})")
                #x5, y5, z5 = transform_point(x5, y5, z5, transformation_matrix)
                #putText(x6,y6, z6, theta6, phi6, psi6)
                #x.append(x6)
                #y.append(y6)
                #z.append(z6)
                #theta.append(theta6)
                #phi.append(phi6)
                #psi.append(psi6)
                x_test.append(x[5])
                phi_robot = phi[5]
                theta_robot = theta[5]
                psi_robot = psi[5]
                x_robot = x[5]
                y_robot = y[5]
                z_robot = z[5]

                if phi_robot <0:
                    phi_robot = phi_robot + 360
                putText(x_robot, y_robot, z_robot, 0, phi_robot, 0, frame, bottom_right)

                x[7] = x_robot
                y[7] = y_robot
                z[7] = z_robot
                theta[7] = theta_robot
                phi[7] = phi_robot
                psi[7] = psi_robot
            elif ids[0] == 7:
                x[6], y[6], z[6], theta[6], phi[6],psi[6] = (tVec[i][0][0] ), tVec[i][0][1] , tVec[i][0][2] , theta_rot, -phi_rot + 180, psi_rot#(f"Position: x3, y3 ({x3:.2f}, {y3:.2f})")
                #x5, y5, z5 = transform_point(x5, y5, z5, transformation_matrix)
                #putText(x7,y7, z7, theta7, phi7, psi7)
                #x.append(x7)
                #y.append(y7)
                #z.append(z7)
                #theta.append(theta7)
                #phi.append(phi7)
                #psi.append(psi7)
                x_test.append(x[6])
                phi_robot = phi[6]
                theta_robot = theta[6]
                psi_robot = psi[6]
                x_robot = x[6]
                y_robot = y[6]
                z_robot = z[6]
                if phi_robot <0:
                    phi_robot = phi_robot + 360
                putText(x_robot, y_robot, z_robot, 0, phi_robot, 0, frame, bottom_right)

                x[7] = x_robot
                y[7] = y_robot
                z[7] = z_robot
                theta[7] = theta_robot
                phi[7] = phi_robot
                psi[7] = psi_robot
            else:
                break

            '''
            cv.putText(
                frame,
                f"x: {round(tVec[i][0][0], 1)} y: {round(tVec[i][0][1], 1)} z: {round(tVec[i][0][2], 1)} θ: {round(theta, 1)}",
                bottom_left,
                cv.FONT_HERSHEY_PLAIN,
                1.0,
                (0, 0, 255),
                2,
                cv.LINE_AA,
            )
            '''

                #print(f"Position: x4, y4 ({x4:.2f}, {y4:.2f})")
            # Call the function to navigate the robot
            '''
            if phi1 !=0:
                phi_robot = phi[0]
                theta_robot = theta[0]
                psi_robot = psi[0]
                x_robot = x[0]
                y_robot = y[0]
                z_robot = z[0]
            elif phi2 != 0:
                phi_robot = phi[1]
                theta_robot = theta[1]
                psi_robot = psi[1]
                x_robot = x[1]
                y_robot = y[1]
                z_robot = z[1]
            elif phi6 != 0:
                phi_robot = phi[5]
                theta_robot = theta[5]
                psi_robot = psi[5]
                x_robot = x[5]
                y_robot = y[5]
                z_robot = z[5]
            elif phi7 !=0:
                phi_robot = phi[6]
                theta_robot = theta[6]
                psi_robot = psi[6]
                x_robot = x[6]
                y_robot = y[6]
                z_robot = z[6]
            else:
                break
            '''
            

            #x.append(x_robot)
            #y.append(y_robot)
            #z.append(z_robot)
            #theta.append(theta_robot)
            #phi.append(phi_robot)
            #psi.append(psi_robot)
            #x[7] = x_robot
            #y[7] = y_robot
            #z[7] = z_robot
            #theta[7] = theta_robot
            #phi[7] = phi_robot
            #psi[7] = psi_robot
            #navigate_robot(x_robot, z_robot, phi_robot,x3, z3,phi3)
            #navigate_robot(x_robot, z_robot, phi_robot,x4, z4,phi4)
            #navigate_robot(x_robot, z_robot, phi_robot,x5, z5,phi5)
            # Example: send a reset command


         
            #navigate_robot(x1, z1, phi1+psi1, x3, z3, phi3+psi3)
            #navigate_robot(0, 0, 0, x3, z3, phi3)
            #navigate_robot(x1, z1, phi_robot, x3, z3, phi3)    
            #command_to_robot(command_robot_move_right, robot1)
            end_time = time()
            time_elapsed = end_time - start_time
            #print(time_elapsed)



    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        #break
        return
    return x,y,z,theta, phi, psi

# Load in the calibration data
calib_data_path = "/Users/apple/Documents/MS /Summer Semester/research/AprilTags/ArUco/OpenCV-main/Distance Estimation/4. calib_data/MultiMatrix.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

MARKER_SIZE = 5  # centimeters (measure your printed marker size)

marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

param_markers = aruco.DetectorParameters()

# URLs for the ESP32 camera streams
robot1 = "192.168.4.2"

command_rotate_camera_up = '{"N": 106, "D1": 1}\n'
command_rotate_camera_down = '{"N": 106, "D1": 2}\n'
command_rotate_camera_left = '{"N": 106, "D1": 3}\n'
command_rotate_camera_right = '{"N": 106, "D1": 4}\n'

# Open the camera stream
cap = cv.VideoCapture('http://192.168.4.1:81/stream')

def command_to_robot(command, robot_name):
    host = robot_name
    port = 80
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    print(f'Sent_Command: {command}')
    s.sendall(bytes(str(command), 'utf-8'))
    sleep(2)
    
    buffer = " "
    while True:
        received_message = s.recv(1024).decode('utf-8')
        if not received_message:
            break
        buffer += received_message
        if '\n' in buffer:
            print(f"Received from server: {received_message}")
            break

    
    '''
    try:
        data = s.recv(8)  # Buffer size of 1024 bytes
            #if not data:
            #    break  # Exit loop if no more data is received

            # Decode the received bytes to a string
        received_message = data.decode('utf-8')
        print(f"Received from server: {received_message}")    
    #sleep(5)
    except ConnectionResetError:
        print("Connection was reset by the server, attempting to reconnect")
        sleep(5)
    except Exception as e:
        print(f"An error occured: {e}")
    #finally:
         #if 'client_socket' in locals():
                #s.close()
                #print("Disconnected from server")
    '''

#    s.close()





def putText(x, y, z , theta, phi, psi,frame,bottom_right):
    cv.putText(
                frame,
                #f"x: {round(x, 1)} y: {round(y, 1)} z: {round(z, 1)} θ: {round(theta, 1)}",
                f"x: {round(x, 1)}  y: {round(z, 1)} θ: {round(phi, 1)}",# θ: {round(psi, 1)} θ: {round(theta, 1)}",
                #f"x: {x}  y: {round(z, 1)} θ: {round(theta, 1)}",
                bottom_right,
                cv.FONT_HERSHEY_PLAIN,
                1.0,
                (0, 0, 255),
                2,
                cv.LINE_AA,
            )

time_data = []
tan_angle_data = []
robot1_angle_data = []
distance_data = []
disp_data = []
start_time = time()
check_robot_disconnect_x = [0,1]
check_robot_disconnect_y = [0,1]
check_robot_disconnect_theta = [0,1]

#def navigate_robot(x1, y1, theta1, x2, y2, theta2):
def navigate_robot_to_target(target_number):
    target_reached = False
    while target_reached != True: 
    #and (check_robot_disconnect_x[-1] != check_robot_disconnect_x[-2] and check_robot_disconnect_y[-1] !=check_robot_disconnect_y[-2]) :
#receive_data_from_robot(robot1)

        l1 = len(x_test)
        x,y,z,theta_target,phi_target,psi_target =  read_frame()
        l2 = len(x_test)
        if l2 == l1:
            print("Robot location not updated")
            continue

        print(x)
        print(y)
        print(z)
        print(theta_target)
        print(phi_target)
        print(psi_target)
    
        x2 = x[target_number-1]
        print(x2)
        y2 = z[target_number-1]
        print(y2)
        theta2 = phi_target[target_number-1]
        print(theta2)
        x1 = x[-1]
        print(x1)
        y1 = z[-1]
        print(y1)
        theta1 = phi_target[-1]
        print(theta1)
        check_robot_disconnect_x.append(x1)
        check_robot_disconnect_y.append(y1)
        check_robot_disconnect_theta.append(phi_target)
    

        #if x2==0 and x1==0:
        #    continue
        #elif check_robot_disconnect_x[-1] == check_robot_disconnect_x[-2]:
        #    continue
        #elif check_robot_disconnect_y[-1] == check_robot_disconnect_y[-2]:
        #    continue
        #elif check_robot_disconnect_theta[-1] == check_robot_disconnect_theta[-2]:
        #    continue
            
        #if check_robot_disconnect_x[-1] == check_robot_disconnect_x[-2] and check_robot_disconnect_y[-1] ==check_robot_disconnect_y[-2]:
        #    print("Sending break command")
        #    break
     #if (x2,y2,theta2,x1,y1,theta) != (0,0,0,0,0,0):

    #while True:
   # i=1
   # while(i>0):
        # Calculate distance to the target
        d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        y_error = y2-y1
        x_error = x2-x1

        # Calculate desired heading angle to the target
        angle_rad = math.atan2(y2 - y1, x2 - x1)
        angle_deg = math.degrees(angle_rad) 
        print(f"ATan_Angle : {angle_deg}")

        # Calculate angular error
        #delta_theta = theta_d - theta1
        delta_theta = angle_deg - theta1
        if delta_theta < -360:
           delta_theta = delta_theta + 360

    
        time_data.append(time()- start_time)
        tan_angle_data.append(angle_deg)
        robot1_angle_data.append(theta1)
        dist = round(d,1)
        disp_data.append(y1)
        disp = disp_data[-1] - disp_data[0]
        distance_data.append(disp)

        #print(time_data)
        #print(angle_data)

       # k_rpm_rot = 0.5*delta_theta
        k_rpm_rot = 40
        k_rpm_dis = d

        #k_rpm = math.sqrt(abs(delta_theta))
        rpm_rot = min(max(int(abs(k_rpm_rot)), 50), 190)  # Clamp rpm between 0 and 255
        rpm_dis = min(max(int(abs(k_rpm_dis)), 50), 190)  # Clamp rpm between 0 and 255

        command_robot_move_forward = f'{{"N": 3, "D1": 3, "D2": {rpm_dis}}}\n'
        command_robot_move_back = f'{{"N": 3, "D1": 4, "D2": {rpm_dis}}}\n'
        command_robot_move_left = f'{{"N": 3, "D1": 1, "D2": {rpm_rot}}}\n'
        command_robot_move_right = f'{{"N": 3, "D1": 2, "D2": {rpm_rot}}}\n'

       
        


        # Print the updated state of robot2
        print(f"Position of robot 1: ({x1:.2f}, {y1:.2f}), Orientation: {theta1:.2f}, Distance: {d:.2f}, Angle Error: {delta_theta:.2f}")


        # Print the updated state of target1 
        print(f"Position of target {target_number},: ({x2:.2f}, {y2:.2f}), Orientation: {theta2:.2f}, Distance: {d:.2f}, Angle Error: {delta_theta:.2f}")
        #return



        
        if abs(delta_theta) >30 or abs(delta_theta) > 330:
            if delta_theta < 0 and delta_theta > -180:
                #command = f'{{"N": 3, "D1": 1, "D2": {rpm}}}\n'
                command_to_robot(command_robot_move_right, robot1)
                #command_to_robot(command, robot1)
               
                print("Sending Command to Robot1 to move right")
                #break
                continue
            elif delta_theta > -180:
                command_to_robot(command_robot_move_left, robot1)
                print("Sending Command to Robot1 to move left")
                #break
                continue

            else:
                command_to_robot(command_robot_move_left, robot1)
                print("Sending Command to Robot1 to move left")
                #break
                continue
        
            
        elif abs(y_error) > 20:
       # elif d > 20:   
                command_to_robot(command_robot_move_forward, robot1)
                print("Sending Command to Robot1 to move forward")
                #break
                continue
         
    
    
        else:
            print("Target Reached:", target_number)
            send_command('RESET')
            print("Target1_Reset_Command_Sent")
            df = pd.DataFrame(tan_angle_data, columns = ['Numbers'])
            df.to_excel('tan_angle_data.xlsx', index = False)
            df = pd.DataFrame(robot1_angle_data, columns = ['Numbers'])
            df.to_excel('robot1_angle_data.xlsx', index = False)
            df = pd.DataFrame(distance_data, columns = ['Numbers'])
            df.to_excel('distance.xlsx', index = False)
            print("List saved to Excel File successfully.")
            target_reached = True
            #print("Exiting program...")
            #sys.exit()
            return
            #break
        
     #else:
        #break
    #return

            
 
        

navigate_robot_to_target(3)
navigate_robot_to_target(4)
navigate_robot_to_target(5)
#while True:


#cap.release()
#cv.destroyAllWindows()


