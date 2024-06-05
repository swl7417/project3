from transformers import pipeline
from PIL import Image, ImageDraw
from dbr import BarcodeReader
import numpy as np
import cv2
import torch
import matplotlib.pyplot as plt
import sys
import numpy as np
import struct
import socket
from sklearn.linear_model import LinearRegression
sys.path.append("..")
from segment_anything import sam_model_registry, SamPredictor
import time
import math
import serial
import threading
from PyQt5.QtWidgets import QApplication, QWidget, QTextEdit, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QMessageBox, QFrame
from PyQt5.QtCore import QTimer, QDateTime, Qt, QRectF
from PyQt5.QtGui import QFont, QPainter, QBrush

pipe = pipeline(task="depth-estimation", model="LiheYoung/depth-anything-small-hf") #small, base, large
sam_checkpoint = "C:/yolotest/sam_vit_b_01ec64.pth"
model_type = "vit_b"
device = "cuda"
sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
sam.to(device=device)
predictor = SamPredictor(sam)


reader = BarcodeReader()
reader.init_license("YOUR_LICENSE_KEY")


# ser2 = serial.Serial('COM8', 9600, timeout = 1)
# print(ser2)
read = 0

start = False  
A = False
B = False
Stop = False

def recognize_barcodes(image):
    barcode_results = reader.decode_buffer(image)
    barcode_text = ""
    if barcode_results is not None:
        for barcode_result in barcode_results:
            barcode_format = barcode_result.barcode_format_string
            barcode_text = barcode_result.barcode_text
            points = barcode_result.localization_result.localization_points

            print(f"Barcode Format: {barcode_format}, Barcode Text: {barcode_text}")

            try:
                barcode_text.encode('ascii')

            except UnicodeEncodeError:
                barcode_text = barcode_text.encode('ascii', 'replace').decode('ascii')
                print(f"Non-ASCII characters in barcode text replaced: {barcode_text}")

            for i in range(4):
                pt1 = (points[i][0], points[i][1])
                pt2 = (points[(i + 1) % 4][0], points[(i + 1) % 4][1])
                cv2.line(image, pt1, pt2, (0, 255, 0), 2)

            cv2.putText(image, f"{barcode_format}: {barcode_text}", (points[0][0], points[0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return image, barcode_text


def generate_masks(image, x, y):
    image = np.expand_dims(image, axis=2)
    image = np.repeat(image, 3, axis=2)
    predictor.set_image(image)
    input_point = np.array([[x, y]])
    input_label = np.array([1])
    masks, scores, logits = predictor.predict(
        point_coords=input_point,
        point_labels=input_label,
        multimask_output=True,
    )
    max_score_index = np.argmax(scores)
    return masks[max_score_index], input_point, input_label

def display_image_with_mask(image, mask, input_point, input_label):
    plt.figure(figsize=(10, 10))
    plt.imshow(image)
    show_mask(mask, plt.gca())
    show_points(input_point, input_label, plt.gca())
    plt.show()

def show_mask(mask, ax, random_color=False):
    if random_color:
        color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
    else:
        color = np.array([30/255, 144/255, 255/255, 0.6])
    h, w = mask.shape[-2:]
    mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    ax.imshow(mask_image)

def apply_mask(image, mask):
    masked_image = np.zeros_like(image)
    masked_image[mask] = image[mask]
    return masked_image

def show_points(coords, labels, ax, marker_size=375):
    pos_points = coords[labels==1]
    neg_points = coords[labels==0]
    ax.scatter(pos_points[:, 0], pos_points[:, 1], color='green', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)
    ax.scatter(neg_points[:, 0], neg_points[:, 1], color='red', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)

#for grayscale image
def find_target_fromGRAY(image, x,y):
    image = np.expand_dims(image, axis=2)
    image = np.repeat(image, 3, axis=2)
    predictor.set_image(image)
    input_point = np.array([[x,y]])
    input_label = np.array([1])
    masks, scores, logits = predictor.predict(
    point_coords=input_point,
    point_labels=input_label,
    multimask_output=True,
    )
    max_score_index = np.argmax(scores)
    plt.figure(figsize=(10, 10))
    plt.imshow(image)
    show_mask(masks[max_score_index], plt.gca())
    show_points(input_point, input_label, plt.gca())
    plt.show()
    return masks[max_score_index]

#for rgb image
def find_target_fromRGB(image, x,y):
    image=cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    predictor.set_image(image)
    input_point = np.array([[x,y]])
    input_label = np.array([1])
    masks, scores, logits = predictor.predict(
    point_coords=input_point,
    point_labels=input_label,
    multimask_output=True,
    )
    max_score_index = np.argmax(scores)
    plt.figure(figsize=(10, 10))
    plt.imshow(image)
    show_mask(masks[max_score_index], plt.gca())
    show_points(input_point, input_label, plt.gca())
    plt.show()
    return masks[max_score_index]

def find_brightest_point(image):
    max_index = np.unravel_index(np.argmax(image, axis=None), image.shape)
    bright_x, bright_y = max_index[1], max_index[0]
    print(bright_x, bright_y)
    return bright_x, bright_y

def draw_line(array, point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    
    if x2 - x1 == 0:
        slope = None
    else:
        slope = (y2 - y1) / (x2 - x1)
    
    for x in range(min(x1, x2), max(x1, x2) + 1):
        if slope is None:
            y = min(y1, y2) + (x - min(x1, x2))
        else:
            y = int(slope * (x - x1) + y1)
        array[y, x] = True
    
    return array

def line_length(boolean_array):
    # Find indices of True values along the line
    indices = np.argwhere(boolean_array)
    distances = np.linalg.norm(np.diff(indices, axis=0), axis=1)
    
    total_length = np.sum(distances)
    if total_length==0:
        return total_length, [0,0], [0,0]
    return total_length, indices[0], indices[-1]

def draw_min_area_rect(image, mask):
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_contour = max(contours, key=cv2.contourArea)

    rect = cv2.minAreaRect(max_contour)
    
    box = cv2.boxPoints(rect)
    box = np.intp(box)
    cv2.drawContours(image, [box], 0, (0, 255, 0), 2)
    
    if rect[1][0] < rect[1][1]:
        # shorter_edge_length=rect[1][0]
        center1 = (int((box[0][0] + box[1][0]) / 2), int((box[0][1] + box[1][1]) / 2))
        center2 = (int((box[2][0] + box[3][0]) / 2), int((box[2][1] + box[3][1]) / 2))
    else:
        # shorter_edge_length=rect[1][1]
        center1 = (int((box[1][0] + box[2][0]) / 2), int((box[1][1] + box[2][1]) / 2))
        center2 = (int((box[0][0] + box[3][0]) / 2), int((box[0][1] + box[3][1]) / 2))
    

    line_np = draw_line(np.zeros_like(mask),tuple(center1), tuple(center2))
    result_mask=line_np & mask
    short_length, line_start, line_end = line_length(result_mask)  #Real Shortest Distance

    cv2.line(image, (line_start[1],line_start[0]), (line_end[1],line_end[0]), (0, 0, 255), 2)

    line = np.array([center1, center2])
    direction_vector = line[1] - line[0]
    angle_with_y_axis = np.arctan2(direction_vector[0], direction_vector[1]) * 180 / np.pi
    angle_with_y_axis =  (angle_with_y_axis%180)-90
    short_length = short_length * 0.0328 * 9
    print("angle with y axis : ", angle_with_y_axis)
    if short_length > 100:
        short_length = 50
    return image, rect, angle_with_y_axis, short_length

def map_coordinate(x, y):
    cam_x, cam_y = 640, 480  # Assuming the camera resolution is 640x480

    # Original camera points and corresponding MoveIt coordinates
    camera_points = np.array([[365.5, 229.5], [39.0, 61.0], [53.624839782714844, 433.6141052246094], [587.0, 50.0], [587.0, 432.5],[361.8340759277344, 227.84954833984375]])
    moveit_coords = np.array([[-0.212599, -0.00892], [-0.287732, -0.145396], [-0.154044, -0.144546], [-0.271177, 0.0676568], [-0.155317, 0.0684298],[-0.22635356966948272, -0.016062925349326407]])

    # Normalize camera coordinates
    normalized_camera = camera_points / [cam_x, cam_y]

    # Fit linear regression models for x and y coordinates separately
    model_x = LinearRegression().fit(normalized_camera, moveit_coords[:, 0])
    model_y = LinearRegression().fit(normalized_camera, moveit_coords[:, 1])

    # Normalize the input coordinates
    normalized_x = x / cam_x
    normalized_y = y / cam_y

    # Predict MoveIt coordinates using the fitted models
    moveit_x = model_x.predict([[normalized_x, normalized_y]])
    moveit_y = model_y.predict([[normalized_x, normalized_y]])

    moveit_x -= 0.015 # 30
    moveit_y += 0.028 # 38

    return moveit_x[0], moveit_y[0]



class CommandInputWidget(QWidget):

    def __init__(self):
        super().__init__()

        self.client_socket = None

        self.setWindowTitle('process1')
        self.resize(290, 400)

        # Create a label for date and time
        self.datetime_label = QLabel(self)
        self.datetime_label.setGeometry(10, 0, 300, 40)  # (x, y, width, height)
        font = QFont()
        font.setPointSize(10)  # Set font size to 14
        self.datetime_label.setFont(font)
        self.update_datetime()  # Initial update of date and time

        # Create a label
        self.label = QLabel('waiting..', self)
        self.label.setGeometry(50, 280, 150, 80)  # (x, y, width, height)
        font = QFont()
        font.setPointSize(10)  # Set font size to 16
        self.label.setFont(font)

        # Create a frame
        self.frame = QFrame(self)
        self.frame.setGeometry(120, 50, 150, 200)
        self.frame.setFrameShape(QFrame.Panel)
        self.frame.setFrameShadow(QFrame.Raised)

        # Set up a vertical layout for the frame
        frame_layout = QVBoxLayout(self.frame)

        # Create a label for the frame
        self.frame_label = QLabel("COBOT", self.frame)
        self.frame_label.setAlignment(Qt.AlignCenter)
        frame_layout.addWidget(self.frame_label)

        # Create a frame
        self.frame2 = QFrame(self)
        self.frame2.setGeometry(20, 50, 100, 100)
        self.frame2.setFrameShape(QFrame.Panel)
        self.frame2.setFrameShadow(QFrame.Raised)

        # Set up a vertical layout for the frame
        frame2_layout = QVBoxLayout(self.frame2)

        # Create a label for the frame
        self.frame2_label = QLabel("Sorting\nB_BOX", self.frame2)
        font = QFont()
        font.setPointSize(7)  # Set font size to 14
        self.frame2_label.setFont(font)
        self.frame2_label.setAlignment(Qt.AlignCenter)
        frame2_layout.addWidget(self.frame2_label)

        # Create a frame
        self.frame3 = QFrame(self)
        self.frame3.setGeometry(20, 150, 100, 100)
        self.frame3.setFrameShape(QFrame.Panel)
        self.frame3.setFrameShadow(QFrame.Raised)

        # Set up a vertical layout for the frame
        frame3_layout = QVBoxLayout(self.frame3)

        # Create a label for the frame
        self.frame3_label = QLabel("Sorting\nA_BOX", self.frame3)
        font = QFont()
        font.setPointSize(7)  # Set font size to 14
        self.frame3_label.setFont(font)
        self.frame3_label.setAlignment(Qt.AlignCenter)
        frame3_layout.addWidget(self.frame3_label)

        button_1 = QPushButton('process\nstart', self)
        button_1.setGeometry(160, 270, 100, 50)
        button_1.clicked.connect(self.send_command_2)
        # layout.addWidget(button_3)
        button_2 = QPushButton('conveyer\nstart', self)
        button_2.setGeometry(160, 330, 100, 50)
        button_2.clicked.connect(self.send_command_1)

        # Set up a timer to update the date and time label every second
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_datetime)
        self.timer.start(1000)

        # self.setLayout(layout)
    def update_datetime(self):
        current_datetime = QDateTime.currentDateTime()
        self.datetime_label.setText(current_datetime.toString("yyyy-MM-dd hh:mm:ss"))

    def show_stop_message_box(self):
        self.label.setText("waiting..")
        self.frame2_label.setText("B_BOX")
        self.frame3_label.setText("A_BOX")
        msg_box = QMessageBox()
        msg_box.setWindowTitle("Stop Message")
        msg_box.setText("Stop Label")
        msg_box.setIcon(QMessageBox.Information)
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.exec_()

    def show_A_message_box(self):
        self.frame3_label.setText("A_BOX\nDone!")
        # self.frame5_label.setText("A_BOX\nDone!")
        msg_box = QMessageBox()
        msg_box.setWindowTitle("A Done")
        msg_box.setText("A Done")
        msg_box.setIcon(QMessageBox.Information)
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.exec_()
        

    def show_B_message_box(self):
        self.frame2_label.setText("B_BOX\nDone!")
        # self.frame4_label.setText("B_BOX\nDone!")
        msg_box = QMessageBox()
        msg_box.setWindowTitle("B Done")
        msg_box.setText("B Done")
        msg_box.setIcon(QMessageBox.Information)
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.exec_()

    def send_command_1(self):
        self.send_command(1)

    def send_command_2(self):
        global client_socket
        self.label.setText("Processing...")
        self.frame2_label.setText("B_BOX\nprocessing...")
        self.frame3_label.setText("A_BOX\nProcessing...")
        send_data_to_client(client_socket)

    def send_command(self, cmd_int):
        if cmd_int == 1:
            # ser2.write(b'1')
            print("conv start")

def send_data_to_client(client_socket):
    try:
        data = '1'
        client_socket.sendall(data.encode())
        print("data: ", data)
    except socket.error as e:
        print(f"Socket error: {e}")
    except struct.error as e:
        print(f"Struct packing error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    # finally:
    #     client_socket.close()

def gui_thread():
    # global A, B, Stop
    app = QApplication(sys.argv)
    command_input_widget = CommandInputWidget()
    command_input_widget.show()

    def check_flags():
        global A,B,Stop
        if A:
            command_input_widget.show_A_message_box()
            A = False
            print("A: ", A)
        if B:
            command_input_widget.show_B_message_box()
            B = False
            print("B: ", B)
        if Stop:
            command_input_widget.show_stop_message_box()
            Stop = False
            print("Stop: ", Stop)

    timer = QTimer()
    timer.timeout.connect(check_flags)
    timer.start(100)

    sys.exit(app.exec_())

def main():
    global start, A, B, Stop

    
    # calibration_matrix = np.load('calibration_matrix.npy')
    # distortion_coefficients = np.load('distortion_coefficients.npy')
    cap = cv2.VideoCapture(0)
    count=0
    if not cap.isOpened():
        print("Error: Couldn't open camera")
        return

    flag=1
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('172.30.1.24', 8080)
    server_socket.bind(server_address)
    server_socket.listen(1)
    print("Server is listening on port", server_address[1])

    global client_socket
    client_socket, client_address = server_socket.accept()

    while True:

        try:
            if flag!=0:
                data = client_socket.recv(1024)
                print("Received:", data.decode())
                

            if flag!=0 and data.decode() == "Start!":
                data = 0
                start = True
                flag = 0

            if start:
                data1 = client_socket.recv(1024)
                print("Received:", data1.decode())

                if data1.decode() == "Run!":
                    # Your existing code block for handling "Run!" command
                    data1 = 0
                    count+=1
                    print("Tiral :", count)
                    ret, frame = cap.read()
                    # cv2.imshow("rgb image", frame)
                    # frame = cv2.undistort(frame, calibration_matrix, distortion_coefficients)
                    print("")
                    print("Making depth...")
                    height, width, _ = frame.shape
                    #make ROI
                    # for y in range(height):
                    #     for x in range(width):
                    #         if (0<=y<height//4):
                    #             frame[y, x] = [0, 0, 0]   
                    start_time=time.time()
                    # numpy to pil
                    image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                    depth = pipe(image)["depth"]
                    depth_array = np.array(depth)
                    # cv2.imshow("depth image", depth_array)
                    
                    x,y = find_brightest_point(depth_array) #except box -> depth_array[100:600, 100:400]
                    # find_target_fromRGB(frame, x,y)
                    # masks=find_target_fromGRAY(depth_array, x, y)
                    masks,input_point, input_label = generate_masks(depth_array, x, y)

                    num_true= np.sum(masks)
                    if num_true>len(masks)*len(masks[0])/2 :
                        masks=np.logical_not(masks)
                    
                    # cutting_x, cutting_y = 30, 30
                    # masks[:cutting_x, :]= False
                    # masks[-cutting_x:, :]= False
                    # masks[:, :cutting_y]= False
                    # masks[:, -cutting_y:]= False
                    
                    masked_frame = apply_mask(frame, masks)

                    # display_image_with_mask(image, masks, input_point, input_label)
                    frame_with_rects, rect, rotated, dist= draw_min_area_rect(masked_frame.copy(), masks)
                    if(dist==0):
                        print("no object detected")

                    #Convert Transformation
                    cam_x, cam_y = rect[0][0], rect[0][1]
                    # print("cam_coordinate_center :", (cam_x, cam_y))
                    
                    moveit_x, moveit_y = map_coordinate(cam_x, cam_y)
                    # print("moveit_coordinate_center :", (moveit_x, moveit_y))
                    # print("angle", rotated)
                    # print("dist", dist )

                    frame_with_barcodes, barcode = recognize_barcodes(frame_with_rects)
                    
                    if barcode == "":
                        barcode = "None"

                    print("barcode: ",barcode)

                    end_time=time.time()
                    cv2.imshow('a',frame_with_barcodes)

                    # print('inference time :', end_time-start_time)  #average ~= 2.4

                    # Send response back to the client

                    if barcode == "None":
                        data2 = struct.pack('!ffff', moveit_x, moveit_y, rotated, dist)
                        print("Sending x,y,rz,dist:" , moveit_x, moveit_y, rotated, dist)
                        client_socket.sendall(data2)
                        print("Send success")

                        data3 = client_socket.recv(1024)
                        print("Received:", data3.decode())

                        if data3.decode() == "send barcode!":
                            data2 = 0
                            data3 = 0
                            barcode_data = str(barcode)
                            print("Sending barcode:", barcode_data)
                            client_socket.sendall(barcode_data.encode())
                            print("Send success")
                            barcode_data = 0
                        

                    elif not barcode == "None":
                        data2 = struct.pack('!ffff', moveit_x, moveit_y, rotated, dist)
                        print("Sending x,y,rz,dist:" , moveit_x, moveit_y, rotated, dist)
                        client_socket.sendall(data2) 
                        print("Send success")

                        data3 = client_socket.recv(1024)
                        print("Received:", data3.decode())

                        if data3.decode() == "send barcode!":
                            data2 = 0 
                            data3 = 0
                            barcode_data = str(barcode[34:])
                            print("Sending barcode:", barcode_data)
                            client_socket.sendall(barcode_data.encode())
                            print("Send success")
                            barcode_data = 0

                    cv2.imshow('a',frame_with_rects)
                    print("inference time :", end_time-start_time)
                    data=None
                    time.sleep(3)

                    pass

                elif data1.decode() == "Stop!":
                    data1 = 0
                    Stop = True
                    start = False
                    flag = 1
                    print("start: ", start)         
                    print("stop: ", Stop)

                elif data1.decode() == "A":
                    data1 =0
                    A = True

                elif data1.decode() == "B":
                    data1 = 0
                    B = True

        except socket.error as e:
            print(f"Socket error: {e}")
        except struct.error as e:
            print(f"Struct packing error: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

        
        # finally:
        #     client_socket.close()

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    threading.Thread(target=gui_thread).start()
    threading.Thread(target=main).start()