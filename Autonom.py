import cv2
import numpy as np
from video import Video
from MainSystem import AUVController

controller = AUVController('udpin:0.0.0.0:14550')
print("Arm Start")
controller.arm_vehicle()
print("Arm Success")

# Flags
big = 0
seen = 0

def kordinat_to_yazi(coordinates, frame_width, frame_height):
    if coordinates == [None]: return None

    areas = []
    x, y, w, h = coordinates

    center_x = x + w // 2
    center_y = y + h // 2
    k = frame_width * (2.0/5)

    if center_x < k:
        horizontal = "left"
    elif center_x > frame_width -k:
        horizontal = "right"
    else:
        horizontal = "middle"

    g = frame_height * (1.0/5)
    g2 = frame_height * (2.0/5)
    if center_y < g:
        vertical = "top"
    elif center_y > frame_height + g2:
        vertical = "bottom"
    else:
        vertical = "middle"

    areas.append((horizontal, vertical))

    return areas


def detect_red_color(frame):
    # HSV çevir
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Kırmızı renk belirle
    # lower_red1 = np.array([0, 120, 70])
    # upper_red1 = np.array([10, 255, 255])
    #
    # lower_yellow = np.array([255, 213, 0])
    # upper_yellow= np.array([234, 255, 0])
    #
    # mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    #
    # lower_red2 = np.array([170, 120, 70])
    # upper_red2 = np.array([180, 255, 255])
    # mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    #
    # # İki maskeyi birleştir
    # mask = mask1 + mask2

    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # Maskeyi kullanarak kırmızı alanları bul
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]

    # En büyük kırmızı alanı bulma
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        if(w*h>200):
            return (x, y, w, h)
        return None

    # Kırmızı alan bulunamazsa None döndür
    return None


def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)


def image_callback(cv_image: cv2.Mat, controller: AUVController):
    global big, seen

    show_image(cv_image)
    
    detection_res = detect_red_color(cv_image)

    if detection_res and detection_res[2] * detection_res[3] > 10000: # if the color area too big it is big
        big = 1
        print("Big flag true")
    elif not detection_res and big: # we already passed through the door, so we now not see the color but it it already seen, so make seen flag true
        big = 0
        seen = 1
        print("Görüldü, seen flag true")

    # main movement

    if not detection_res and not seen:
        print("Bulunamadı - Araç dönüyor")
        controller.go_to_vehicle(0,0,0,0.3) 
       
    elif big:
        print("Aşağı")
        controller.go_to_vehicle(0,0,-1,0)
  
    elif seen:
        print("İleri")
        controller.go_to_vehicle(1,0,0,0)

    else:
        res = kordinat_to_yazi(detection_res, cv_image.shape[1],cv_image.shape[0])
        print(res)

        usey = 0 # vey far away from object
        if detection_res[2] * detection_res[3] < 500:
            usey = 1


        if res[0][0] == "right":
            print("Sağ")
            if usey:
                controller.go_to_vehicle(0.3,-0.5,0,0)
            else:
                controller.go_to_vehicle(0,0,0,-0.5)

        elif res[0][0] == "left":
            print("Sol")
            if usey:
                controller.go_to_vehicle(0.3,0.5,0,0)
            else:
                controller.go_to_vehicle(0,0,0,0.5)
            
        elif res[0][1] == "bottom" or res[0][1] == "middle":
            print("Aşağı")
            controller.go_to_vehicle(0,0,-1,0)

        else:
            print("İleri")
            controller.go_to_vehicle(1,0,0,0)


if __name__ == '__main__': # erkaaammmmmmmm #
    # Create the video object
    # Add port= if is necessary to use a different one
    video = Video()

    print('Initialising stream...')
    waited = 0
    while not video.frame_available():
        waited += 1
        print('\r  Frame not available (x{})'.format(waited), end='')
        cv2.waitKey(30)
    print('\nSuccess!\nStarting streaming - press "q" to quit.')

    while True:
        # Wait for the next frame to become available
        if video.frame_available():
            # Only retrieve and display a frame if it's new
            frame = video.frame()
            cv2.imshow('frame', frame)
        # Allow frame to display, and check if user wants to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # cap = cv2.VideoCapture("test.mkv")
        # frameTime = 50
        #Get image from endpoint dont care rn

        # ret ,image = cap.read()
        #
        # if cv2.waitKey(frameTime) & 0xFF == ord('q'):
        #     break
        print("test")
        image = controller.get_image()
        print("test2")
        image_callback(image,controller)
        print("test3")
