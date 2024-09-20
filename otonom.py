import cv2
import numpy as np
from MainSystem import AUVController
#
controller = AUVController('udpin:0.0.0.0:14550')
print("Arm Start")
controller.arm_vehicle()
print("Arm Success")
controller.set_mode("STABILIZE")
def callback(x):
    pass

cv2.namedWindow("image")

lower_yellow = np.array([35,175,125])
lower_yellow = np.array([35,175,120])

upper_yellow = np.array([60,235,150])
upper_yellow = np.array([65,235,150])

ilowH = lower_yellow[0]
ihighH = upper_yellow[0]

ilowS = lower_yellow[1]
ihighS = upper_yellow[1]
ilowV = lower_yellow[2]
ihighV = upper_yellow[2]

# create trackbars for color change
cv2.createTrackbar('lowH','image',ilowH,179,callback)
cv2.createTrackbar('highH','image',ihighH,179,callback)

cv2.createTrackbar('lowS','image',ilowS,255,callback)
cv2.createTrackbar('highS','image',ihighS,255,callback)

cv2.createTrackbar('lowV','image',ilowV,255,callback)
cv2.createTrackbar('highV','image',ihighV,255,callback)


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

    g = frame_height * (2.0/5)
    g2 = frame_height * (1.5/5)
    if center_y < g:
        vertical = "top"
    elif center_y > frame_height + g2:
        vertical = "bottom"
    else:
        vertical = "middle"

    areas.append((horizontal, vertical))

    return areas


def detect_yellow_color(frame):
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

    lower_yellow = np.array([35,175,125])
    upper_yellow = np.array([60,235,150])

    # lower_red = np.array([10,20,80]) #red
    # upper_red = np.array([50,135,115])
    lower_red = np.array([60,160,135]) #green
    upper_red = np.array([75,200,190])
    # lower_red = np.array([35,150,150]) #yellow
    # upper_red = np.array([52,195,206])
    # lower_red = np.array([80,200,100]) #blue
    # upper_red = np.array([95,220,120])


    # get trackbar positions
    lower_yellow[0] = cv2.getTrackbarPos('lowH', 'image')
    upper_yellow[0] = cv2.getTrackbarPos('highH', 'image')
    lower_yellow[1] = cv2.getTrackbarPos('lowS', 'image')
    upper_yellow[1] = cv2.getTrackbarPos('highS', 'image')
    lower_yellow[2] = cv2.getTrackbarPos('lowV', 'image')
    upper_yellow[2] = cv2.getTrackbarPos('highV', 'image')

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([lower_yellow[0], lower_yellow[1], lower_yellow[2]])
    upper_yellow = np.array([upper_yellow[0], upper_yellow[1], upper_yellow[2]])
    mask = cv2.inRange(hsv, lower_red, upper_red)


    # mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # Maskeyi kullanarak kırmızı alanları bul
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    cv2.drawContours(frame,contours,-1,(0,255,0),3)
    cv2.imshow('cam', frame)

    # En büyük kırmızı alanı bulma
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        if(w*h>1000):

            return (x, y, w, h)
        return None

    # Kırmızı alan bulunamazsa None döndür
    return None


def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)


def image_callback(cv_image: cv2.Mat, controller):
    global big, seen

    detection_res = detect_yellow_color(cv_image)
    if(detection_res):
        print(detection_res[2] * detection_res[3])
    if detection_res and detection_res[2] * detection_res[3] > 15000: # if the color area too big it is big
        big = 1
        print("Big flag true")
    elif not detection_res and big: # we already passed through the door, so we now not see the color but it it already seen, so make seen flag true
        big = 0
        seen = 1
        print("Görüldü, seen flag true")

    # main movement

    if not detection_res and not seen:
        print("Bulunamadı - Araç dönüyor")
        controller.go_to_vehicle(1,0,0.05,-0.2)

    elif big:
        print("Aşağı")
        controller.go_to_vehicle(0.1,0,-0.1,0)

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
                print("sag capraz")
                controller.go_to_vehicle(0.3,0.5,0,0)
            else:
                print("sag")
                controller.go_to_vehicle(0.2,0,0,0.1)

        elif res[0][0] == "left":
            print("Sol")

            if usey:
                print("sol capraz")
                controller.go_to_vehicle(0.3,-0.3,0,0)
            else:
                print("sol")
                controller.go_to_vehicle(0.2,0,0,-0.1)

        elif res[0][1] == "bottom" :
            print("Aşağı")
            controller.go_to_vehicle(0,0,-1,0)
        elif res[0][1]=="top":
            print("Yukarı")
            controller.go_to_vehicle(0,0,1,0)
        else:
            print("İleri")
            controller.go_to_vehicle(1,0,0,0)

if __name__ == '__main__': # erkaaammmmmmmm #

    stream_url = "rtsp://192.168.2.2:8554/video_stream__dev_video2"

    cap = cv2.VideoCapture(stream_url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("cam not connected.")
    else:
        while True:
            ret, frame = cap.read()
            cv2.flip(frame,1)
            if not ret:
                print("not captured")
                break

            image_callback(frame,controller)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
