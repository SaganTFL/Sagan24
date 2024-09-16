#!/usr/bin/env python2.7
# encoding=utf-8
# license removed for brevity

import cv2
import numpy as np
from MainSystem import AUVController


seen = 0
was_big = 0



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
        print(cv2.contourArea(contours))
        if(cv2.contourArea(contours)<100):
            return None

        x, y, w, h = cv2.boundingRect(largest_contour)

        return (x, y, w, h)

    # Kırmızı alan bulunamazsa None döndür
    return None

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

def image_callback(cv_image: cv2.Mat, controller : AUVController):
    print("call")
    global seen, was_big
    show_image(cv_image)
    detection_res = detect_red_color(cv_image)
    if(detection_res==None):
        if(seen==1):
            seen = 2
        if(seen==2):
            controller.go_to_vehicle(1, 0, 0, 0)
        else:
            controller.go_to_vehicle(0,0,0,0.1)
        print("No flag")
    elif((detection_res[2]*detection_res[3]>10000 or was_big ==1) and seen != 2):
        controller.go_to_vehicle(0,0,-1,0)
        was_big = 1
    else:
        if(seen==0 and detection_res[2]*detection_res[3]>1000):
            seen = 1
        res = kordinat_to_yazi(detection_res, cv_image.shape[1],cv_image.shape[0])
        print(res)
        controller.go_to_vehicle(0,1,0,0)
        if(res[0][0]=="right"):

            controller.go_to_vehicle(0,1,0,0)
        elif(res[0][0]=="left"):
            controller.go_to_vehicle(0,-1,0,0)
            print("moved")
        elif(res[0][1]=="bottom" or res[0][1]=="middle"):
            controller.go_to_vehicle(0,0,-1,0)
        else :
            controller.go_to_vehicle(0,0,0,0)
        if(res[0][0]=="middle" and res[0][1]=="top"):
            controller.go_to_vehicle(1,0,0,0)

def autonomous():
    controller = AUVController('tcp:localhost:5762')
    controller.arm_vehicle()
    while(1):
        #Get image from endpoint dont care rn
        image = controller.getImage()
        image_callback(image, controller)



import cv2
import vlc
import numpy as np
import time
import threading

# Global değişken: OpenCV'nin işlem yapacağı video karesi
frame = None

def vlc_stream(rtsp_url):
    global frame
    # VLC instance oluştur
    instance = vlc.Instance()
    player = instance.media_player_new()
    media = instance.media_new(rtsp_url)
    player.set_media(media)

    # Video akışını başlat
    player.play()

    # Akışın gelmesini beklemek için bir süre bekleyin
    time.sleep(2)  # 2 saniye bekle

    while True:
        # VLC oynatma sırasında kareleri elde etmek için snapshot alıyoruz
        player.video_take_snapshot(0, 'frame.jpg', 0, 0)  # Görüntüyü bir dosyaya kaydet

        # OpenCV ile dosyadan kareyi okuyoruz
        img = cv2.imread('frame.jpg')

        if img is not None:
            frame = img  # OpenCV'nin işlemesi için global frame değişkenine atıyoruz

        time.sleep(0.03)  # 30ms bekle, yaklaşık 30 FPS (frame per second) için

def opencv_process():
    global frame
    controller = AUVController('udpin:0.0.0.0:14550')
    controller.arm_vehicle()
    controller.set_mode("STABILIZE")
    while True:
        if frame is not None:
            # OpenCV ile görüntüyü işleyin (görüntü işlemi ekleyebilirsiniz)
            cv2.imshow('RTSP Stream', frame)
            #Get image from endpoint dont care rn
            image_callback(frame, controller)

            # 'q' tuşuna basıldığında döngüyü kır
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    # RTSP URL'si
    rtsp_url = 'rtsp://192.168.2.2:8554/video_stream__dev_video2'

    # VLC akışını başlatan thread
    vlc_thread = threading.Thread(target=vlc_stream, args=(rtsp_url,))
    vlc_thread.start()

    # OpenCV işlemesini başlatan ana thread
    opencv_process()
