import cv2
import numpy as np
from MainSystem import AUVController

controller = AUVController('udpin:0.0.0.0:14550')
print("arm start")
controller.arm_vehicle()
print("arm success")

seen = 0
was_big = 0

#!/usr/bin/env python
"""
BlueRov video capture class
"""

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
        return (x, y, w, h)

    # Kırmızı alan bulunamazsa None döndür
    return None

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

def image_callback(cv_image: cv2.Mat, controller : AUVController):
    global was_big, seen
    show_image(cv_image)
    detection_res = detect_red_color(cv_image)
    if(detection_res==None):
        if(seen==1):

            seen = 2
        if(seen==2):
            print("ileri")
            controller.go_to_vehicle(1, 0, 0, 0)
        else:
            print("durdu")
            controller.go_to_vehicle(0,0,0,0)
        print("No flag")
    elif((detection_res[2]*detection_res[3]>5000 or was_big ==1) and seen != 2):
        print("aşağı")
        controller.go_to_vehicle(0,0,-1,0)

        was_big = 1
    else:
        if(seen==0):

            seen = 1
        res = kordinat_to_yazi(detection_res, cv_image.shape[1],cv_image.shape[0])
        print(res)
        print("sağ")
        controller.go_to_vehicle(0,-1,0,0)
        if(res[0][0]=="right"):
            print("sağ")
            controller.go_to_vehicle(0,1,0,0)
        elif(res[0][0]=="left"):
            print("sol")
            controller.go_to_vehicle(0,1,0,0)
            print("moved")
        elif(res[0][1]=="bottom" or res[0][1]=="middle"):
            print("aşağı")
            controller.go_to_vehicle(0,0,-1,0)
        else :
            print("durdu")
            controller.go_to_vehicle(0,0,0,0)
        if(res[0][0]=="middle" and res[0][1]=="top"):
            print("ileri")
            controller.go_to_vehicle(1,0,0,0)




class Video():
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
        latest_frame (np.ndarray): Latest retrieved video frame
    """

    def __init__(self, port=5600):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self.latest_frame = self._new_frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps_structure = sample.get_caps().get_structure(0)
        array = np.ndarray(
            (
                caps_structure.get_value('height'),
                caps_structure.get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        """ Get Frame

        Returns:
            np.ndarray: latest retrieved image frame
        """
        if self.frame_available:
            self.latest_frame = self._new_frame
            # reset to indicate latest frame has been 'consumed'
            self._new_frame = None
        return self.latest_frame

    def frame_available(self):
        """Check if a new frame is available

        Returns:
            bool: true if a new frame is available
        """
        return self._new_frame is not None

    def run(self):
        """ Get frame to update _new_frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        self._new_frame = self.gst_to_opencv(sample)

        return Gst.FlowReturn.OK


if __name__ == '__main__':
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

