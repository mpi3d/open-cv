from random import randint
from time import sleep

from Adafruit_SSD1306 import SSD1306_128_64
from cv2 import (WINDOW_GUI_NORMAL, WINDOW_KEEPRATIO, WINDOW_NORMAL,
                 CascadeClassifier, VideoCapture, destroyAllWindows, flip,
                 imshow, line, moveWindow, namedWindow, rectangle, waitKey)
from PIL.Image import new
from PIL.ImageDraw import Draw

from robotic_arm import Arm

parms = ({
    "Port": 0,
    "Adjust": 5,
    "Min": 130,
    "Max": 475,
    "Reverse": True
}, {
    "Left Port": 1,
    "Right Port": 2,
    "Left Adjust": 20,
    "Min": 130,
    "Max": 475,
    "Reverse": True
}, {
    "Port": 3,
    "Adjust": -5,
    "Min": 150,
    "Max": 500,
    "Reverse": True
}, {
    "Port": 4,
    "Adjust": -5,
    "Min": 125,
    "Max": 540,
    "Reverse": True
}, {
    "Port": 5,
    "Min": 280,
    "Max": 450
})

UP = [81, 119]
DOWN = [83, 115]
RIGHT = [82, 100]
LEFT = [84, 97]

DEG = 10
NMOVE = 15
RCHANCE = 20
WCHANCE = 30
NDISABLE = 50
EYEADJUST = 5
sentences = [
    "Hello world!", "Happy to meet you!", "I don't know what to say",
    "Life is good!"
]

NDEG = DEG * -1
MOVE = 0
DISABLE = NDISABLE + 1
SENTENCES_LEN = len(sentences) - 1
KEY = -1
TEXT = ""

face_cascade = CascadeClassifier("cascades/face.xml")
eye_cascade = CascadeClassifier("cascades/eye.xml")

disp = SSD1306_128_64(None)
disp.begin()
disp.clear()
disp.set_contrast(1)
disp.display()

width = disp.width
height = disp.height

textx = width - 15

arm = Arm(parms, (90, 90, 90, 0, 90))
sleep(0.1)
arm.activate(False)

cam = VideoCapture(0)

namedWindow("Camera",
            flags=WINDOW_NORMAL | WINDOW_KEEPRATIO | WINDOW_GUI_NORMAL)
moveWindow("Camera", 20, 20)

try:
    while True:
        for i in range(0, 10):
            cam.grab()

        ret, img = cam.retrieve()
        if ret:
            img = flip(img, -1)

            faces = face_cascade.detectMultiScale(img, 1.1, 10)

            if not isinstance(faces, tuple):
                faces = faces.tolist()
                faces.sort(key=lambda val: val[2] * val[3], reverse=True)

                for x, y, w, h in faces:
                    face = img[y:y + h, x:x + w]
                    eyes = eye_cascade.detectMultiScale(face, 1.1, 10)[:2]
                    rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    for ex, ey, ew, eh in eyes:
                        rectangle(face, (ex, ey), (ex + ew, ey + eh),
                                  (255, 0, 0), 2)

                x, y, w, h = faces[0]
                hw = int(round(w / 2))
                hh = int(round(h / 2))
                qw = int(round(hw / 2))
                qh = int(round(hh / 2))

                line(img, (x, y), (x, y - hh), (0, 255, 255), 2)
                line(img, (x, y - hh), (x + qw, y - qh), (0, 255, 255), 2)
                line(img, (x + qw, y - qh), (x + hw, y - hh), (0, 255, 255), 2)
                line(img, (x + hw, y - hh), (x + 3 * qw, y - qh), (0, 255, 255),
                     2)
                line(img, (x + 3 * qw, y - qh), (x + w, y - hh), (0, 255, 255),
                     2)
                line(img, (x + w, y - hh), (x + w, y), (0, 255, 255), 2)

                xw = x + hw
                yh = y + hh
                mx = int(round(img.shape[1] / 2))
                my = int(round(img.shape[0] / 2))
                dx = (mx - xw) * 65 / mx
                dy = (my - yh) * 65 / mx
                MOVE += 1
                DISABLE = 0

                if dx > DEG or NDEG > dx:
                    arm.base.set(arm.base.get() - dx)
                    MOVE = 0
                    ETY = 0
                else:
                    ETY = dy / EYEADJUST * -1

                if dy > DEG or NDEG > dy:
                    pos = arm.elbow.get() + arm.wrist.get() + dy

                    arm.shoulder.set(90)
                    if pos < 90:
                        arm.elbow.set(pos)
                        arm.wrist.set(0)
                    else:
                        arm.elbow.set(90)
                        arm.wrist.set(pos - 90)

                    MOVE = 0
                    ETX = 0
                else:
                    ETX = dx / EYEADJUST

                if MOVE == 0:
                    line(img, (xw, yh), (mx, my), (255, 255, 255), 2)
                    if randint(0, WCHANCE) == 0:
                        arm.wrench.set(0)
                        arm.wrench.set(90)

                if TEXT == "" and randint(0, RCHANCE) == 0:
                    TEXT = sentences[randint(0, SENTENCES_LEN)]
            else:

                if KEY in LEFT:
                    MOVE = 0
                    DISABLE = 0
                    arm.base.set(arm.base.get() - 10)
                elif KEY in UP:
                    MOVE = 0
                    DISABLE = 0
                    pos = arm.elbow.get() + arm.wrist.get() + 10

                    arm.shoulder.set(90)
                    if pos < 90:
                        arm.elbow.set(pos)
                        arm.wrist.set(0)
                    else:
                        arm.elbow.set(90)
                        arm.wrist.set(pos - 90)
                elif KEY in RIGHT:
                    MOVE = 0
                    DISABLE = 0
                    arm.base.set(arm.base.get() + 10)
                elif KEY in DOWN:
                    MOVE = 0
                    DISABLE = 0
                    pos = arm.elbow.get() + arm.wrist.get() - 10

                    arm.shoulder.set(90)
                    if pos < 90:
                        arm.elbow.set(pos)
                        arm.wrist.set(0)
                    else:
                        arm.elbow.set(90)
                        arm.wrist.set(pos - 90)
                elif KEY == 32:
                    DISABLE = 0
                    MOVE = NMOVE
                elif KEY == 13:
                    arm.wrench.set(0)
                    arm.wrench.set(90)
                else:
                    DISABLE += 1

                ETX = 0
                ETY = 0

            imshow("Camera", img)
        else:
            DISABLE = NDISABLE

        if DISABLE > NDISABLE:
            MOVE = 0
            sleep(0.5)
        elif DISABLE < NDISABLE:
            screen = new("1", (width, height))
            draw = Draw(screen)
            draw.rectangle(((15, 0), (50, 40)), 0, 1, 2)
            draw.rectangle(((width - 50, 0), (width - 15, 40)), 0, 1, 2)
            ETY += 18
            draw.rectangle(((ETX + 31, ETY), (ETX + 34, ETY + 3)), 1)
            draw.rectangle(
                ((ETX + width - 31, ETY), (ETX + width - 34, ETY + 3)), 1)

            if MOVE > NMOVE:
                draw.polygon(((15, 0), (50, 0), (50, 15), (15, 15)), 1)
                draw.polygon(((width - 50, 0), (width - 15, 0),
                              (width - 15, 15), (width - 50, 15)), 1)
            elif MOVE < NMOVE:
                draw.polygon(((15, 0), (50, 0), (50, 20), (15, 5)), 1)
                draw.polygon(((width - 50, 0), (width - 15, 0), (width - 15, 5),
                              (width - 50, 20)), 1)
            else:
                arm.activate(False)
                MOVE += 1
                draw.polygon(((15, 0), (50, 0), (50, 18), (15, 10)), 1)
                draw.polygon(((width - 50, 0), (width - 15, 0),
                              (width - 15, 10), (width - 50, 18)), 1)

            draw.rectangle(((15, height - 15), (width - 15, height - 1)), 0, 1,
                           2)

            if TEXT != "":
                if len(TEXT) * 6 + textx - 15 <= 0:
                    TEXT = ""
                    textx = width - 15
                else:
                    draw.text((textx, height - 14), TEXT, 1)
                    draw.rectangle(((0, height - 15), (14, height - 1)), 0)
                    draw.rectangle(
                        ((width, height - 15), (width - 14, height - 1)), 0)
                    textx -= 10

            disp.image(screen)
            disp.display()
            sleep(0.1)
        else:
            arm.set((90, 90, 90, 0, 90))
            sleep(0.1)
            arm.activate(False)
            disp.clear()
            disp.display()
            DISABLE += 1

        KEY = waitKey(100)

        if arm.base.is_activate():
            arm.base.activate(False)
        if arm.shoulder.is_activate():
            arm.shoulder.activate(False)
        if arm.wrench.is_activate():
            arm.wrench.activate(False)

except KeyboardInterrupt:
    destroyAllWindows()

    cam.release()

    arm.set((90, 90, 90, 0, 90))
    sleep(0.1)
    arm.activate(False)

    disp.clear()
    disp.display()
