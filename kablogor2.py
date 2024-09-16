import math
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


vic = cv2.VideoCapture("dnm6.mp4")


"""
hata ayıklarken debug değişkenini True yapıp kodu değiştirmeyi deneyin.
  |---> şekli erode yaptığımız için bulunan köşeler aslında oldukları yerden merkeze daha yakınlar
  |---> debug modeda videoyu devam ettirmek için üzerinde herangi bir tuşa basın
mxlosframe
  |---> kayıp frame toleransı
  |---> daha iyi bir debug deneyimi için 0 yapın !!!!!
"""
"""
yenilikler:
    türkçe karakter desteği
    trombüs algılama kodu iyileştirmesi
"""
debug = False
mxlosframe = 2
fontdosya_ismi = "times.ttf"
renk = (0, 0, 0)  # yazı rengi
background_color = (255, 255, 255)  # arkaplan rengi (olmaması için None yapın)


sekil = [0] * 10  # [daire, elips, yonca, ucgen, kare, trombüs, dikdörtgen, beşgen, altıgen, yıldız]
losed = mxlosframe
while True:
    k, kare = vic.read()
    if not k:
        break
    kare = cv2.resize(kare, [kare.shape[1] // 2, kare.shape[0] // 2])  # !!!!!!!! sabit sayı koymayın görüntüyü bozuyor
    # illa resize yapacaksanız görüntüyü işledikten sonra yapın
    sari = cv2.inRange(kare, (0, 100, 100), (70, 255, 255))
    sari = cv2.morphologyEx(sari, cv2.MORPH_CLOSE, np.ones([7, 7]))
    sari = cv2.morphologyEx(sari, cv2.MORPH_OPEN, np.ones([5, 5]))
    if debug:
        cv2.imshow("ana gor", kare)
        cv2.imshow("sari", sari)
    if mxlosframe <= losed:
        sekil = [0] * 10
    contours, hierarchy = cv2.findContours(sari, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    if len(contours) > 1:
        sekil = [0] * 10
    if contours and len(contours[0]) > 50:
        losed = 0
        rect = cv2.boundingRect(contours[0])
        ############# köşeleri sivriltmek için kısım(biraz fazla cpu kullanıyor olabilir)  ################
        for i in range(5):
            sari = cv2.erode(sari, np.ones((3, 3)))
        for i in range(5):
            sari = cv2.dilate(sari, np.ones((3, 3)))
        for i in range(5):
            sari = cv2.erode(sari, np.ones((3, 3)))
        #######################################
        if debug:
            cv2.imshow("sari2", sari)
        contourss, hierarchy = cv2.findContours(sari, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if contourss:
            contours = contourss[0]
            center: list[int] = [rect[0] + rect[2] // 2, rect[1] + rect[3] // 2]
            if debug:
                cv2.circle(kare, center, 10, (0, 255, 0), 10)
            nokset: set[float] = {10000000000}
            nokset.pop()
            ilerigeri = 30
            noktain: list[list[int]] = []
            noktaout: list[list[int]] = []
            for o in range(-ilerigeri, ilerigeri):
                ii = (o + len(contours)) % len(contours)
                nokset.add(math.hypot(contours[ii][0][0] - center[0], contours[ii][0][1] - center[1]))
            for i in range(len(contours)):
                """
                cv2.goodFeaturesToTrack() => bence çok iyi çalışmıyor
                """
                ii = (i + ilerigeri + len(contours)) % len(contours)
                nokset.add(math.hypot(contours[ii][0][0] - center[0], contours[ii][0][1] - center[1]))
                u = math.hypot(contours[i][0][0] - center[0], contours[i][0][1] - center[1])
                if u >= max(nokset):
                    if len(noktaout) == 0 or (math.hypot(noktaout[len(noktaout) - 1][0] - contours[i][0][0],
                                                        noktaout[len(noktaout) - 1][1] - contours[i][0][1]) > 40 and
                                              math.hypot(noktaout[0][0] - contours[i][0][0],
                                                         noktaout[0][1] - contours[i][0][1]) > 40):
                        noktaout.append([contours[i][0][0], contours[i][0][1]])
                if u <= min(nokset):
                    if len(noktain) == 0 or math.hypot(noktain[len(noktain) - 1][0] - contours[i][0][0],
                                                       noktain[len(noktain) - 1][1] - contours[i][0][1]) > 40:
                        noktain.append([contours[i][0][0], contours[i][0][1]])
                ii = (i - ilerigeri + len(contours)) % len(contours)
                if nokset.issuperset({math.hypot(contours[ii][0][0] - center[0], contours[ii][0][1] - center[1])}):
                    nokset.remove(math.hypot(contours[ii][0][0] - center[0], contours[ii][0][1] - center[1]))
            if debug:
                print(noktaout, noktain, center)
                if noktain:
                    for i in noktain:
                        cv2.circle(kare, [int(i[0]), int(i[1])], 10, (255, 0, 0), 10)
                if noktaout:
                    for i in noktaout:
                        cv2.circle(kare, [int(i[0]), int(i[1])], 10, (0, 0, 255), 10)
            cv2.rectangle(kare, rect, (0, 0, 255), 10)
            if kare.shape[0] - 5 > rect[1] + rect[3] and rect[1] > 50 and\
               kare.shape[1] - 5 > rect[0] + rect[2] and rect[0] > 5:  # ekranın kenarına değiyorsa geç
                m = max([math.hypot(p[0] - center[0], p[1] - center[1]) for p in noktaout])
                n = min([math.hypot(p[0] - center[0], p[1] - center[1]) for p in noktain])
                if m - n < rect[2] / 15:
                    # dairenin köşesi veya merkeze uzak belli bir köşesi olmadığı için ilk buna bakılır
                    # daire
                    sekil[0] += 1
                elif len(noktaout) == 3:
                    if len(noktain) > 3:
                        # yonca
                        sekil[2] += 1
                    else:
                        # ucgen
                        sekil[3] += 1
                elif len(noktaout) == 5:
                    a = sari[(noktaout[0][1] * 9 + noktaout[1][1]) // 10, (noktaout[0][0] * 9 + noktaout[1][0]) // 10]
                    b = sari[((noktaout[0][1] + noktaout[1][1]) * 15 + center[1]) // 31, ((noktaout[0][0] + noktaout[1][0]) * 15 + center[0]) // 31]
                    b2 = sari[((noktaout[2][1] + noktaout[1][1]) * 15 + center[1]) // 31, ((noktaout[2][0] + noktaout[1][0]) * 15 + center[0]) // 31]
                    b3 = sari[((noktaout[2][1] + noktaout[3][1]) * 15 + center[1]) // 31, ((noktaout[2][0] + noktaout[3][0]) * 15 + center[0]) // 31]
                    b4 = sari[((noktaout[4][1] + noktaout[3][1]) * 15 + center[1]) // 31, ((noktaout[4][0] + noktaout[3][0]) * 15 + center[0]) // 31]
                    b5 = sari[((noktaout[4][1] + noktaout[0][1]) * 15 + center[1]) // 31, ((noktaout[4][0] + noktaout[0][0]) * 15 + center[0]) // 31]
                    if b and b2 and b3 and b4 and b5:
                        # besgen
                        sekil[7] += 1
                    elif a:
                        # yonca
                        sekil[2] += 1
                    else:
                        # yildiz
                        sekil[9] += 1
                elif len(noktaout) == 4:
                    n1 = [(noktaout[0][0] + noktaout[1][0]) // 2, (noktaout[0][1] + noktaout[1][1]) // 2]
                    n2 = [(noktaout[2][0] + noktaout[1][0]) // 2, (noktaout[2][1] + noktaout[1][1]) // 2]
                    n3 = [(noktaout[2][0] + noktaout[3][0]) // 2, (noktaout[2][1] + noktaout[3][1]) // 2]
                    n4 = [(noktaout[0][0] + noktaout[3][0]) // 2, (noktaout[0][1] + noktaout[3][1]) // 2]
                    a = True
                    has = -.03
                    has2 = has + 1
                    x = int(((noktaout[0][0] + noktaout[1][0]) / 2) * has2 - center[0] * has)
                    y = int(((noktaout[0][1] + noktaout[1][1]) / 2) * has2 - center[1] * has)
                    if 0 <= x < sari.shape[1] and 0 <= y < sari.shape[0]:
                        a = sari[y, x]
                        if debug:
                            cv2.circle(kare, [x, y], 10, (255, 255, 0), 10)
                    x = int(((noktaout[1][0] + noktaout[2][0]) / 2) * has2 - center[0] * has)
                    y = int(((noktaout[1][1] + noktaout[2][1]) / 2) * has2 - center[1] * has)
                    if 0 <= x < sari.shape[1] and 0 <= y < sari.shape[0]:
                        a = a and sari[y, x]
                        if debug:
                            cv2.circle(kare, [x, y], 10, (255, 255, 0), 10)
                    x = int(((noktaout[3][0] + noktaout[2][0]) / 2) * has2 - center[0] * has)
                    y = int(((noktaout[3][1] + noktaout[2][1]) / 2) * has2 - center[1] * has)
                    if 0 <= x < sari.shape[1] and 0 <= y < sari.shape[0]:
                        a = a and sari[y, x]
                        if debug:
                            cv2.circle(kare, [x, y], 10, (255, 255, 0), 10)
                    x = int(((noktaout[3][0] + noktaout[0][0]) / 2) * has2 - center[0] * has)
                    y = int(((noktaout[3][1] + noktaout[0][1]) / 2) * has2 - center[1] * has)
                    if 0 <= x < sari.shape[1] and 0 <= y < sari.shape[0]:
                        a = a and sari[y, x]
                        if debug:
                            cv2.circle(kare, [x, y], 10, (255, 255, 0), 10)
                    if not a:
                        # yonca
                        sekil[2] += 1
                    elif math.hypot(n1[0] - n3[0], n1[1] - n3[1]) * 1.5 < math.hypot(n2[0] - n4[0], n2[1] - n4[1]) or\
                            math.hypot(n1[0] - n3[0], n1[1] - n3[1]) > 2 * math.hypot(n2[0] - n4[0], n2[1] - n4[1]):
                        # dikdortgen
                        sekil[6] += 1
                    else:
                        # köşegenler farkı
                        k1 = math.hypot(noktaout[0][0] - noktaout[2][0], noktaout[0][1] - noktaout[2][1])
                        k2 = math.hypot(noktaout[1][0] - noktaout[3][0], noktaout[1][1] - noktaout[3][1])
                        if abs(k1 - k2) > min(k1, k2) // 10:
                            # trombus
                            sekil[5] += 1
                        else:
                            # kare
                            sekil[4] += 1
                elif len(noktaout) == 2:
                    # elips
                    sekil[1] += 1
                elif len(noktaout) == 6:
                    # altigen
                    sekil[8] += 1
                # [daire, elips, yonca, ucgen, kare, trombüs, dikdörtgen, beşgen, altıgen, yıldız]
                if debug:
                    print("[daire, elips, yonca, ucgen, kare, trombüs, dikdörtgen, beşgen, altıgen, yıldız]")
                    print(sekil)

                pil_im = Image.fromarray(kare)
                draw = ImageDraw.Draw(pil_im)
                font = ImageFont.truetype(fontdosya_ismi, size=40, encoding="utf-8")
                m = max(sekil)
                text = ""
                if sekil[0] == m:
                    text = "DAİRE"
                if sekil[1] == m:
                    text = "ELİPS"
                if sekil[2] == m:
                    text = "DÖRT YAPRAKLI YONCA"
                if sekil[3] == m:
                    text = "ÜÇGEN"
                if sekil[4] == m:
                    text = "KARE"
                if sekil[5] == m:
                    text = "TROMBÜS"
                if sekil[6] == m:
                    text = "DİKDÖRGEN"
                if sekil[7] == m:
                    text = "BEŞGEN"
                if sekil[8] == m:
                    text = "ALTIGEN"
                if sekil[9] == m:
                    text = "YILDIZ"
                if background_color:
                    left, top, right, bottom = draw.textbbox((rect[0], rect[1] - 50), text, font=font)
                    draw.rectangle((left - 5, top - 5, right + 5, bottom + 5), fill=background_color)
                draw.text((rect[0], rect[1] - 50), text, renk, font=font)
                kare = cv2.cvtColor(np.array(pil_im), cv2.COLOR_BGR2BGRA)
    else:
        contours20 = contours
        cv2.drawContours(kare, contours20, -1, (0, 255, 0), 3)
        losed += 1
    cv2.imshow("ekran", kare)
    if debug:
        if cv2.waitKey(0) == ord("q"):
            break
    else:
        if cv2.waitKey(1) == ord("q"):
            break
