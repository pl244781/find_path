import cv2 as cv
import numpy as np

def FindPath(img_name):
    img = cv.imread(img_name)

    scale_percent = 20
    width = int(img.shape[1] * scale_percent / 100)
    length = int(img.shape[0] * scale_percent / 100)
    dim = (width, length)

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    edges = cv.Canny(gray, 400, 800, None, 3)
    blur = cv.blur(edges, (19, 19))

    linesP = cv.HoughLinesP(blur, cv.HOUGH_PROBABILISTIC, theta=np.pi / 180, threshold=15, minLineLength=30,
                            maxLineGap=10)

    imm = edges.copy()

    if linesP is not None:
        for i in range(len(linesP)):
            l = linesP[i][0]
            cv.line(imm, (l[0], l[1]), (l[2], l[3]), (255, 255, 255), 3, cv.LINE_AA)

    incre = 0

    '''while incre < imm.shape[1]:
        for x in range(imm.shape[1]):
            if imm[imm.shape[0] - 1 - incre][x-1] == 255:
                check1 = x
        for y in range(imm.shape[1]):
            if imm[imm.shape[0] - 1 - incre][imm.shape[1] - y - 1] == 255:
                check2 = y
        mid = int((check1 + check2)/2)'''



    img = cv.resize(img, dim, interpolation=cv.INTER_AREA)
    imm = cv.resize(imm, dim, interpolation=cv.INTER_AREA)
    '''edges = cv.resize(edges, dim, interpolation=cv.INTER_AREA)
    blur = cv.resize(blur, dim, interpolation=cv.INTER_AREA)
    immg = cv.resize(immg, dim, interpolation=cv.INTER_AREA)'''

    cv.imshow("output", imm)
    cv.waitKey(0)
    cv.imshow("output", img)

    new_name = img_name[:len(img_name) - 4] + "_output.jpg"
    cv.imwrite(new_name, img)

    cv.destroyAllWindows()

FindPath("left.jpg")
FindPath("right.jpg")
