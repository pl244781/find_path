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

    tester = 10

    incre = imm.shape[0]//tester
    h_points = []
    v_points = []

    for j in range(tester):
        bool1 = False
        bool2 = False
        y = imm.shape[0] - incre*j - 1
        for x in range(imm.shape[1] - 1):
            if imm[y, x - 1] == 255:
                save1 = x
                bool1 = True
                break
        for x in range(imm.shape[1] - 1):
            if imm[y, imm.shape[1] - x - 1] == 255:
                save2 = imm.shape[1] - x - 1
                bool2 = True
                break
        if (bool1 and bool2) and (save2-save1 > 50):
            mid = (save1 + save2)//2
            h_points.append((mid, y))
        else:
            continue

    for j in range(tester):
        bool1 = False
        bool2 = False
        x = imm.shape[1] - incre*j - 1
        for y in range(imm.shape[0] - 1):
            if imm[y - 1, x] == 255:
                save1 = y
                bool1 = True
                break
        for y in range(imm.shape[0] - 1):
            if imm[imm.shape[0] - y - 1, x] == 255:
                save2 = imm.shape[0] - y - 1
                bool2 = True
                break
        if (bool1 and bool2) and (save2-save1 > 50):
            mid = (save1 + save2)//2
            v_points.append((x, mid))
        else:
            continue

    slope = (v_points[0][1]-h_points[0][1])//(v_points[0][0]-h_points[0][0])

    if slope > 0:
        points = h_points + v_points
    else:
        v_points = sorted(v_points, key=lambda tup:tup[0])
        points = h_points + v_points

    print(points)

    for i in range(len(points)-2):
        if i != 3 and i != 4 and i != 5:
            cv.arrowedLine(img, (points[i][0], points[i][1]), (points[i+1][0], points[i+1][1]), (0, 0, 0), 3, cv.LINE_AA)
        elif i == 3:
            cv.arrowedLine(img, (points[i][0], points[i][1]), (points[i + 3][0], points[i + 3][1]), (0, 0, 0), 3, cv.LINE_AA)


    img = cv.resize(img, dim, interpolation=cv.INTER_AREA)

    cv.imshow("output", img)

    new_name = img_name[:len(img_name) - 4] + "_output.jpg"
    cv.imwrite(new_name, img)

    cv.waitKey(0)
    cv.destroyAllWindows()

FindPath("left.jpg")
FindPath("right.jpg")
