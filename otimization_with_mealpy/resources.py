from scipy.spatial import distance
import math
import numpy as np

def AcharXYFinal(pos_robo, len_Map, Angulo):
    L = [[0, 0, len_Map[0], 0], [0, 0, 0, len_Map[1]], [0, len_Map[1], len_Map[0], len_Map[1]],
         [len_Map[0], 0, len_Map[0], len_Map[1]]]
    NumRetas = np.size(L, 0)
    map = L
    dist = []
    dists = []
    pxs = []
    pys = []
    angs = []
    for i in range(NumRetas):
        # Posição X1 e Y1 é a posição do Robo
        x1 = pos_robo[0]
        y1 = pos_robo[1]

        if(x1==0 or x1 == len_Map[0] or y1 == 0 or y1 == len_Map[1]):
            return x1, y1

        AngleRobo = pos_robo[2]
        AngleRobo = (AngleRobo + 360) % 360
        if AngleRobo > 180:
            AngleRobo = AngleRobo - 360
        # Posição X2 e Y2 é o segundo ponto para definir a reta

        Angle = -AngleRobo - Angulo
        Angle = (Angle + 360) % 360
        if Angle == -180:
            Angle = 180
        if Angle > 180:
            Angle = Angle - 360

        x2 = x1 + math.cos(np.deg2rad(Angle))
        y2 = y1 + math.sin(np.deg2rad(Angle))

        # (x3,y3)e(x4,y4) são pontos da reta que forma os obstáculos do mapa.

        x3 = map[i][0]
        y3 = map[i][1]
        x4 = map[i][2]
        y4 = map[i][3]

        # Ponto de intersecção
        den = (((x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4)))

        if den == 0:
            # Denominador igual a zero, portanto são Paralelas.
            dist.append(float('inf'))
            pxs.append('inf')
            pys.append('inf')
            angs.append('inf')
        else:
            # Valor diferente de 0, Não paralela
            Px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den
            Py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / den
            thP = np.mod(np.rad2deg(math.atan2(Py - y1, Px - x1)), 360)
            if thP > 180:
                thP = thP - 360

            pxs.append(Px)
            pys.append(Py)
            angs.append(thP)

    for i in range(len(pxs)):
        if pxs[i] != 'inf' and pys[i] != 'inf':
            if pxs[i] >= 0-0.001 and pxs[i] <= len_Map[0]+0.001 and pys[i] >= 0-0.001 and pys[i] <= len_Map[1]+0.001:
                if np.abs(Angle - angs[i]) < 0.1:
                    X_Real = pxs[i]
                    Y_Real = pys[i]

    return X_Real, Y_Real

def getIntersectPts(strPt, endPt, geom=[0,1,0,0,0,1]):
# '''
#     Find intersections pts for every half cell size
#     ** cell size has only been tested with 1

#     Returns cell coordinates that the line passes through
# '''

    x0 = geom[0]
    y0 = geom[3]

    (sX, sY) = (strPt[0], strPt[1])
    (eX, eY) = (endPt[0], endPt[1])
    xSpace = geom[1]
    ySpace = geom[5]

    sXIndex = ((sX - x0) / xSpace)
    sYIndex = ((sY - y0) / ySpace)
    eXIndex = ((eX - sXIndex) / xSpace) + sXIndex
    eYIndex = ((eY - sYIndex) / ySpace) + sYIndex


    dx = (eXIndex - sXIndex)
    dy = (eYIndex - sYIndex)
    xHeading = 1.0 if dx > 0 else -1.0 if dx < 0 else 0.0
    yHeading = 1.0 if dy > 0 else -1.0 if dy < 0 else 0.0

    xOffset = (1 - (math.modf(sXIndex)[0]))
    yOffset = (1 - (math.modf(sYIndex)[0]))

    ptsIndexes = []
    x = sXIndex
    y = sYIndex
    pt = (x, y) #1st pt

    if dx != 0:
        m = (float(dy) / float(dx))
        b = float(sY - sX * m )

    dx = abs(int(dx))
    dy = abs(int(dy))

    if dx == 0:
        for h in range(0, dy + 1):
            pt = (x, y + (yHeading *h))
            ptsIndexes.append(pt)

        return ptsIndexes

    #snap to half a cell size so we can find intersections on cell boundaries
    sXIdxSp = round(2.0 * sXIndex) / 2.0
    sYIdxSp = round(2.0 * sYIndex) / 2.0
    eXIdxSp = round(2.0 * eXIndex) / 2.0
    eYIdxSp = round(2.0 * eYIndex) / 2.0

    # ptsIndexes.append(pt)
    prevPt = False

    #advance half grid size
    for w in range(0, dx * 4):
        x = xHeading * (w / 2.0) + sXIdxSp
        y = (x * m + b)
        if xHeading < 0:
            if x < eXIdxSp:
                break
        else:
            if x > eXIdxSp:
                break

        pt = (round(x), round(y)) #snapToGrid

        if prevPt != pt:
            ptsIndexes.append(pt)
            prevPt = pt

    #advance half grid size
    for h in range(0, dy * 4):
        y = yHeading * (h / 2.0) + sYIdxSp
        x = ((y - b) / m)
        if yHeading < 0:
            if y < eYIdxSp:
                break
        else:
            if y > eYIdxSp:
                break
        pt = (round(x), round(y)) # snapToGrid

        if prevPt != pt:
            ptsIndexes.append(pt)
            prevPt = pt

    return set(ptsIndexes) #elminate duplicates


    aux_points_list = []
    aux_points_list_error = []
    adjust = 0
    for i in range(len(points_list)):
        a = points_list[i][0]
        b = points_list[i][1]
        if a < lenght and b < height:
                aux_points_list.append((points_list[i][0], points_list[i][1]))
        else:
            if a >= lenght:
                while a >= lenght:
                    a = a - 1
                    adjust = adjust + 1
                aux_points_list_error.append((points_list[i][0]-adjust, points_list[i][1]))
            else:
                if b >= height:
                    while b >= height:
                        b = b - 1
                        adjust = adjust + 1
                aux_points_list_error.append((points_list[i][0], points_list[i][1])-adjust)
        adjust = 0
    return aux_points_list, aux_points_list_error

def getObstaclePoints(points_list, bw_img, len_map):
    obstacle_list = []
    for i in range(len(points_list)):
        a = int(points_list[i][0])
        b = int(points_list[i][1])

        if a > len_map[0]:
            a = len_map[0]
        if a < 0:
            a = 0
        if b > len_map[0]:
            b = len_map[0]
        if b < 0:
            b = 0
        if bw_img[b][a] == 0:
            obstacle_list.append((b, a))
    return obstacle_list

def adjustEdges(bw_img):
    bw_img2 = bw_img
    tam = bw_img.shape
    for i in [0, tam[0]-1]:
        for j in range(tam[1]-1):
            bw_img2[i][j] = 0

    for j in [0, tam[0]-1]:
        for i in range(tam[1]-1):
            bw_img2[i][j] = 0

    return bw_img2

def convert_meters_to_pixels(coordinate_in_meters, dimension):
    qtd_pixels = (coordinate_in_meters*dimension)/20
    qtd_pixels = math.ceil(qtd_pixels)
    coordinate_in_pixels = qtd_pixels - 1
    if coordinate_in_pixels < 0:
        return 0
    if coordinate_in_pixels > dimension-1:
        return dimension-1
    return coordinate_in_pixels

def convert_pixels_to_meters(coordinate_in_pixels, dimension):
    value = (20*(coordinate_in_pixels+1))/dimension
    return value

def rangefinder(angle_list, RoboPos, pixels_map, plotar = 0):
    distance = []
    for i in range(len(angle_list)):
        distance.append(rangefinderone(angle_list[i], RoboPos, pixels_map))

    return distance

def rangefinderone(laser_angle, RoboPos, pixels_map):
    # Econtra a dimensão e tamanho do mapa
    dimensions = pixels_map.shape
    len_map = [dimensions[0]-1, dimensions[1]-1]

    # Criar e acha a posição do robo em pixels
    robot_pos_in_pixels = [0,0,0]
    robot_pos_in_pixels[0] = convert_meters_to_pixels(RoboPos[0], dimensions[0])
    robot_pos_in_pixels[1] = convert_meters_to_pixels(RoboPos[1], dimensions[1])
    robot_pos_in_pixels[2] = RoboPos[2]

    x, y = AcharXYFinal(robot_pos_in_pixels, len_map, laser_angle)
    x = round(x)
    y = round(y)

    p1 = [robot_pos_in_pixels[0], robot_pos_in_pixels[1]]
    p2 = [x, y]
    X = getIntersectPts(p1, p2)  # Lista de slots entre a posição do robô e a borda do mapa

    # Converte para uma lista
    points_list = list(X)

    # Encontra os pontos de obstaculo
    obstacle_list = getObstaclePoints(points_list, pixels_map, len_map)

    # Formatando dados
    formatted_robot_pos = np.array([(robot_pos_in_pixels[0], robot_pos_in_pixels[1])])
    obstacle_list = np.array(obstacle_list)

    # Acha a distancia entre os pontos de obstaculo e o melhor ponto
    distance_list = distance.cdist(obstacle_list, formatted_robot_pos)
    best_point = obstacle_list[np.argmin(distance_list)]

    best_point_in_meters = [0, 0]
    best_point_in_meters[0] = convert_pixels_to_meters(best_point[1], dimensions[0])
    best_point_in_meters[1] = convert_pixels_to_meters(best_point[0], dimensions[1])

    distance_in_meters = math.sqrt(((best_point_in_meters[0]-RoboPos[0])**2) + ((best_point_in_meters[1]-RoboPos[1])**2))
    return distance_in_meters