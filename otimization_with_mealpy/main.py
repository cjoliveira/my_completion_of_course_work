#!/usr/bin/python3
import numpy as np
import resources
import math
import cv2
import csv
import time

from mealpy.evolutionary_based import GA
from mealpy.swarm_based import BA
from mealpy.swarm_based import ACOR
from mealpy.bio_based import SMA
from hwcounter import Timer, count, count_end

def quaternion_to_euler(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
        
def get_original_position(req):
    global pos_aux
    global original_pos
    if pos_aux==0:
        pos_aux = 1
        original_pos=req

def scan(req):
    global aux
    global scan_readings
    if aux == 0:
        aux = 1
        Scan_Data = req
        Laser_Ang_max = Scan_Data.angle_max
        Laser_Ang_min = Scan_Data.angle_min
        Laser_Ang_Incre = Scan_Data.angle_increment
        Laser_Ang_ranges = Scan_Data.ranges
        Angulos = np.arange(Laser_Ang_min,Laser_Ang_max,Laser_Ang_Incre)
        scan_readings = np.array(Laser_Ang_ranges)
  
    return 0

def fitness(Ind):
    global L
    global angles_list
    global scan_readings
    dist = resources.rangefinder(angles_list, Ind, L)

    erro = 0

    for j in range(len(angles_list)):
        erro =  erro + (np.abs(dist[j] - scan_readings[j]))

    Fitness = erro
    return Fitness

def read_bd():
    pos_list = []
    scan_list = []
    aux = 0
    with open('bd_ros.csv', 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            if aux > 0:
                aux_pos_array = []
                aux_scan_array = []
                for i in range(len(row)):
                    if i<3:
                        aux_pos_array.append(float(row[i]))
                    else:
                        aux_scan_array.append(float(row[i]))
                
                pos_list.append(aux_pos_array)
                scan_list.append(aux_scan_array)

            aux = aux + 1
    
    return pos_list, scan_list

if __name__ == "__main__":
    global aux
    global scan_readings
    global angle_list
    global L
    global pos_aux
    global original_pos

    img = cv2.imread('img_final.jpeg', 0)  # Lê a imagem em escala preto e branco
    img = cv2.resize(img, (40, 40), interpolation=cv2.INTER_AREA)  # Converte a imagem para dimensao dada
    ret, bw_img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)  # Transforma a imagem em preto(0) e branco(255)
    bw_img = resources.adjustEdges(bw_img)
    
    cv2.imshow("image", bw_img)
  
    # waits for user to press any key
    # (this is necessary to avoid Python kernel form crashing)
    cv2.waitKey(0) 
    
    # closing all open windows
    cv2.destroyAllWindows() 
    L = bw_img

    aux = 0
    scan_readings = []
    pos_aux = 0
    
    pos_list, scan_list = read_bd()
  
    
    angles_list = np.linspace(-90,90,10)
    print('Angulos Escolhidos pro Laser:',angles_list)


    problem_size = 3              # Dimensão do Problema
    epoch = 50                    # Número de Gerações que o problema irá executar
    pop_size = np.array([100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 2000, 3000, 4000, 5000])     # Número de Indivíduos da Populaçao Inicial
    pc = 0.95                     # Probabilidade de Crossover
    pm = 0.025                    # Probabilidade de Mutação
    obj_func = fitness            # Função de Aptidão
    varbound=np.array([[0,20],
                    [0,20],
                    [-180,180]]) # Limite máximo e minimo da população

    algorithm_param = {
        "obj_func": fitness,
        "lb": [0, 0, -180],
        "ub": [20, 20, 180 ],
        "minmax": "min",
        "verbose": True,
    }

    Pos_Original = [0,0,0]
    f = open('dados.csv', 'w', newline='', encoding='utf-8')
    w = csv.writer(f)
    w.writerow(["iteration", "method", "population", "time in sec", "time in clocks", "X_found", "Y_found", "Thetha_found", "X_original", "Y_original", "Thetha_original", "global_best"])
    for i in range(14):
        for j in range(4):
            method = j
            for k in range(20):
                start_execution_time = time.time()
                start_clock_time = count()
                Pos_Original[0] = pos_list[k][0]
                Pos_Original[1] = pos_list[k][1]
                Pos_Original[2] = pos_list[k][2]
                scan_readings = np.array(scan_list[k])
                print("################################### i="+str(i)+ " j="+str(j)+" k=" + str(k) +" ###################################")

                if method == 0:
                    model = GA.BaseGA(algorithm_param, epoch, pop_size[i])
                if method == 1:
                    model = BA.BaseBA(algorithm_param, epoch, pop_size[i])
                if method == 2:
                    model = ACOR.BaseACOR(algorithm_param, epoch, pop_size[i])
                if method == 3:
                    model = SMA.BaseSMA(algorithm_param, epoch, pop_size[i])

                model.solve()
                Pos_GA = model.solution[0]
                global_best = model.solution[1][0]

                end_execution_time = time.time()
                end_clock_time = count_end()
                
                execution_time = end_execution_time - start_execution_time
                clock_time = end_clock_time - start_clock_time

                w.writerow([k, method, pop_size[i], execution_time, clock_time, Pos_GA[0]-10, Pos_GA[1]-10, Pos_GA[2], Pos_Original[0], Pos_Original[1], Pos_Original[2], global_best])

    f.close()

