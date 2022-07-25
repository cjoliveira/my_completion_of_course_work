#!/usr/bin/python3
import cv2

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

img = cv2.imread('img.jpeg', 0) # Lê a imagem em escala preto e branco
img = cv2.resize(img, (40, 40), interpolation = cv2.INTER_AREA) # Converte a imagem para dimensao dada
ret, bw_img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY) # Transforma a imagem em preto(0) e branco(255)

bw_img = adjustEdges(bw_img) # Ajusta as bordas da imagem, caso tenha algum erro na leitura

cv2.imshow("Binary", bw_img)
cv2.waitKey(0)
cv2.destroyAllWindows()