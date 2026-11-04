#!/usr/bin/env python3

import csv
import glob
import math
import os
import sys


def leer_csv(ruta): # literalmente leer csv
    xs = []
    ys = []
    with open(ruta, newline="") as archivo:
        lector = csv.reader(archivo)
        next(lector, None)
        for fila in lector:
            xs.append(float(fila[0]))
            ys.append(float(fila[1]))
    return xs, ys


def distancia_a_segmento(px, py, ax, ay, bx, by):
    abx = bx - ax
    aby = by - ay
    largo_cuadrado = abx * abx + aby * aby
    if largo_cuadrado == 0.0
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * abx + (py - ay) * aby) / largo_cuadrado
    t = max(0.0, min(1.0, t))
    cx = ax + t * abx
    cy = ay + t * aby
    return math.hypot(px - cx, py - cy)


def calcular_error(plan_x, plan_y, real_x, real_y):
    suma_de_cuadrados = 0.0
    for i in range(len(real_x)):
        px = real_x[i]
        py = real_y[i]

        distancia_minima = float("inf")
        for j in range(len(plan_x) - 1):
            d = distancia_a_segmento(
                px, py,
                plan_x[j], plan_y[j],
                plan_x[j + 1], plan_y[j + 1])
            if d < distancia_minima:
                distancia_minima = d # nos quedamos con el tramo mas cercano
        suma_de_cuadrados += distancia_minima * distancia_minima
    mse = suma_de_cuadrados / len(real_x) # calculamos el promedio de los cuadrados
    rmse = math.sqrt(mse)
    return mse, rmse


def main():
    if len(sys.argv) > 1:
        carpeta = sys.argv[1]
    else:
        carpeta = "resultados"
    archivos_plan = sorted(glob.glob(f"{carpeta}/*_plan.csv"))

    for ruta_plan in archivos_plan:
        nombre = os.path.basename(ruta_plan).replace("_plan.csv", "")
        ruta_real = carpeta + "/" + nombre + "_real.csv"

        if not os.path.exists(ruta_real):
            print(nombre + ": falta el archivo " + ruta_real)
            continue

        plan_x, plan_y = leer_csv(ruta_plan)
        real_x, real_y = leer_csv(ruta_real)

        if len(plan_x) < 2 or len(real_x) == 0:
            print(nombre + ": no hay datos")
            continue

        mse, rmse = calcular_error(plan_x, plan_y, real_x, real_y)
        print(f"{nombre}: MSE = {mse:.6f} m^2, RMSE = {rmse:.4f} m")


if __name__ == "__main__":
    main()