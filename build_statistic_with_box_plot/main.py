import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

ERRO_TOTAL = "erro_total"

time_metric = "População • Quantidade de Ciclos de Clock"

theta_metric = "População • Erro absoluto em Theta"

y_metric = "População • Erro absoluto em Y"

x_metric = "População • Erro absoluto em X"


# Método usado para gerar o dataframe que usamos no box-plot [POPULAÇÃO X ERRO ABSOLUTO EM METRIC]

def print_box_plot_for_all_populations_with_metrics(dataframe, metric):
    fig, ax = plt.subplots()
    population = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 2000, 3000, 4000, 5000]
    data = []

    for pop_size in population:
        data.append(generate_data_by_metric_and_population(dataframe, metric, pop_size))

    ax.yaxis.grid(True)
    ax.set_xlabel('População', fontsize=28)
    ax.set_ylabel('Erro total absoluto', fontsize=28)
    ax.set_xticklabels(population, fontsize=10)
    bp = ax.boxplot(data, sym='k+', notch=1, patch_artist=True, bootstrap=5000, boxprops=dict(facecolor='darkslategrey'))
    fig.set_size_inches(18.5, 10.5, forward=True)
    plt.show()

def generate_data_by_metric_and_population(dataframe, metric, population):
    data = dataframe.loc[dataframe["population"] == population]
    if metric == x_metric:
        data = np.array(abs(data["X_found"]-data["X_original"]))
        return data
    if metric == y_metric:
        data = np.array(abs(data["Y_found"]-data["Y_original"]))
        return data
    if metric == theta_metric:
        data = np.array(abs(data["Thetha_found"]-data["Thetha_original"]))
        return data
    if metric == time_metric:
        data = np.array(data["time in clocks"])
        return data
    if metric == ERRO_TOTAL:
        data = np.array(data[ERRO_TOTAL])
        return data

# Método usado para gerar o dataframe que usamos no box-plot [MÉTODO X ERRO TOTAL]

def print_box_plot_for_methods_with_metrics(dataframe, metric, best_population):
    fig, ax = plt.subplots()
    methods_list = [0, 1, 2, 3]
    methods_labels = ["GA", "BA", "ACO-R", "SMA"]
    data = []

    for id in methods_list:
        data.append(generate_data_by_metric_by_method_by_best_population(dataframe, id, metric, best_population))
    ax.yaxis.grid(True)
    ax.set_xlabel('Métodos', fontsize=28)
    ax.set_ylabel('Erro total absoluto', fontsize=28)
    ax.set_xticklabels(methods_labels, fontsize=14)
    bp = ax.boxplot(data, sym='k+', notch=1, patch_artist=True, bootstrap=5000, boxprops=dict(facecolor='steelblue'))
    fig.set_size_inches(18.5, 10.5, forward=True)
    plt.show()

def generate_data_by_metric_by_method_by_best_population(dataframe, method_id, metric, population):
    data = dataframe.loc[dataframe["population"] == population]
    data = data.loc[data["method"] == method_id]
    data = np.array(data[metric])
    return data

df = pd.read_csv("doc_completo.csv")
print_box_plot_for_all_populations_with_metrics(df, ERRO_TOTAL)

