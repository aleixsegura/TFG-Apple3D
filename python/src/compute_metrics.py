"""
This program analyzes the accuracy of a generated point cloud by computing metrics related to the 
geometrical characteristics of the trees, with respect to Viametris LiDAR data (considered as ground truth).
"""

import sys
import pandas
import numpy as np
import matplotlib.pyplot as plt
from typing import List
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_absolute_error, mean_absolute_percentage_error, root_mean_squared_error

LIDAR_MODEL = f'{sys.argv[1]}'.lower()

GEOMETRICAL_GROUND_TRUTH_PATH = '../geometry/viametris_go.txt'
GEOMETRICAL_COMPUTED_PCL_PATH = f'../geometry/{LIDAR_MODEL}_go.txt'

GEOMETRICAL_CHARACTERISTICS = [
    'AmpleMax', 'Ample95', 'AltMax', 'Alt95', 
    'S_CHULL', 'S_AmpleMax', 'S_Ample90',
    'S_reticula005'
]

def export_to_txt(mae_results: List[np.float64], mape_results: List[np.float64], rmse_results: List[np.float64], r2_results: List[np.float64]):
    metrics_file = f'../results/metrics/{LIDAR_MODEL}/metrics.txt'
    
    name_width = max(len(name) for name in GEOMETRICAL_CHARACTERISTICS) + 2
    col_width = 12

    with open(metrics_file, 'w', encoding='utf-8') as f:
        f.write(f'{"":<{name_width}}{"MAE":>{col_width}}{"MAPE":>{col_width}}{"RMSE":>{col_width}}{"R2":>{col_width}}\n')

        for geom_characteristic, mae, mape, rmse, r2 in zip(
                                                            GEOMETRICAL_CHARACTERISTICS, 
                                                            mae_results,
                                                            mape_results, 
                                                            rmse_results, 
                                                            r2_results
                                                        ):
            f.write(f'{geom_characteristic:<{name_width}}{mae:>{col_width}.2f}{mape:>{col_width}.2f}{rmse:>{col_width}.2f}{r2:>{col_width}.2f}\n')
        

def main():
    ground_truth_df = pandas.read_csv(GEOMETRICAL_GROUND_TRUTH_PATH, sep=r'\s+')
    computed_pcl_df = pandas.read_csv(GEOMETRICAL_COMPUTED_PCL_PATH, sep=r'\s+')


    ground_truth_geometrical_data = [ground_truth_df[feature] for feature in GEOMETRICAL_CHARACTERISTICS]
    computed_pcl_geometrical_data = [computed_pcl_df[feature] for feature in GEOMETRICAL_CHARACTERISTICS]

    mae_results, mape_results, rmse_results  = [], [], []

    for gt_series, pcl_series in zip(ground_truth_geometrical_data, computed_pcl_geometrical_data):
        MAE  = np.round(mean_absolute_error(gt_series, pcl_series), 2)
        MAPE = np.round(mean_absolute_percentage_error(gt_series, pcl_series) * 100, 2)
        RMSE = np.round(root_mean_squared_error(gt_series, pcl_series), 2)
        
        mae_results.append(MAE)
        mape_results.append(MAPE)
        rmse_results.append(RMSE)

    
    r_squared_results = []

    for feature in GEOMETRICAL_CHARACTERISTICS:
        x = ground_truth_df[feature].values.reshape(-1, 1)
        y = computed_pcl_df[feature].values.reshape(-1, 1)

        model = LinearRegression()
        model.fit(x, y)

        r_squared = model.score(x, y)  
        r_squared_results.append(np.round(r_squared, 2))

        x_line = np.linspace(x.min(), x.max(), 100).reshape(-1, 1)
        y_line = model.predict(x_line)

        plt.figure(figsize=(5, 4))
        plt.scatter(x, y, label='Computed Data', alpha=0.7)
        plt.plot(x_line, y_line, color='red', label=f'Linear Regression\n$y = {model.coef_[0][0]:.2f}x + {model.intercept_[0]:.2f}$')
        plt.xlabel('Ground Truth')
        plt.ylabel('Computed Point Cloud')
        plt.title(f'{feature}')
        plt.legend()
        plt.grid(True)
        plt.savefig(f'../results/metrics/{LIDAR_MODEL}/regression_charts/{feature}_regression.png')
        plt.close()

    export_to_txt(mae_results, mape_results, rmse_results, r_squared_results)

if __name__ == '__main__':
    main()
