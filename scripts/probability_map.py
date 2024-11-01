import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import cm



if __name__ == '__main__':
    probability_map = pd.read_csv("build/test.csv", header=None).values
    print(probability_map)
    
    fig = plt.figure()
    ax = fig.add_subplot()
    x1 = np.unique(probability_map[:, 0])
    x2 = np.unique(probability_map[:, 1])
    p = probability_map[:, 2].reshape((x1.shape[0], x2.shape[0]))

    ax.contourf(x1, x2, p, cmap=cm.coolwarm, alpha=0.35, levels=50)
    ax.set_xlim([x1.min(), x1.max()])
    ax.set_ylim([x2.min(), x2.max()])
    ax.set_aspect('equal', 'box')
    plt.show()


