import numpy as np
import math, os, json
from sklearn.linear_model import LinearRegression


def linear_regression(x, y):
    lm = LinearRegression()
    lm.fit(np.reshape(x, (len(x), 1)), np.reshape(y, (len(y), 1)))

    # 印出係數
    # print(lm.coef_[0])

    # 印出截距
    # print(lm.intercept_)

    n = 1 / lm.coef_[0][0]
    a = -1 * lm.intercept_[0] * n
    # print(n, a)

    return n, a

def gradient_descent(x, y):
    learning_rate=0.0001
    #經過3000次的調整參數
    n_iterations=3000

    #隨機定義參數值
    theta = np.random.randn(2,1)
    #如同最小平方法,必須加入截距項
    x=np.concatenate((np.ones((x.shape[0], 1)),x[:,np.newaxis]),axis=1)
    y=y[:,np.newaxis]
    # print(x, y)

    #來畫點不一樣風格的圖吧
    #開始梯度下降
    for iteration in range(n_iterations):
        #求出預測的yhat值
        scores = np.dot(x, theta)
        #誤差值
        output_error = y-scores
        #x的shape(50,2) output_error的shapeｊ為(50,1) gradients為(2,1)
        gradients = 2*np.dot(x.T, output_error)
        #每次對theta
        theta += learning_rate*gradients
    # print(theta)
    n = 1 / theta[1]
    a = -1 * theta[0] * n
    # print(n, a)
    return n, a

def main():
    DB_NAME = '/rssi_db.json'
    FILEPATH = os.path.dirname(os.path.abspath(__file__))
    DB_PATH = FILEPATH + DB_NAME
    rssi_db = dict()
    if os.path.isfile(DB_PATH):
        with open(DB_PATH) as infile:
            rssi_db = json.load(infile)
        infile.close()

    param_dict = dict()
    for w in rssi_db.keys():
        rssi = []
        dist = []
        for d in rssi_db[w].keys():
            for r in rssi_db[w][d]:
                rssi.append(float(r))
                dist.append(float(d))
        rssi = np.array(rssi)
        dist = np.array(dist)
        rssi = np.array([abs(r) for r in rssi])
        dist = np.array([math.log10(d) * 10 for d in dist])

        # n, a = gradient_descent(rssi, dist)
        n, a = linear_regression(rssi, dist)
        param_dict[w] = [n, a]

    for w in param_dict.keys():
        print(w)
        print(param_dict[w])

    # dist = np.array([7.10828389978903, 7.7730688920142725, 8.493933129004489, 7.474356159563177, 7.108283899789034, 7.7730688920142725, 8.493933129004489, 7.474356159563177, 7.108283899789034, 7.7730688920142725, 8.493933129004489, 7.474356159563177])
    # rssi = np.array([-53, -48, -47, -58, -45, -43, -56, -53, -43.7, -42.9, -54.6, -51.3])
    # dist = np.array([7.108283899789034, 7.7730688920142725, 8.493933129004489, 7.474356159563177])
    # rssi = np.array([-43.7, -42.9, -54.6, -51.3])
    # dist = np.array([math.log10(d) * 10 for d in dist])
    # rssi = np.array([abs(r) for r in rssi])

    # gradient_descent(rssi, dist)
    # linear_regression(rssi, dist)

if __name__ == "__main__":
    main()