'''
    File name: linear_regression.py
    Author: zeabus2018
    Date created: 1/18/2018
    Python Version: 3.6.1
'''
import csv
import numpy as np
from sklearn import linear_model
from sklearn.metrics import mean_squared_error, explained_variance_score

abs_path = 'C:/Users/skconan/Desktop/Workspace/dice/'
img_path = abs_path + 'images/'

def main():
    X_train = []
    y_train = []
    with open(abs_path+'dataset.csv', newline='\n') as csvfile:
        csvfile.readline()
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            x = []
            for col in row[:-1]:
                x.append(int(col))
            X_train.append(x)
            y_train.append(int(row[-1]))
    print(X_train)
    print(y_train)
    regr = linear_model.LinearRegression()
    regr.fit(X_train, y_train)
    # The coefficients
    print('Coefficients: \n', regr.coef_)
    print('Intercept: ',regr.intercept_)
    for co in regr.coef_:
        print(float('%.8lf' % co),',')
    # Make predictions using the testing set
    # pomelo_y_pred = regr.predict(pomelo_X_test)

if __name__=='__main__':
    main()