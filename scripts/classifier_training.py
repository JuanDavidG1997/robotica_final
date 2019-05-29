#!/usr/bin/env python

from sklearn.externals import joblib
from sklearn.datasets import fetch_openml
from sklearn.linear_model import LogisticRegression
from sklearn.model_selection import train_test_split


mnist = fetch_openml('mnist_784')

train_img, test_img, train_lbl, test_lbl = train_test_split(mnist.data, mnist.target, test_size=1/7.0, random_state=0)

logisticRegr = LogisticRegression(solver = 'lbfgs')

logisticRegr.fit(train_img, train_lbl)

score = logisticRegr.score(test_img, test_lbl)

print(test_img[0])

print(score)

joblib.dump(logisticRegr, "clasificador.pkl", compress=3)