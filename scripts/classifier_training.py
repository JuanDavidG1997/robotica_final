#!/usr/bin/env python

from sklearn import svm
from sklearn.metrics import accuracy_score
from sklearn.datasets import fetch_openml
from sklearn.ensemble import RandomForestClassifier
from sklearn.externals import joblib
from sklearn.model_selection import train_test_split

# Fetch data and split into train and test data
print("Fetching data")
mnist = fetch_openml('mnist_784')
print("Splitting data into train and test")
train_img, test_img, train_lbl, test_lbl = train_test_split(mnist.data, mnist.target, test_size=1/7.0, random_state=0)

print("Creating classifier")
clf = RandomForestClassifier(n_estimators=100)
print("Training classifier")
clf.fit(train_img, train_lbl)

predicted = clf.predict(test_img)

print("Accuracy: ", accuracy_score(test_lbl, predicted))

print(test_img[0].shape)

joblib.dump(clf, "clasificador.pkl", compress=3)