# -*- coding: utf-8 -*-
"""
Created on Thu Nov  2 14:33:43 2023

@author: Sagnik Sen
"""

import numpy as np
from sklearn.neural_network import MLPClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.naive_bayes import GaussianNB
from sklearn.svm import SVC
from sklearn.metrics import classification_report, confusion_matrix
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler

import pandas as pd

# Load the final cvs folder after adjusting signs' images
df = pd.read_csv("final_dataset.csv")

train, test = train_test_split(df, test_size=0.2, random_state=42)
features = ['Feature1', 'Feature2', 'Feature3', 'Feature4', 'Feature5', 'X', 'Y', 'Z', 'Category1', 'Category2']
x_train = train[features]
y_train = train['Label']
x_test = test[features]
y_test = test['Label']

# Standardizing the features
scaler = StandardScaler()
x_train = scaler.fit_transform(x_train)
x_test = scaler.transform(x_test)

# Initializing and training the Random Forest Classifier
rf_classifier = RandomForestClassifier(n_estimators=100, random_state=42)
rf_classifier.fit(x_train, y_train)

# Initializing and training the Naïve Bayes Classifier
nb_classifier = GaussianNB()
nb_classifier.fit(x_train, y_train)

# Initializing and training the SVM Classifier
svm_classifier = SVC(kernel='linear', C=1)
svm_classifier.fit(x_train, y_train)

# MOdel predictions
rf_predictions = rf_classifier.predict(x_test)
nb_predictions = nb_classifier.predict(x_test)
svm_predictions = svm_classifier.predict(x_test)

# Evaluateion of models
print("Random Forest Classifier Results:")
print(confusion_matrix(y_test, rf_predictions))
print(classification_report(y_test, rf_predictions))

print("Naïve Bayes Classifier Results:")
print(confusion_matrix(y_test, nb_predictions))
print(classification_report(y_test, nb_predictions))

print("Support Vector Machine Classifier Results:")
print(confusion_matrix(y_test, svm_predictions))
print(classification_report(y_test, svm_predictions))
