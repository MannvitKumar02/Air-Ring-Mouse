# Air-Ring-Mouse
# IMU Ring Mouse — Gesture-Controlled Air Mouse via BLE IMU Sensor

## Overview

This project implements a real-time gesture recognition system that converts hand movements into mouse control actions using a BLE-connected IMU wearable. A 6-axis inertial measurement unit (accelerometer + gyroscope) worn on the hand captures motion data wirelessly over Bluetooth Low Energy. A machine learning model running on the host PC classifies these gestures in real time and translates them into mouse events such as cursor movement, clicks, scrolling, and swiping.

The motivation behind this project is to provide a hands-free, contactless human-computer interaction interface that can be useful in accessibility applications, presentations, or any scenario where a physical mouse is impractical.

---

## How It Works

The system operates in three stages: data collection, model training, and real-time inference.

**Data Collection** — A GUI application connects to the IMU device over BLE and records raw sensor readings (accelerometer and gyroscope on X, Y, Z axes) into CSV files. Each recording is labeled with a gesture name and a session identifier. Multiple sessions across different days are collected to improve generalization.

**Model Training** — The collected CSV files are preprocessed and used to train a Random Forest classifier. Each recording is resampled to a fixed time interval of 15ms, trimmed to the active gesture region, and split into overlapping windows of 600ms. From each window, 42 statistical features are extracted across six signal channels (gyroscope axes, gyroscope magnitude, accelerometer magnitude, and high-pass filtered accelerometer magnitude). The model is trained using a session-grouped train-test split to simulate real-world deployment, and class imbalance is handled by oversampling minority classes only on the training split.

**Real-time Inference** — The trained model is loaded into a desktop application that continuously receives BLE sensor data, extracts features from a sliding buffer, and predicts the current gesture. A hybrid idle detection mechanism (combining model prediction and raw sensor thresholds) locks the cursor in place when the hand is still, allowing precise targeting. When a tap jerk is detected during an idle lock, a short recording window is captured and classified to determine whether it was a single click, double click, right click, or a swipe gesture. Pointer movement is derived from gyroscope readings with exponential smoothing.

---

## Gesture Classes

The system recognizes eight gesture classes: left click, right click, double click, swipe up, swipe down, swipe left, swipe right, and idle. Discrete gestures (clicks and swipes) are triggered during cursor freeze windows, while idle keeps the cursor stationary to allow precise aiming.

---

## ML Pipeline Details

Raw BLE packets are decoded as comma-separated integers representing the six sensor axes. The data is buffered over a 2.5-second rolling window and resampled to a uniform 15ms grid using linear interpolation. A moving-average high-pass filter removes the gravity component from the accelerometer magnitude. Features are computed over a 40-sample (600ms) window using seven statistics per channel: mean, standard deviation, minimum, maximum, range, RMS, and slope. This produces a 42-dimensional feature vector fed into a `StandardScaler` + `RandomForestClassifier` sklearn pipeline with 900 estimators. Training uses `GroupShuffleSplit` to hold out full sessions as the test set, preventing data leakage between splits.

---

## Setup and Usage

Install dependencies:
```
pip install numpy pandas scikit-learn joblib bleak pyautogui matplotlib seaborn
```

**Collect data** by running `CODE_v4/data_collector_updated.py`. Connect your BLE device, select a gesture label, and press A to start and S to stop each sample recording. Data is saved to `~/Desktop/IMU_Gesture_Data/<GestureLabel>/<session_id>/sampleN.csv`.

**Train the model** by running `CODE_v6/model_train.py`. This reads all CSVs from the data directory, trains the pipeline, and saves `gesture_rf.joblib`.

**Run real-time control** by running `CODE_v6/rt_model.py` with the model file in the same directory. Scan for your BLE device, press F5 to connect, and F7 to toggle mouse control. Press F6 to stop.

**Generate evaluation plots** using `CODE_v6/vis_v2.py`, which produces confusion matrix, per-class metrics, feature importances, PCA scatter, and accuracy charts saved to `CODE_v6/plots/`.

---

## Hardware

The system works with any BLE IMU module that streams six comma-separated integer values (Ax, Ay, Az, Gx, Gy, Gz) over a BLE notification characteristic. The default characteristic UUID is `0000FEF4-0000-1000-8000-00805F9B34FB`. The project was developed and tested with a custom ring-form-factor wearable built around an MPU-6050 class sensor.

---

## Project Context

This project was developed as part of an M.Tech IoT coursework. The codebase evolved across multiple versions, with `CODE_v6` being the latest stable version featuring the unified 8-class model, and `CODE_v4` containing the most complete data collection tool.
