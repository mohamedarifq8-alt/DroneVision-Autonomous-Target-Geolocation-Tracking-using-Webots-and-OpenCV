# DroneVision-Autonomous-Target-Geolocation-Tracking-using-Webots-and-OpenCV

**** is an advanced aerial reconnaissance project developed in the **Webots** simulator. It features a quadcopter (Mavic 2 Pro) capable of autonomous takeoff, computer vision-based target tracking, and precise geographical coordinate estimation of ground objects.

## 🌟 Key Features
*   **Autonomous Mission Cycle:** Fully automated "Takeoff -> Scan -> Track -> Capture" workflow.
*   **Computer Vision Integration:** Real-time red-color segmentation using **OpenCV** (HSV Color Space).
*   **Precise Geolocation:** A mathematical model that translates pixel coordinates from the camera feed into global World Coordinates (X, Y) using GPS and IMU data.
*   **Visual Servoing:** Dynamic PID adjustment to center the drone directly above the detected target.
*   **Automated Reporting:** Generates a timestamped image (`.png`) and a text report (`.txt`) containing the target's precise coordinates.

---

### 📸 Visual Documentation

#### 1. Computer Vision Pipeline (HSV Masking)
The drone's "eye" identifying the target using color thresholding and contour detection.
![Target Detection](./protos/Screenshot%202026-04-24%20031539.png)

#### 2. Precision Hover & Capture
The stabilization phase where the drone aligns itself at 6.5m altitude to capture the target data.
![Hover and Capture](./protos/Screenshot%202026-05-02%20001558.png)

---

## 🏗 System Architecture

### 1. Vision Logic (OpenCV)
The drone processes raw RGBA frames into BGR, then applies an **HSV Mask** to isolate specific wavelengths (Red).
* **Noise Reduction:** Contours are filtered by area (`cv2.contourArea > 300`) to ignore background artifacts.
* **Target Centering:** Calculates the `off_x` and `off_y` representing the distance of the target from the center of the frame.

### 2. Geolocation Mathematics (The Control Engine)
This is the core engineering highlight. The system calculates the target's position using:
* **FOV Consideration:** Translating pixel offsets into angular deviations based on the camera's Field of View.
* **Gimbal Compensation:** Correcting for the camera's tilt angle (Gimbal Pitch).
* **Trigonometric Projection:** Using `drone_alt * np.tan(angle)` to project the pixel location onto the 2D ground plane.

### 3. Flight Control (FSM)
| State | Behavior |
| :--- | :--- |
| **TAKEOFF** | Ascends to 6.5 meters using PD altitude control. |
| **SCANNING** | Performs a forward search pattern until a target enters the FOV. |
| **TRACKING** | Actively adjusts Roll, Pitch, and Yaw to center the target in the camera. |
| **CAPTURING** | Once centered (`error < 0.1`), it triggers the data saving sequence. |

---

## 🛠 Technical Specifications
* **Programming Language:** Python 3.x
* **Primary Libraries:** `OpenCV (cv2)`, `NumPy`, `Controller (Webots API)`
* **Control Loops:** Triple-axis PID (Roll, Pitch, Yaw) + Altitude PD.
* **Outputs:** Timestamped `.png` captures and `.txt` reports with X/Y coordinates.

---

## 👨‍💻 Author
**Mohamed Arif Mahyoub Haider**
*Electrical Engineer - Computer and Industrial Control*

---

### 🏷 Project Topics
`webots` `computer-vision` `opencv` `autonomous-drones` `pid-control` `geolocation` `robotics` `target-tracking` `python`
