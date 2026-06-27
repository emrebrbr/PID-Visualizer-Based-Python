# 🎛️ Interactive PID Control Simulator

This project is an interactive **PID (Proportional-Integral-Derivative) Control Simulator** built with **Python and Streamlit**. It mathematically models a room's temperature control loop based on Newton's Law of Cooling, allowing users to tune PID parameters in real-time and observe the system's response.

## ✨ Key Features

* **Real-time Tuning:** Instantly see how changes in $K_p$, $K_i$, and $K_d$ affect the system's overshoot, settling time, and steady-state error.
* **Anti-Windup Protection:** Implements integral clamping to prevent the I-term from accumulating excessively when the heater reaches its 100% saturation limit.
* **Sensor Noise Modeling:** Test the robustness of the controller by introducing dynamic, normally-distributed noise to the temperature readings.
* **Data Export:** Download the simulation results as a CSV file for further analysis.

## 🧠 Theoretical Background

The system simulates temperature dynamics using the discrete Euler method based on Newton’s Law of Cooling. The PID controller minimizes the error $e(t)$ between the setpoint and measured temperature:

$$u(t) = K_p e(t) + K_i \int e(t)dt + K_d \frac{de(t)}{dt}$$
## 🚀 How to Run

1. **Clone the repository:**
   ```bash
   git clone https://github.com/emrebrbr/PID-Visualizer-Based-Python.git
   cd PID-Visualizer-Based-Python
   ```

2. **Install the dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

3. **Run the simulation:**
   ```bash
   streamlit run "PID Controller.py"
