import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# --- Page Configuration ---
st.set_page_config(
    page_title="PID Control Simulator",
    page_icon="🎛️",
    layout="wide"
)

# --- Title and Description ---
st.title("🎛️ PID Temperature Control Simulation")
st.markdown("""
This project simulates a **PID control loop** designed to maintain a room's temperature at a desired **Set Point**.
The system is modeled based on Newton's Law of Cooling.
""")

# --- Sidebar: Parameter Inputs ---
st.sidebar.header("⚙️ System Settings")

# 1. Targets
st.sidebar.subheader("State Variables")
target_temp = st.sidebar.slider("Target Temperature (Set Point) [°C]", 0, 100, 50)
initial_temp = st.sidebar.slider("Initial Room Temperature [°C]", 0, 100, 20)

# 2. PID Coefficients
st.sidebar.subheader("PID Parameters")
Kp = st.sidebar.slider("Kp (Proportional Gain)", 0.0, 10.0, 2.0)
Ki = st.sidebar.slider("Ki (Integral Gain)", 0.0, 2.0, 0.1)
Kd = st.sidebar.slider("Kd (Derivative Gain)", 0.0, 5.0, 1.0)

# 3. Advanced Settings
st.sidebar.subheader("Simulation Details")
anti_windup_on = st.sidebar.checkbox("🟢 Anti-Windup (İntegral Şişmesini Önle)", value=True)

# YENİ EKLENEN: Gürültü Yüzde Sürgüsü
noise_percent = st.sidebar.slider(
    "Sensor Noise Level (%)", 
    0.0, 20.0, 0.0, step=0.5, 
    help="Gerçek dünyadaki sensör hatalarını (gürültüyü) hedefin yüzdesi cinsinden simüle eder. %0 = Gürültüsüz."
)

simulation_duration = st.sidebar.slider("Simulation Duration (seconds)", 10, 200, 100)

# --- Simulation Engine ---
def run_pid_simulation(Kp, Ki, Kd, setpoint, init_temp, duration, noise_level, anti_windup):
    # Time settings
    dt = 0.1
    steps = int(duration / dt)
    time_values = np.linspace(0, duration, steps)
    
    # Data storage arrays
    temps = np.zeros(steps)
    measured_temps = np.zeros(steps) # Sensörün okuduğu gürültülü veriyi tutmak için
    errors = np.zeros(steps)
    control_signals = np.zeros(steps)
    p_terms = np.zeros(steps)
    i_terms = np.zeros(steps)
    d_terms = np.zeros(steps)
    
    # Initial Conditions
    current_temp = init_temp
    integral = 0
    previous_error = 0
    
    # Physical Model Constants
    thermal_inertia = 25.0  
    ambient_temp = 10.0     
    cooling_constant = 0.1  
    
    for i in range(steps):
        # 1. Sensör Okuması ve Gürültü 
        # Gerçek oda sıcaklığını okuyoruz ve üzerine kullanıcının belirlediği oranda gürültü ekleniyor
        measured_temp = current_temp
        if noise_level > 0:
            # Gürültü standart sapmasını hedef sıcaklığın yüzdesi olarak hesapla
            noise_std = (noise_level / 100.0) * setpoint
            measured_temp += np.random.normal(0, noise_std)
            
        # 2. Hata Hesaplama (Artık kontrolcü gerçeği değil, gürültülü sensör verisini referans alıyor)
        error = setpoint - measured_temp
        
        # 3. PID Terms Calculation
        P = Kp * error
        integral += error * dt
        
        if anti_windup and Ki > 0:
            max_integral_limit = 100.0 / Ki
            integral = max(-max_integral_limit, min(max_integral_limit, integral))
            
        I = Ki * integral
        
        derivative = (error - previous_error) / dt
        D = Kd * derivative
        
        # Total Control Signal (u(t))
        u = P + I + D
        u = max(0, min(100, u))
            
        # 4. Physical System Response (Gerçek odanın fiziksel değişimi)
        heat_loss = cooling_constant * (current_temp - ambient_temp)
        dT = (u - heat_loss) / thermal_inertia * dt
        current_temp += dT  
        
        # Store Data
        temps[i] = current_temp
        measured_temps[i] = measured_temp
        errors[i] = error
        control_signals[i] = u
        p_terms[i] = P
        i_terms[i] = I
        d_terms[i] = D
        
        previous_error = error
        
    return time_values, temps, measured_temps, control_signals, (p_terms, i_terms, d_terms)

# --- Run Simulation ---
times, temps, measured_temps, signals, pid_components = run_pid_simulation(
    Kp, Ki, Kd, target_temp, initial_temp, simulation_duration, noise_percent, anti_windup_on
)

# --- Layout & Visualization ---
col1, col2, col3, col4 = st.columns(4)
col1.metric("Final Temperature", f"{temps[-1]:.2f} °C", delta=f"{temps[-1]-target_temp:.2f} °C Diff")
col2.metric("Max Overshoot", f"{np.max(temps) - target_temp:.2f} °C")
col3.metric("Est. Settling Time", f"{simulation_duration}s") 
col4.metric("Mean Absolute Error", f"{np.mean(np.abs(temps - target_temp)):.2f}")

st.markdown("---")

chart_col, info_col = st.columns([3, 1])

with chart_col:
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)
    
    # Gerçek sıcaklık (Kalın kırmızı) ve Sensörün okuduğu sıcaklık (İnce gri)
    if noise_percent > 0:
        ax1.plot(times, measured_temps, label="Noisy Sensor Reading", color="gray", alpha=0.3, linewidth=1)
    
    ax1.plot(times, temps, label="Actual Temperature", color="#E63946", linewidth=2.5)
    ax1.axhline(target_temp, color="black", linestyle="--", label="Target (Set Point)", alpha=0.7)
    ax1.fill_between(times, temps, target_temp, alpha=0.1, color="#E63946")
    ax1.set_ylabel("Temperature (°C)")
    ax1.set_title("Temperature vs. Time Response")
    ax1.legend(loc="lower right")
    ax1.grid(True, linestyle="--", alpha=0.5)
    
    ax2.plot(times, signals, label="Heater Power Output (u)", color="#1D3557", linewidth=2)
    ax2.plot(times, pid_components[0], label="P-Term", linestyle=":", alpha=0.5)
    ax2.plot(times, pid_components[1], label="I-Term", linestyle=":", alpha=0.5)
    ax2.plot(times, pid_components[2], label="D-Term", linestyle=":", alpha=0.5)
    
    ax2.set_ylabel("Power Output (%)")
    ax2.set_xlabel("Time (seconds)")
    ax2.set_title("Controller Output & PID Components")
    ax2.legend(loc="upper right")
    ax2.grid(True, linestyle="--", alpha=0.5)
    
    st.pyplot(fig)

with info_col:
    st.info("""
    **How to read the graph?**
    
    * **Red Line:** Current room temperature.
    * **Dashed Line:** The target Set Point.
    * **Blue Line:** Power used by the controller.
    
    **Tuning Tips:**
    * **High Kp:** Reaches target fast but may be Overshoot.
    * **Add Ki:** Ensures the temp settles exactly at target.
    * **Add Kd:** Acts as a brake, dampening sudden changes.
    """)
    
    df = pd.DataFrame({"Time": times, "Actual Temp": temps, "Measured Temp": measured_temps, "Power": signals})
    csv = df.to_csv(index=False).encode('utf-8')
    st.download_button("Download Data (CSV)", csv, "pid_simulation_data.csv", "text/csv", key='download-csv')