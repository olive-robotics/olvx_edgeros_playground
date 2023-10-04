# Olive™ EdgeROS™ Playground: Python ROS 2 Examples

Welcome to the Olive™ EdgeROS™ Playground, where you can explore Python ROS 2 through a series of instructive and interactive examples designed specifically for the Olive™ EdgeROS™ platform.

## 🎯 Included Examples

### 🚥 GPIO (General-Purpose Input/Output)

#### 💡 User LED
- **Objective:** Control the onboard user LED to indicate status or notifications.
- **Use-Cases:** System status indication, user notifications.

#### 🔄 Reset Button
- **Objective:** Software control of system reset using a GPIO pin.
- **Use-Cases:** Implementing custom reset or shutdown routines.

### 🔢 Analog

#### 🔄 ADC (Analog to Digital Converter)
- **Objective:** Read analog signals through ADC and interpret in software.
- **Use-Cases:** Sensor data acquisition, analog signal processing.

#### 🌊 PWM (Pulse Width Modulation)
- **Objective:** Control digital signals to simulate analog signal properties.
- **Use-Cases:** Motor control, LED dimming.

### 🔄 Interface

#### 🚌 CAN (Controller Area Network)
- **Objective:** Implement CAN network communication for robust data transmission.
- **Use-Cases:** Vehicle communications, industrial automation.

#### 📟 Serial
- **Objective:** Achieve bidirectional communication over serial ports.
- **Use-Cases:** Data transmission to peripherals, debugging.

#### 🚊 I2C (Inter-Integrated Circuit)
- **Objective:** Facilitate communication with I2C devices.
- **Use-Cases:** Communicating with sensors, OLED displays, and other I2C peripherals.

## 📦 Dependencies 

Ensure smooth execution of examples by installing the necessary Python packages:

```shell
pip install pyserial
pip install smbus2

