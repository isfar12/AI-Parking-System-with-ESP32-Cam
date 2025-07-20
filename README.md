# ESP32-CAM AI Parking System

A complete AI-powered parking management system using ESP32-CAM with Edge Impulse machine learning for car and payment detection, dual servo motor gates, IR sensor exit control, and real-time web streaming.

## 🚗 Features

- **AI Object Detection**: Car and coin recognition using Edge Impulse ML model
- **Dual Gate System**: Independent entry and exit gates with servo motors
- **Smart Payment System**: Entry requires both car AND coin detection
- **Parking Management**: Real-time slot tracking (0-4 slots)
- **IR Sensor Exit**: Automatic exit gate control for departing vehicles
- **Web Streaming**: Live camera feed via WiFi with RGB565 to JPEG conversion
- **OLED Display**: Real-time status and parking information
- **Conflict Prevention**: Gates operate independently with proper state management

## 🛠️ Hardware Requirements

### Main Components
- **ESP32-CAM AI-Thinker** (with PSRAM)
- **2x SG90 Servo Motors** (Entry & Exit Gates)
- **IR Sensor Module** (Exit detection)
- **0.96" OLED Display** (128x64, I2C)
- **Breadboard & Jumper Wires**
- **Power Supply** (5V recommended for servos)

### GPIO Pin Connections

| Component | GPIO Pin | Purpose |
|-----------|----------|---------|
| Entry Servo | GPIO 12 | Entry gate control (AI-controlled) |
| Exit Servo | GPIO 2 | Exit gate control (IR-controlled) |
| IR Sensor | GPIO 13 | Car exit detection |
| OLED SDA | GPIO 14 | I2C data line |
| OLED SCL | GPIO 15 | I2C clock line |

## 📋 System Logic

### Entry Gate (GPIO 12)
- **Trigger**: Car AND Coin detected by AI (60% confidence)
- **Condition**: Available parking slots > 0
- **Action**: Opens for 5 seconds, decreases slot count
- **Status**: "ENTRY GATE: Opened/Closed"

### Exit Gate (GPIO 2)  
- **Trigger**: IR sensor detects object
- **Condition**: Parking not empty (slots < 4)
- **Action**: Opens for 5 seconds, increases slot count
- **Status**: "EXIT GATE: Opened/Closed"

### Parking Management
- **Maximum Slots**: 4 vehicles
- **Entry Blocked**: When slots = 0 (parking full)
- **Exit Blocked**: When slots = 4 (parking empty)
- **Real-time Display**: Current availability on OLED

## 🔧 Installation & Setup

### 1. Hardware Assembly
```
ESP32-CAM Connections:
├── Entry Servo → GPIO 12
├── Exit Servo → GPIO 2  
├── IR Sensor → GPIO 13
├── OLED SDA → GPIO 14
├── OLED SCL → GPIO 15
└── Power: 5V for servos, 3.3V for sensors
```

### 2. Software Requirements
- **Arduino IDE** with ESP32 board package
- **Edge Impulse Arduino Library**: `isfar12-project-1_inferencing`
- **Adafruit SSD1306**: OLED display library
- **Adafruit GFX**: Graphics library
- **WiFi & WebServer**: ESP32 built-in libraries

### 3. Library Installation
```bash
# Arduino Library Manager
- Adafruit SSD1306
- Adafruit GFX Library
- ESP32 Board Package (v2.0.4+)

# Edge Impulse Model
- Download your trained model library
- Replace "isfar12-project-1_inferencing.h" with your model
```

### 4. Configuration
```cpp
// WiFi Settings (lines 87-88)
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Parking Configuration (line 154)
#define MAX_PARKING_SLOTS 4  // Adjust as needed

// Servo Angles (lines 139-140)
#define SERVO_BASE_ANGLE 0    // Closed position
#define SERVO_ACTIVE_ANGLE 90 // Open position
```

## 🖥️ Web Interface

### Access Points
- **Live Stream**: `http://[ESP32_IP]/`
- **Single Capture**: `http://[ESP32_IP]/capture`
- **Auto-refresh**: 1-second intervals

### Features
- Real-time camera feed
- RGB565 to JPEG conversion
- Mobile-friendly responsive design
- Error handling for camera failures

## 📊 AI Model Training

### Edge Impulse Setup
1. **Data Collection**: Capture images of cars and coins
2. **Labeling**: Create bounding boxes for "car" and "coin" classes
3. **Training**: Use object detection with 60% confidence threshold
4. **Export**: Download Arduino library and integrate

### Detection Logic
```cpp
// Confidence threshold: 60%
if (bb.value > 0.6) {
    if (strcmp(bb.label, "car") == 0) car_detected = true;
    if (strcmp(bb.label, "coin") == 0) coin_detected = true;
}
```

## 🔍 Troubleshooting

### Common Issues

| Problem | Solution |
|---------|----------|
| Camera not initializing | Check power supply and GPIO connections |
| Servo not moving | Verify 5V power supply and GPIO pins |
| WiFi connection fails | Update SSID/password, check signal strength |
| OLED display blank | Check I2C connections (SDA/SCL) |
| False AI detections | Retrain model with more diverse data |

### Debug Serial Output
```
ENTRY GATE: Opened (AI detected car + coin)
ENTRY GATE: Closed (5 seconds elapsed)
EXIT GATE: Opened (IR sensor detected object)
EXIT GATE: Closed (5 seconds elapsed)
PARKING STATUS: 3/4 slots available
```

### Memory Management
- RGB565 format for better performance
- Single frame buffer to conserve memory
- Automatic garbage collection for JPEG conversion

## 📈 System Monitoring

### OLED Display States
- **AI PARKING**: Idle state with slot availability
- **CAR ENTRY**: Entry gate activated
- **CAR EXIT**: Exit gate activated  
- **CAR FOUND**: Car detected (needs payment)
- **COIN FOUND**: Payment detected (needs car)
- **PARKING FULL**: No entry allowed

### Performance Metrics
- **AI Processing**: ~100-200ms per inference
- **Servo Response**: <1 second gate operation
- **Web Streaming**: 1 FPS auto-refresh
- **Memory Usage**: Optimized for ESP32-CAM PSRAM

## 🚀 Advanced Features

### Extensibility Options
- **Database Integration**: Log entry/exit events
- **Mobile App**: Remote monitoring and control
- **Payment Gateway**: Digital payment processing
- **License Plate Recognition**: Vehicle identification
- **Multi-location Management**: Scale to multiple parking areas

### Code Structure
```
esp32_camera.ino
├── AI Detection Logic (Lines 350-420)
├── Servo Control System (Lines 710-810)
├── IR Sensor Management (Lines 820-860)
├── Web Server Handlers (Lines 920-1000)
└── OLED Display Functions (Lines 600-680)
```

## 📄 License

This project is based on Edge Impulse Arduino examples with custom parking system implementation.

---

## 🏆 Project Success Metrics

✅ **Dual servo gates operating independently**  
✅ **AI detection with 60% confidence threshold**  
✅ **Real-time parking slot management (0-4)**  
✅ **Web streaming with RGB565-JPEG conversion**  
✅ **IR sensor exit control with debouncing**  
✅ **OLED display with status information**  
✅ **Conflict-free gate operation**  
✅ **Memory-optimized performance**  

**Ready for deployment in real-world parking scenarios!** 🎉
