# Autonomous Robot 
Welcome to the repository for our robot built for the **Robot Design and Competition 2024** at the Department of Electronic and Telecommunication Engineering, University of Moratuwa. This year's competition is inspired by the **Stranger Things** universe and takes place in the fictional town of **Hawkins**, where robots must complete a series of interdimensional tasks to prevent a catastrophic merging of dimensions.


---

## 🕹️ Tasks Overview

| Task No. | Description                                                                                     |
|----------|-------------------------------------------------------------------------------------------------|
| 01       | Line width detection and binary code decoding                                                   |
| 02       | Maze navigation and virtual box manipulation                                                    |
| 03       | Color path following (Blue or Red)                                                              |
| 04       | Dotted line (psychic trail) navigation                                                          |
| 05       | Portal timing and navigation (5s open/close)                                                    |
| 06       | Box arrangement by height depending on color followed                                           |
| 07       | Hidden task + small box insertion into a chamber                                                |
| 08       | Uneven terrain traversal and 2-rupee coin drop on magnetic spot to seal the rift                |

---

## 🗺️ Arena Specifications

| Attribute              | Value                                       |
|------------------------|---------------------------------------------|
| Size                   | 12 feet x 8 feet                            |
| Line Width             | 30 mm (black on white or vice versa)        |
| Surface                | Matte non-reflective                        |
| Zones                  | Line following, maze, portal, box zones     |
| Checkpoints            | Red and Blue squares                        |
| Boxes (W x D x H)      | 5cm x 5cm x [5cm, 10cm, 15cm]               |
| Terrain                | Stacked styrofoam squares for elevation     |

---

## 🤖 Robot Specifications

| Parameter             | Description                                              |
|-----------------------|----------------------------------------------------------|
| Operation             | Fully autonomous                                          |
| Size Limit            | Max 250 mm x 250 mm (No height restriction)              |
| Expansion             | Allowed without damaging the arena                       |
| Components Allowed    | Pre-made MCUs and sensors (No LEGO or wireless kits)     |
| Control               | On-board start via switch; No external communication     |
| Power Supply          | On-board (Max 24V DC)                                     |
| Stability             | Must stand independently at start                        |

---

## 🧪 Game Rules (Summary)

- Max time: **10 minutes**
- Up to **3 restarts** allowed
- **No code changes** after submission
- **No external inputs** or communication allowed
- If robot **fails line following** for 10s: must restart
- **Restart timer runs continuously**
- Violations (like arena damage, wireless control) lead to **disqualification**

---

## 🧠 Technologies Used

- Microcontroller: `Atmega2560`
- Programming: `C++`
- Sensors: IR array , color sensors, ultrasonic
- Actuators: DC motors, servos
- Power: 3 Li-ion rechargeable batteries
- Special Features: Coin drop mechanism, gate detection, line width detection

---
## System Overview and Circuitry


 <img src="https://github.com/user-attachments/assets/77102576-f819-42c8-9af8-3c651255f063" alt="Circuitry" width="500"/>

---

## 🧩 Task-by-Task Approach

Our robot is designed to autonomously complete each of the 8 tasks in the Hawkins Interdimensional Challenge using a combination of sensor fusion, mechanical actuation, and intelligent algorithms. Below is a task-wise breakdown of our approach.

### 🛠️ Mechanical Design

Our robot is compact and modular:
- **Base Size**: 24 cm × 18 cm
- **Height**: ~15 cm
- **Arm Mechanism**: Equipped with a custom-designed 2-DOF mechanical arm capable of gripping, lifting, and placing boxes. The arm performs actions like grabbing/dropping based on pre-programmed sequences.
- **Mobility**: Dual DC motor system with encoder feedback for precise movement and PID-based control.

### Initial Mechanical Design

 <img src="https://github.com/user-attachments/assets/921d879c-0284-4cc0-802f-2e4686128796" alt="Mechanical Design" width="500"/>


---

### 🔢 Task 1: Counting and Line Navigation

**Objective**: Read a barcode made of lines with two different widths and decode it into a binary number.

**Our Approach**:
- **IR Sensors** detect the difference between 3 cm (narrow) and 6 cm (wide) white lines.
- **Encoder-based PID control** ensures the robot moves straight during line scanning.
- A custom **state machine** captures the sequence of line widths and converts it into binary format, ignoring the last 3 lines as instructed.

---

### 📦 Task 2: Maze Navigation and Box Manipulation

**Objective**: Navigate a virtual maze, pick up a virtual box, and move it to the correct checkpoint.

**Our Approach**:
- Uses an **ultrasonic sensor (HC-SR04)** to detect presence/absence of walls and determine if a path is blocked.
- **Optimized maze-solving algorithm** identifies box position and calculates the shortest valid route.
- The robot indicates **box pickup** with a blue LED, restricts motion to forward/backward while carrying, and releases at junctions to change direction.

---

### 🔴🔵 Task 3: Color Line Following

**Objective**: Follow a Red or Blue line based on the previously selected checkpoint.

**Our Approach**:
- **Color sensor** discriminates between Red, Blue, and other colors.
- **IR sensor array** is used for following the detected colored path.
- Line-following is dynamically adjusted using PID for stability even on intersecting or broken lines.

---

### ⚪ Task 4: Dotted Line Navigation

**Objective**: Follow a discontinuous white dashed line.

**Our Approach**:
- **IR sensors + Encoder PID**: While on visible segments, IR-based line following is active.
- When the line disappears, robot switches to a **"go-straight-until-line"** mode using encoder PID until the next segment is detected.
- This cycle continues until the end of the dashed path is reached.

---

### 🚪 Task 5: Portal Navigation

**Objective**: Pass through a gate that opens and closes every 5 seconds.

**Our Approach**:
- **Proximity sensor** (e.g., IR or ultrasonic) checks the gate status.
- Once the gate is confirmed open, the robot **quickly navigates through** using a timed and pre-planned burst movement.
- Post-portal, the robot switches from **white line on black** to **black line on white** detection logic.

---

### 📏 Task 6: Box Arrangement

**Objective**: Arrange three boxes in ascending or descending order based on previously followed color (Blue = Ascending, Red = Descending).

**Our Approach**:
- The robot’s **mechanical arm** picks and places boxes using a **gripper** mechanism.
- **Proximity sensor mounted on the arm** is used to detect box height (5 cm, 10 cm, 15 cm).
- Sorting algorithm determines the pickup and placement sequence based on height and color context.

---

### 🕳️ Task 7: Hidden Task and Chamber Insertion

**Objective**: Pick a small box and insert it into a chamber after completing a hidden task.

**Our Approach**:
- Robot uses the **arm mechanism** to grab the box.
- Follows a line to reach the chamber.
- Hidden task logic is modular and adaptable – we use dynamic code modules to adjust during live coding.
- The box is aligned and **inserted into the chamber slot** with mechanical precision.

---

### 🪙 Task 8: Coin Drop and Task Completion

**Objective**: Navigate uneven terrain and drop a coin at the 'X' mark to light a bulb.

**Our Approach**:
- Robot navigates over **stacked Styrofoam tiles** using stable wheelbase and controlled speed.
- A **custom-designed coin dropper system** is integrated into the chassis.
- Once the robot detects the marked 'X' (via position tracking or visual cue), it **releases the coin** using a solenoid or servo trigger mechanism, successfully closing the circuit to light the bulb.

---

Each module in our robot—from motion control to manipulation—has been carefully integrated to ensure reliable, autonomous performance across all eight tasks. We focus on **robustness, adaptability, and precision** to complete the Hawkins Interdimensional Challenge.


## At the competition
[![Watch the video](https://github.com/user-attachments/assets/1a71a825-eff3-4f8e-857a-fd69ac0b17a5)](https://youtu.be/30hX-Og9dUU)




## 🧑‍💻 Live Coding Round

During the event, each team must participate in a **30-minute live coding session**. A region of the arena (Task 7) will be kept **secret** until then and revealed for this purpose. It is designed to test adaptability and real-time coding under pressure.


---
## Team Pixie Bots
<img src="https://github.com/user-attachments/assets/6cd1ae25-c625-4bb7-9760-ebe8e014f4e3" alt="Team Pixie Bots" width="300"/>


---
## 📂 Repository Structure

```bash
Autonomous-Robot/
├── hardware_design/         # CAD files, circuit diagrams
├── firmware/                # Arduino/ESP32 code
├── simulations/             # Arena simulation/test setups
├── documentation/           # PDFs, reports, research
├── images/                  # Photos, renders
└── README.md
