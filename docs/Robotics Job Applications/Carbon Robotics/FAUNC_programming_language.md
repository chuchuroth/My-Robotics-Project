
# What is a Robot and Its Purpose

## Overview
Robots serve a wide range of applications, enhancing automation across industries.

- **Applications**:
  - Material Handling
  - Palletizing
  - Pick and Place
  - Material Removal (e.g., grinding, painting)
  - Welding

- **Post-Epidemic Relevance**:
  - Reliable operation
  - No sick leave
  - Maintains cleanliness
  - Produces high-quality parts

### Importance of Learning Robotics
- Positions you at the forefront of future development.
- **Industries Using Robots**:
  - RV Industry
  - Automotive Industry
  - Pharmaceutical Industry (with camera integration)
- **Future Potential**: Don’t limit to current applications; new uses will emerge.

### Key Application Areas
- **Material Handling**: Largest application.
- Followed by Palletizing, Material Removal, and Welding.

## Robot Space

- **Maximum Space**: Furthest reach of the robot arm when fully extended, even if operations aren’t feasible there (e.g., "lying down" extended).
- **Restricted Space**: Safety/practical area, limited by protective measures within maximum space.
- **Operating Space**: Actual working area within restricted space.
- **Maximum Envelope**: All areas swept by moving parts during motion.
- **Operating Envelope**: Focused work area, designed to reduce cycle time.

## Safety

### First Principles
- **Top Priority**: Operator safety—robots and parts are replaceable, people are not.
- **Safety Priority Order**:
  1. Operator
  2. Robot
  3. External Equipment (e.g., signal lights, fixtures, controllers)
  4. Tooling
  5. Workpiece

### Safety Features
- **Teach Pendant**:
  - Emergency Stop (E-stop) button: Stops all movement in emergencies.
- **Dead Man Switch**: Three positions:
  - Fully released: No movement
  - Fully pressed: Emergency stop
  - Middle: Allows movement (required for manual operation)

### Common Safety Measures
- Light Curtains: Stop on entry
- Safety Fences
- Pressure Mats: Triggered when stepped on
- Interlocks: E.g., prevents startup if door is open
- Warning Lights
- Motion Limits: Physical screws or software settings

### Collaborative Robots
- Equipped with pressure, speed, and safety sensors.
- Stop automatically on collision.

### Work Area Safety Precautions
- Carry teach pendant or E-stop device (preferably pendant).
- Safety doors/guardrails with interlocks.
- Follow Robotics Industries Association (RIA) guidelines ([RIA website](https://www.robotics.org)).
- **Prohibited**: Entering open robot work envelope.
- **Educational Note**: Small robots can injure; never start in Auto mode without teacher approval.
- **Auto Mode**: Close safety door before operation.
- **Manual/Teach Mode**: Exercise caution, limit time in work cell.

### Maintenance Safety
- **Lock Out Tag Out**: Ensures safety during maintenance (e.g., servo motor replacement), prevents accidental startup.

### Programming Safety
- Use handshaking signals with external equipment.
- Monitor override speed, visualize motion trajectories.
- Keep work area clean; ensure circuit protection.
- **Anti-Tie Down Logic**: Prevents bypassing safety (e.g., no heavy objects on pressure sensors).
- **Advanced Features**: Home Position, Interference Zones enhance safety.

## Axes and Motion

### Industrial Robot Axes
- **Six Axes**: Provide six degrees of freedom for flexibility.
- **Major Axes**:
  - **J1**: Base rotation (hip, torsional motion)
  - **J2**: Arm bending (waist forward)
  - **J3**: Shoulder up/down
- **Minor Axes**: Fine movements
  - **J4**: Wrist twist (torsional)
  - **J5**: Wrist bending/swinging
  - **J6**: End flange rotation (tool posture)
- **Optional**: J7 (gantry/turntable, not covered here).

### Motion Characteristics
- Each axis moves independently or coordinates with others.
- **Axis Limits**: Programmable, expressed in angles (e.g., 360°).
  - Manufacturer presets; adjustable with caution (restart may be needed).

### FANUC Dual Check Safety (DCS)
- Monitors speed and position.
- Features: Position checks, speed checks, Cartesian speed checks.
- Stops robot if exiting safety area.

## Controller

- **Role**: Brain of the robot, runs software.
- **Components**: Power supply, transformer, memory, I/O board.
- **Types**: A-type (used here), B-type, Open Air Mate, Mate.
- **Operating Panel**:
  - Fault reset button
  - Teach/Run mode switch
  - Start button
  - Fault indicator light
- **Modes**:
  - Local: Testing/teaching
  - Remote: Production (PLC-controlled)
- **User Buttons**: Customizable for macros.

## Software

- Pre-installed in controller; back up regularly.
- **Operating Modes**:
  - **T1 Mode**: Max 250 mm/s, for safe teaching.
  - **T2 Mode**: Test run, up to 100% speed, teach mode.
  - **Auto Mode**: Full speed; safety breach (Class 1 Stop) cuts servo power.

### Mode-Specific Safety
- Auto: Safety fence breach stops robot.
- T1/T2: Safety measures may be bypassed (operator inside), requiring extra caution.

## Robot System Components

### Mechanical Unit
- Driven by AC Servo Motors with serial pulse counters.
- **Mechanical Brake**: Released when powered on, engaged when off (prevents gravity fall).
- **Maintenance**: Lubricate joints regularly.
- **Rotary Pulse Encoder**: Inside servo motor.
- **Home Position**: Preset reference; user-adjustable (factory Absolute Zero: arm extended upright).
- **Batteries**: Four C-type, maintain encoder memory when powered off.

### Controller
- Central control unit.

### Peripheral Equipment
- Teach Pendant
- PLC
- End of Arm Tooling
- HMI
- Cameras/Vision Systems

## Teach Pendant

- **Purpose**: Main programming tool, like a game controller.
- **Process**: Jog to position, store with button press.
- **Safety Features**: E-stop, Dead Man Switch.
- **Buttons**:
  - Shift: Blue-labeled functions
  - Menu: Settings/configurations
  - Cursor: Shift + keys for quick jumps
  - Step Mode: Debug step-by-step (recommended for first tests)
  - Reset: Clear alarms
  - Common: Previous, Enter
  - Soft Keys: Context-dependent
  - Position, I/O: View data
  - Forward/Backward: Navigate program
- **Design**: Touchscreen, supports mouse/keyboard.
- **Jogging**: Shift + arrow keys.

## Manual Operation (Jogging)

- **Jog Speed**: Override % (e.g., 50% of 250 mm/s = 125 mm/s), includes Fine Speed (1 pulse).
- **Joint Mode**: Shift + J1-J6, for unjamming/reconfiguration.
- **Coordinate Systems** (switch via COORD button):
  - Joint: Independent joint movement
  - World: XYZ Cartesian, relative to base origin (J1-J2 intersection)
  - Tool: Relative to TCP
  - User: Custom-defined
  - Jog Frame
- **Right-Hand Rule**: Determines XYZ directions in World system.

## Singularity

- **Cause**: Axes align (e.g., J4 and J6 in World system), confusing TCP calculation.
- **Symptoms**: MOTN-230 error, no movement.
- **Fix**:
  1. Switch to Joint system.
  2. Jog J5 ±5-15°.
  3. Return to World system.

## Error Types and Causes

- **Hardware**: Dead battery, switch failure, broken cable, tool disconnection (diagnosed by FANUC systems).
- **Software**: Programming errors (e.g., arc motion mistakes).
- **External**: Safety triggers (e.g., E-stop, interlock from door open).

### Clearing Errors
- **Display**: Error messages (e.g., DCS) on teach pendant.
- **Reset**: Clear with Reset button or Shift + F4 (all errors).
- **DCS Errors**:
  1. Release buttons.
  2. Middle Dead Man Switch.
  3. Hold Shift, press Reset.
  4. Jog out of danger manually.
- **Alarm Codes**: Four-digit prefix + three-digit code (view via Menu > Alarm).

## Program Management

- **Abort Programs**: Function menu > Abort All, ensures clean state.
- **Paused Programs**: Cannot be deleted until aborted.
- **Note**: Abort doesn’t move robot unless commanded.
- **Error Example**: INF i i n t f 105 (program running).
- 
---
# Comprehensive Guide to FANUC Robotics

## Various Applications of Robots

- **Material Handling**: Largest application area.
- **Palletizing**
- **Picking and Placing**
- **Material Removal**: Examples include grinding, spray painting.
- **Welding**
- **Popular Industries**:
  - Automotive Industry
  - Pharmaceutical Industry
  - RV Industry
  - Others

### Importance in the Post-Epidemic Era
- Automation
- Reliability
- No sick leave
- Maintains cleanliness
- Improves part quality

### Discussion on Jobs
- Do robots replace or create jobs?
- Future potential: Don’t limit to known applications; new uses will emerge.

## Robot Terminology

- **Maximum Space**: Furthest reach of all robot parts, even if operation isn’t possible there.
- **Restricted Space**: Area limited for safety or practical reasons (via devices/software).
- **Operating Space**: Where the robot actually performs tasks.

## Robot Safety - First Principles

- **Core Principle**: Operator safety is paramount; robots/parts are replaceable, people are not.
- **Safety Priority**:
  1. Operator
  2. Robot
  3. External Devices (e.g., signal lights, fixtures, controller)
  4. Tooling (end-of-arm tools)
  5. Workpiece (object handled)

### Safety Devices and Measures
- **Teach Pendant**: Includes Emergency Stop (E-stop) button.
- **Dead Man Switch**: Three states:
  - Fully released: Stop
  - Fully pressed: Stop
  - Middle position: Allows movement
- **Light Curtains**: Stop robot when work area is entered.
- **Safety Fences**
- **Pressure Mats**: Detect entry into danger zones.
- **Interlocks**: E.g., dual-key start requirement.
- **Warning Lights**
- **Motion Limits**: Physical switches or software settings.

### Collaborative Robots (Cobots)
- Equipped with:
  - Pressure sensors
  - Safety sensors
  - Speed sensors
- Auto-stop on collision with a person.

### Safety Precautions for Work Area
- Always carry teach pendant or E-stop device.
- Safety doors/guardrails with interlocks.
- Follow Robotics Industries Association (RIA) guidelines.
- **Maximum Envelope**: Entire potential motion range.
- **Operating Envelope**: Actual work area.
- **Anti-Tie Down Logic**: Prevents bypassing safety mechanisms.

## Axis Limits
- **Range**: Programmable, expressed in angles (e.g., ±180°).
- **Adjustments**: May require robot restart.
- **Types**:
  - Hard Stops: Physical limits (e.g., Axis 5 prevents self-bending).
  - Limit Switches: Electrical detection.
  - Software Limits: Axes 4 and 6.

## FANUC Dual Check Safety (DCS)
- **Purpose**: Additional safety layer.
- **Functions**:
  - Position check
  - Speed check
  - Cartesian coordinate speed check
- **Action**: Stops robot if entering unsafe areas or moving too fast.
- **Features**: Defines restricted zones; stops on exit.

## Important Safety Precautions
- No loose clothing/jewelry.
- Visually inspect robot and work cell.
- Never enter powered robot envelope.
- In education: No Auto mode without permission.
- Close safety door in Auto mode.
- Caution in manual/teach mode; limit time in work area.
- **Lock Out Tag Out**: For maintenance (e.g., servo motor replacement).
  - Release residual energy (flywheels, capacitors, springs).

## Programming Considerations
- Ensure safety via programming.
- **Handshaking**: Sync with external devices.
- **Motion Limits**: Protect equipment/personnel.
- Monitor override speed, visualize trajectories.
- Keep work area clean; ensure circuit protection.
- **Advanced Concepts**: Home Position, Interference Zones.

## General Robot System Components
- **Mechanical Unit**: Robot body.
- **Controller**: Runs software, the robot’s brain.
- **Peripheral Equipment**: E.g., Teach Pendant.

### Teach Pendant
- **Purpose**: Programs robot, akin to a game controller.
- **Method**: Jog to position, store with button press.
- **Features**:
  - E-stop button
  - Dead Man Switch
  - Shift (blue functions), Menu, Cursor Keys
  - Step Mode (line-by-line execution)
  - Reset (clear alarms), Enter, Previous, Soft Keys
  - Position, I/O buttons
- **Design**: Modern touchscreens, supports mouse/keyboard.
- **Jogging**: Shift + J1-J6 keys.

### Mechanical Unit and Servo Motors
- **AC Servo Motors**:
  - Drive motion with serial pulse counters for position.
  - Mechanical Brake: On when off, off when on.
  - Drift risk if brake fails without power.
- **Maintenance**: Lubricate joints (not motors).
- **Home Position**: User-set; Absolute Zero for calibration.
- **Batteries**: C-type, maintain encoder memory.

### Robot Axes
- **Major Axes**:
  - J1: Base rotation (torsion)
  - J2: Arm bending
  - J3: Shoulder up/down
- **Minor Axes**:
  - J4: Wrist torsion
  - J5: Wrist pitch
  - J6: Flange roll
- **Optional**: J7 (gantry/turntable, not covered).
- **Spider Robots**: 3 axes for high-speed pick/place.
- **Motion**: Independent or coordinated.

### Axis Motion Range and Limits
- Limited by hard stops, limit switches, software (Axes 4, 6).
- Axis 5: Hard stops prevent over-bending.

### Controller
- **Components**: Power supply, transformer, memory, I/O board.
- **Types**: A, B, Open Air Mate, Mate.
- **Operating Panel**:
  - Fault Reset
  - Teach/Run switch
  - Go (Start) button
  - Fault LED
- **Modes**: Local (test), Remote (production via PLC).

### Software
- Pre-installed; back up with images.
- **Modes**:
  - T1: 250 mm/s max
  - T2: Test, up to 100% speed
  - Auto: Full speed, safety door stops
- Safety door behavior varies by mode.

### Peripheral Equipment
- PLC
- End-of-Arm Tooling
- HMI
- Cameras/Vision Systems

## More Details on Teach Pendant
- Middle Dead Man Switch position enables movement.
- Shift + Cursor: Quick menu skip.
- Step Mode: Line-by-line execution.
- Key Buttons: Previous, Reset, Enter.
- Soft Keys: Context-dependent.
- Position/I/O: Display current status.

## Starting the Robot
- Via switch.

## Jog Speed
- **Override Speed**: % of max (e.g., 50% of 250 mm/s = 125 mm/s).
- **Options**: Includes Fine Speed (1 pulse).

## Manual Teaching (Jogging)
- **Joint Mode**: Shift + J1-J6, for jams/reconfiguration.
- **World Mode**: TCP moves in XYZ straight lines.
- **Tool Mode**: TCP relative to tool system.
- **User Mode**: TCP relative to custom system.
- **Jog Frame Mode**: Less common.
- Switch via Chord button.

## Cartesian Coordinates
- Used in World, Tool, User modes.
- Origin at J1-J2 intersection.
- Right Hand Rule for XYZ directions.

## Singularity
- **Cause**: Axes align, affecting motion.
- **Fix**: Joint mode, adjust J5 (±10-15°).

## FANUC Robot Programming Basics

### Creating and Editing Jobs
- **Create**: `SELECT` > `CREATE`, name (no numbers/spaces).
- **Details**: Add comments, set `Group Mask` (1 for motion).
- **Edit**: Use `EDIT`, add via `INSTRUCTION`.

### Recording and Calling Positions
- **Position Registers**: `DATA` > `POSITION REGISTERS`.
- Record: `SHIFT` + `RECORD`.
- Types: Cartesian (XYZ WPR), Joint angles.
- Call: `INSTRUCTION` > `MOTION` > `JOINT`/`LINEAR`.

### Data Registers
- Store integers; view/modify via `DATA`.
- Manipulate with `ADD`.

### Motion Types
- **Joint**: Fast, undefined path, % speed.
- **Linear**: Straight path, mm/s speed.
- **Circular/Arc**: 3-point arc (start, mid, end), for welding.

### Speed
- Joint/Circular: %.
- Linear: mm/s.
- Adjust via teach pendant.

### Termination Type
- **Fine**: Precise stop.
- **Continuous**: Smooth pass-through.

### Common Instructions
- **Motion**: `JOINT`, `LINEAR`, `CIRCLE`.
- **I/O**: `DO` controls devices; view via `I/O`.
- **Register**: `R[] = R[] + Number`.
- **Branching**: `JMP LBL`, `LBL`.
- **Conditional**: `IF`, `SELECT`.
- **Wait**: `WAIT DI[] = ON`, with timeout.
- **Payload**: `PAYLOAD [Number]`.

### User and Tool Frames
- **User Frame**: Workpiece reference, 3-point setup (`MENU` > `SETUP` > `FRAMES`).
- **Tool Frame**: TCP reference, 3-point or offset.
- Switch: `SHIFT` + `COORD`.

### Program Adjust
- Fine-tune via `MENU` > `SETUP` > `ADJUST`.

### Reference Positions
- E.g., Home Position, callable in program.

### I/O (Input/Output)
- **Digital I/O**: For PLC, sensors; `DO[]`, `DI[]`.
- **Robot I/O**: End effector devices.
- **Ethernet I/O**: Requires option, uses UOP signals.

### Program Flow Control
- **Jump**: `LBL`, `JMP LBL`.
- **Conditional**: `IF`, `SELECT`.
- **Wait**: Pauses until condition met.
- **Looping**: `FOR`/`ENDFOR`, `JMP LBL`.

## Roboguide Software

### Installation and Setup
- Large file; select robot model/version.
- Create/import cell from backup.

### Creating a Cell
- New: `File` > `New Cell`.
- From Backup: `MENU` > `FILE` > `BACKUP EVERYTHING`.

### Importing Tools
- Adjust CAD gripper models in `IOTool Properties`.

### Teach Pendant Simulation
- Virtual pendant; physical option with `ROBOGUIDE MODE`.

### Coordinate System Navigation
- Jog via `SHIFT` + axis buttons; switch via `COORD`.

## Advanced Topics

### Payload
- Set via `MENU` > `SYSTEM` > `MOTION` > `PAYLOAD`.

### Machine Vision
- Components: Light sources, cameras, filters.
- Key for detection/measurement.

### FANUC UOP with Ethernet I/O
- Maps signals (start, stop) to PLC (e.g., CompactLogix).

### Coordinate Mask Setup
- `$SCR_GRP.$COORD_MASK`: Controls jogging coordinate options.

---
# FANUC Robot Programming Guide

## 1. Creating and Editing Programs

- **Creating a Program**:
  - Access: `SELECT` menu > `CREATE`.
  - Naming: Use `KEYBOARD` in `OPTION` menu.
    - **Rules**: No starting numbers, no spaces.
- **Editing**:
  - Enter: Click `EDIT` to access editing interface.
  - Add Instructions: Via `INSTRUCTION` menu (F1 key on teach pendant).
    - Types: Motion, logic control, I/O.
- **Editing Operations**:
  - Use `EDIT COMMAND` menu: `COPY`, `PASTE`, `REPLACE`, `INSERT`, `DELETE`, `REMARK`.
  - **Replace Note**: Applies from cursor line downward only.

## 2. Motion Types

FANUC robots support multiple motion types:

- **Joint Motion**:
  - Fastest movement; joints move to target angles at max speed.
  - Path: Undefined.
  - Program: Select `JOINT` motion type.
  - Use: Quick positioning, transitions between teach points.
  - Recording: Joint angles; recommended for `HOME` (origin) position (independent of tool/user frames).
- **Linear Motion**:
  - TCP moves in a straight line at constant speed (mm/sec).
  - Program: Select `LINEAR` motion type.
  - Use: Precise paths (e.g., welding, spraying, material handling).
- **Circular Motion**:
  - TCP follows a circular arc.
  - Structure: One linear segment + two arc segments.
  - Program: Change to `CIRCLE` or `CIRCULAR ARC`:
    - `CIRCLE` (C command): Define two points on circle.
    - `CIRCULAR ARC` (A command): Define start, intermediate, end points for free curves.
  - Use: Irregular paths (e.g., welding, spraying).

## 3. Recording Positions

- **Method**: Press `SHIFT` + `POINT` (or `TEACH`) keys together.
- **Position Representation Options**:
  - **Cartesian Coordinates (XYZWPR)**:
    - Based on active `User Frame` and `Tool Frame`.
    - **Caution**: Issues if run under different user coordinate system.
  - **Joint Coordinates (JOINT)**:
    - Based on joint angles; versatile, independent of tool/user frames.
  - **Position Registers (PR)**:
    - Store position data; access via `DATA` > `Position Registers`.
    - Naming: For easy identification.
    - Recording: `SHIFT` + `RECORD` at target position.
    - Calling: Use in program to reference stored positions.

## 4. Coordinate Systems (Frames)

Coordinate systems are key for precision:

- **World Frame**:
  - Robot’s reference system, centered at base.
  - Basis for all other frames.
- **Tool Frame (UTOOL)**:
  - Defines TCP position/orientation relative to J6 flange.
  - **Importance**: Determines motion trajectory.
  - Setup: Three Point Method or Direct Entry; in Roboguide, edit tool properties.
  - Default: Flange center.
  - Switch: `SHIFT` + `COORD`.
- **User Frame (UFRAME)**:
  - Defines workpiece/fixture position in World Frame.
  - **Benefit**: Program via workpiece coords; re-teach only UFRAME if workpiece moves.
  - Setup: Three Point Method; in Roboguide, use Fixture settings.
  - Switch: `SHIFT` + `COORD`.

## 5. Basic Commands

- **Position Registers**: `PR[n] = P[m]` assigns position `P[m]` to `PR[n]`.
- **Data Registers (R)**:
  - Store integers/floating-point numbers; access via `DATA` > `Data Registers`.
  - Use: Counting, logic (e.g., `R = R + 1` adds 1).
- **Input/Output (I/O)**:
  - **Digital I/O**:
    - `DO[n] = ON/OFF`: Controls external devices.
    - `IF DI[m] = ON THEN ...`: Checks input status.
  - **Robot I/O (RI)**: Controls end effector (e.g., gripper, sensors).
- **Calling Programs**: `CALL program_name` invokes subroutines.
- **Jumps and Labels**:
  - `LBL[n]`: Defines label.
  - `JMP LBL[n]`: Unconditional jump.
- **Conditional Branching**:
  - `IF condition THEN ...`: E.g., register/I/O checks.
  - `SELECT R[n] = constant THEN CALL program_name`: Multi-case branching.
- **Wait**:
  - `WAIT condition`: Pauses until true (e.g., `DI[m] = ON`).
  - `WAIT time (sec)`: Time-based pause.
- **Payload**: `PAYLOAD[n]` sets end effector/workpiece weight for accuracy/safety.
- **FOR Loop**: `FOR R[n] = start TO end BY step` with `ENDFOR` for repetition.

## 6. Looping

- **Basic Loops**: Use `IF` + `JMP`.
- **Structured Loops**: `FOR/ENDFOR` for repeated code blocks.

## 7. Macros

- **Purpose**: Small background programs for logic/motion.
- **Setup**: `SETUP` > `Macro`; assign to teach pendant buttons.
- **Note**: Non-motion macros need `Group Mask` set to ``.

## 8. Conditional Monitoring

- **Function**: Monitors conditions (e.g., input signals) during main program.
- **Setup**: `MONITOR` and `MONITOR END` in `INSTRUCTION` menu.
- **Action**: Triggers non-motion jobs (e.g., user alarms) when conditions met.

## 9. Menu Utility

- **Purpose**: Operator interaction via teach pendant.
- **Types**: Prompt Boxes, Yes/No Select, List Select.
- **Setup**: `SETUP` > `Menu Utility`.
- **Use**: Call via `INSTRUCTION` > `Macro`; store selections in data registers for branching.

## 10. Offsets

- **Purpose**: Adjust positions without re-teaching.
- **Method**: Use Position Offset Registers to modify existing positions.

## 11. Program Checks and Production Setup

- **Checks**: `MENU` > `SETUP` > `Program Select`:
  - Verify `HOME` position, speed override.
- **System Config**: `MENU` > `SYSTEM` > `Config`:
  - Power failure recovery, auto-start, forced I/O in Auto mode.

## 12. Program Adjust and Reference Positions

- **Program Adjust**: `MENU` > `SETUP` > `Program Adjust`.
  - Fine-tunes trajectories without re-teaching.
- **Reference Positions**: E.g., `HOME`; set via `MENU` > `SETUP` > `Reference Position`.

## 13. User Alarms

- **Purpose**: Custom alarms/actions.
- **Setup**: Non-motion macros; trigger via `INSTRUCTION` > `MISCELLANEOUS` > `UAlarm`.

## 14. Roboguide (Simulation Software)

- **Purpose**: Offline simulation, testing, optimization.
- **Features**: Collision detection, cycle time analysis.
- **Setup**: Create workcell from robot backup; edit tool/user frames visually.

## 15. Navigation and Basic Operations

- **Teach Pendant Menus**:
  - `MENU`: Access functions/settings.
  - `SELECT`: Manage programs.
  - `EDIT`: Edit current program.
  - `DATA`: Edit registers/positions.
  - `I/O`: Monitor/control signals.
  - `SYSTEM`: Configure robot.
  - `ALARM`: View alarm history.
  - `POSITION`: Check current position.
- **Modes**:
  - **TEACH (T1/T2)**: Manual operation; requires Deadman Switch.
  - **AUTO**: Full program run; starts via Cycle Start.
  - **STEP**: Debug step-by-step.
  - **CONTINUOUS**: Full run.
- **Coord Switching**: `COORD` key (Joint, World, Tool, User).

## 16. Error Codes

- **Example**: `MOTN-017` + three-digit code (axis/error type, e.g., motion limit).
- **Purpose**: Diagnose issues via error codes.

## 17. UOP Communication with Allen Bradley CompactLogix via Ethernet/IP

- **Setup**:
  - Configure robot IP and `MENU` > `I/O` > `Ethernet IP`.
  - Match settings on PLC (e.g., CompactLogix).
- **UOP Signals**: Control start, stop, emergency stop; monitor status.


---



Based on your notes on programming FANUC robots, I’ll teach you a few practical examples of how to program a FANUC robot in a real factory setting. These examples are designed to reflect common industrial applications and will walk you through the steps using the teach pendant, a key tool for programming FANUC robots. Let’s dive into three examples: a pick and place operation, a welding application, and a palletizing task.

---

### Example 1: **Pick and Place Operation**
In a factory, FANUC robots are frequently used for pick and place tasks, such as moving parts from a conveyor belt to a processing machine on an assembly line. This is a fundamental application that demonstrates point-to-point movement programming.

#### Steps to Program:
1. **Move to the Pick-Up Position**:  
   - Using the teach pendant, manually jog the robot to the position above the conveyor where it will pick up the part. Your notes mention different coordinate systems (e.g., joint, world, tool); for simplicity, use the **world coordinate system** (press the `COORD` button to switch) to move relative to the robot’s base.
   - Press the `Shift` key and use the jog buttons (e.g., `+X`, `-X`, `+Y`, etc.) to position the robot’s tool center point (TCP) accurately.

2. **Record the Pick-Up Position**:  
   - Once aligned, press the `Position` button (or equivalent) and record this as, say, Position 1 (P[1]). This saves the coordinates in the robot’s memory.

3. **Move to the Place Position**:  
   - Jog the robot to the location where the part will be placed (e.g., inside the machine). Adjust the height and orientation as needed.

4. **Record the Place Position**:  
   - Record this as Position 2 (P[2]) using the same method.

5. **Define Motion Type**:  
   - In the program, specify the motion between positions. Use **linear motion** (`L`) for a straight path if precision is key, or **joint motion** (`J`) for a faster, curved path if speed matters more. For example:
     ```
     1: J P[1] 100% FINE
     2: L P[2] 500mm/sec FINE
     ```
     Here, `100%` is the speed for joint motion, and `500mm/sec` is for linear motion.

6. **Add Gripper Commands**:  
   - Integrate commands to control the gripper (e.g., open/close). Assuming digital I/O signals:
     ```
     1: J P[1] 100% FINE
     2: DO[1]=ON   // Close gripper to pick
     3: WAIT 0.5sec
     4: L P[2] 500mm/sec FINE
     5: DO[1]=OFF  // Open gripper to place
     ```

7. **Test the Program**:  
   - Run the program in **step mode** (press the `Step` button) to execute one line at a time. Check that the robot moves correctly and the gripper operates as expected. If an error occurs (e.g., a safety fault), press the `Reset` button to clear it, as noted in your document.

#### Why This Works:
Pick and place is a staple in factories because it automates repetitive tasks, improving efficiency. This example uses basic position recording and motion commands, making it a great starting point for FANUC programming.

---

### Example 2: **Welding Application**
FANUC robots are widely used for welding, especially in automotive manufacturing, where they follow precise paths to join parts. This example focuses on programming a welding path.

#### Steps to Program:
1. **Define the Welding Path**:  
   - Jog the robot to multiple points along the welding seam (e.g., start, middle, end). Use the **tool coordinate system** (switch via `COORD`) to align the welding torch relative to its TCP.
   - Record each point, e.g., P[1] (start), P[2] (middle), P[3] (end).

2. **Use Linear Motion**:  
   - Program the robot to move linearly between points for a smooth weld. Set a consistent speed (e.g., 50mm/sec) based on the welding process:
     ```
     1: L P[1] 50mm/sec FINE
     2: L P[2] 50mm/sec FINE
     3: L P[3] 50mm/sec FINE
     ```

3. **Integrate Welding Equipment**:  
   - Add commands to control the welding arc (e.g., via I/O signals):
     ```
     1: L P[1] 50mm/sec FINE
     2: DO[2]=ON   // Start welding arc
     3: L P[2] 50mm/sec FINE
     4: L P[3] 50mm/sec FINE
     5: DO[2]=OFF  // Stop welding arc
     ```

4. **Fine-Tune the Path**:  
   - Test the program in step mode. If the torch deviates from the seam, adjust positions using the teach pendant. Your notes mention singularities (e.g., when joints align, like J4 and J6); if the robot hesitates, switch to **joint coordinates**, jog slightly (e.g., ±10° on J5), then revert to world or tool coordinates.

5. **Set Speed and Acceleration**:  
   - Adjust the speed (e.g., 50mm/sec) and acceleration to match the material thickness and welding requirements, ensuring a consistent bead.

#### Why This Works:
Welding requires precision and smooth motion, which linear interpolation provides. This example also shows how to integrate external equipment and handle potential issues like singularities, common in real factory settings.

---

### Example 3: **Palletizing**
Palletizing involves stacking products onto a pallet in a specific pattern, a task FANUC robots excel at in warehouses or production lines.

#### Steps to Program:
1. **Define the Pick-Up Position**:  
   - Jog the robot to where it picks up the product (e.g., from a conveyor) and record it as P[1].

2. **Define Placement Positions**:  
   - Determine the pallet pattern (e.g., 2x3 grid, 2 layers). Jog to each placement spot on the first layer and record positions (e.g., P[2] to P[7]). For example:
     - P[2]: (X=100, Y=100, Z=50)
     - P[3]: (X=200, Y=100, Z=50)

3. **Use a Loop**:  
   - Write a program with a loop to repeat the pick and place for each product. Use a register (R[1]) to track the count:
     ```
     1: R[1]=1
     2: LBL[1]
     3: J P[1] 100% FINE
     4: DO[1]=ON   // Close gripper
     5: WAIT 0.5sec
     6: L P[R[1]+1] 500mm/sec FINE  // Dynamic position (P[2], P[3], etc.)
     7: DO[1]=OFF  // Open gripper
     8: R[1]=R[1]+1
     9: IF R[1]<=7, JMP LBL[1]  // Loop until 6 items placed
     ```

4. **Adjust for Layers**:  
   - For a second layer, increment the Z-coordinate (e.g., add 100mm). Modify the program:
     ```
     1: R[1]=1
     2: R[2]=0  // Layer offset
     3: LBL[1]
     4: J P[1] 100% FINE
     5: DO[1]=ON
     6: WAIT 0.5sec
     7: L P[R[1]+1] 500mm/sec FINE CNT10 OFFSET R[2]  // Add Z offset
     8: DO[1]=OFF
     9: R[1]=R[1]+1
     10: IF R[1]<=4, JMP LBL[1]  // First layer (3 items)
     11: R[2]=100  // Increase Z for second layer
     12: R[1]=1
     13: IF R[1]<=4, JMP LBL[1]  // Second layer
     ```

5. **Test and Optimize**:  
   - Run in step mode to verify the pattern. Adjust positions or speeds to minimize cycle time while ensuring stability.

#### Why This Works:
Palletizing demonstrates how to use loops and offsets to handle repetitive tasks with variations (e.g., layers), a common requirement in factories. It builds on basic programming while introducing more advanced concepts.

---

### Key Takeaways
- **Teach Pendant**: Use it to jog, record positions, and test programs safely (e.g., step mode, E-stop, reset).
- **Motion Types**: Choose **joint** for speed or **linear** for precision, depending on the task.
- **Coordinate Systems**: Select the appropriate system (world, tool, joint) for the job, and switch if singularities arise.
- **Safety**: Always test in step mode first and use the reset button to clear errors, as your notes suggest.

These examples cover core FANUC robot applications in factories. Let me know if you’d like to expand on any of them or explore additional scenarios!


---

Since you have experience with ROS (Robot Operating System) but are new to FANUC robot programming, I’ll guide you through the essentials of FANUC’s TP (Teach Pendant) language and briefly touch on Karel, its more advanced counterpart. I’ll also provide real-life application examples to connect these concepts to practical use cases. Let’s dive in!

---

### **Introduction to FANUC Robot Programming**

FANUC robots are a staple in industrial automation, commonly found in industries like automotive, electronics, and logistics. Unlike ROS, which is an open-source, modular framework with nodes and topics, FANUC uses proprietary programming languages: **TP language** for most tasks and **Karel** for advanced applications. These languages are designed to run on FANUC’s robot controllers and are typically programmed via a **teach pendant**, a handheld device used to control and program the robot.

Think of TP as your starting point—it’s straightforward and widely used—while Karel is a step up for complex logic or custom features. Since you’re familiar with ROS, I’ll draw parallels where possible to ease the transition.

---

### **Basic Concepts of FANUC TP Language**

The TP language is the primary way to program FANUC robots. It’s a textual, structured language (similar to BASIC or Pascal) that you write and edit directly on the teach pendant. Here are the must-know basics:

#### **Syntax and Structure**
- Programs are written line-by-line, with each line typically holding one command.
- Comments start with an exclamation mark: `! This is a comment`.
- Labels (e.g., `LBL[1]`) help control program flow, like jumping to specific sections.

#### **Key Commands**
1. **Motion Commands**  
   These tell the robot where and how to move:
   - `J P[1] 100% FINE`: Moves the robot to position `P[1]` using joint motion at 100% speed with high precision (FINE).
   - `L P[2] 500mm/sec CNT100`: Moves linearly to position `P[2]` at 500 mm/sec with continuous blending (CNT100), avoiding a full stop.
   - Positions like `P[1]` are predefined using the teach pendant by jogging the robot to a spot and recording it.

2. **Input/Output (I/O) Commands**  
   These control external devices like grippers or sensors:
   - `DO[1]=ON`: Turns on digital output 1 (e.g., to close a gripper).
   - `WAIT DI[1]=ON`: Pauses the program until digital input 1 is on (e.g., a sensor detects a part).

3. **Registers**  
   Registers store data, similar to variables in ROS:
   - `R[1]=5`: Sets register 1 to 5.
   - `PR[1]=P[1]`: Assigns position `P[1]` to position register 1 for dynamic adjustments.

4. **Program Flow**  
   TP uses simple logic to control execution:
   - `JMP LBL[1]`: Jumps to label 1.
   - `IF R[1]>0 THEN JMP LBL[2]`: Jumps to label 2 if register 1 is greater than 0.

#### **How It Works**
- You create TP programs on the teach pendant, often by combining manually taught positions with commands.
- Programs run sequentially, step-by-step, unlike ROS’s real-time, distributed approach.
- Debugging is done by stepping through lines on the pendant.

#### **Comparison to ROS**
- **Modularity**: ROS uses nodes for separate tasks (e.g., navigation, sensing), while TP programs are more monolithic—one program handles everything.
- **Flexibility**: TP is less dynamic than ROS; it’s designed for repeatable, deterministic tasks rather than adaptive behaviors.

---

### **A Quick Look at Karel**

While TP is great for standard tasks, **Karel** is FANUC’s advanced language, similar to Pascal. It’s compiled (not edited on the pendant) and used for:
- Complex logic (e.g., loops, conditionals).
- Custom interfaces on the teach pendant.
- Integration with external systems like vision or databases.

For now, focus on TP as your entry point—Karel is a next step once you’re comfortable.

---

### **Real-Life Application Examples**

Here’s how TP (and sometimes Karel) is used in the real world, with parallels to ROS concepts where applicable:

1. **Automotive Welding**
   - **Scenario**: A FANUC robot welds car frames on an assembly line.
   - **TP Use**: A TP program moves the robot along a weld path:
     ```
     J P[1] 100% FINE   ! Move to start
     L P[2] 300mm/sec FINE   ! Weld along seam
     DO[1]=ON   ! Turn on welder
     L P[3] 300mm/sec FINE
     DO[1]=OFF  ! Turn off welder
     ```
   - **ROS Parallel**: In ROS, you’d use a motion planner node and a separate I/O node. TP combines these into one program.
   - **Karel Add-On**: If a sensor adjusts the path in real-time (e.g., seam tracking), Karel could process the data and update positions.

2. **Material Handling (Palletizing)**
   - **Scenario**: A robot stacks boxes on a pallet from a conveyor.
   - **TP Use**: The program picks and places boxes, tracking layers with a register:
     ```
     L P[1] 500mm/sec FINE   ! Pick position
     DO[1]=ON   ! Close gripper
     L P[2] 500mm/sec FINE   ! Place position
     DO[1]=OFF  ! Open gripper
     R[1]=R[1]+1   ! Increment layer count
     IF R[1]<5 THEN JMP LBL[1]   ! Repeat for 5 layers
     ```
   - **ROS Parallel**: ROS might use a perception node to detect boxes and a planning node for motion. TP relies on pre-taught positions.

3. **Vision-Guided Pick and Place**
   - **Scenario**: A robot picks parts using a camera to find them.
   - **TP Use**: With FANUC’s vision option, TP can handle basic vision tasks:
     ```
     VISION RUN_FIND 'Part'   ! Find part with camera
     VISION GET_OFFSET 'Part' VR[1]   ! Get position offset
     L P[1] 500mm/sec FINE   ! Move to adjusted pick position
     ```
   - **Karel Use**: For complex vision (e.g., multiple parts or external cameras), Karel processes the data and sends commands to TP.
   - **ROS Parallel**: This is like ROS’s perception pipeline (e.g., using OpenCV with a move_group node), but FANUC integrates it more tightly.

---

### **Key Takeaways**
- **TP Language**: Start here. Learn motion commands (J, L), I/O control, and registers. It’s your bread-and-butter for FANUC programming.
- **Real-World Use**: TP handles repetitive tasks like welding or palletizing, while Karel steps in for customization or complexity.
- **From ROS to FANUC**: Shift from ROS’s flexibility and real-time control to FANUC’s structured, sequential approach.

To get hands-on, explore FANUC’s **ROBOGUIDE** software for simulation (like Gazebo in ROS) or check FANUC manuals and online tutorials. If you have more questions or want to dive deeper into a specific area, let me know!

---



### Key Points
- It seems likely that FANUC robots use AC servo motors for precise control, commonly integrated with TP for basic programming and Karel for complex logic.
- Research suggests TP is ideal for simple pick-and-place tasks, using motion commands like L (linear) for precision, while Karel handles advanced features like loops.
- The evidence leans toward combining TP and Karel for tasks needing external input checks, ensuring flexibility in industrial settings.

### Step-by-Step Guide for Programming a FANUC Robot
#### Teaching Positions
Start by using the teach pendant to manually move the robot to the pick-up location (e.g., P[1]) and place location (e.g., P[2]), recording each position into position registers for later use.

#### Writing the TP Program
Create a basic TP program for the pick-and-place task:
- Use **L (Linear motion)** for precise movement, e.g., `L P[1] 100% FINE` to move to the pick position at full speed and stop precisely.
- Add gripper control using digital outputs (DO), e.g., `DO[1]=ON` to close the gripper, with waits like `WAIT 500` for gripper actions.
- Example program:
  ```
  1: L P[1] 100% FINE ;
  2: DO[1]=ON ;
  3: WAIT 500 ;
  4: L P[2] 100% FINE ;
  5: DO[1]=OFF ;
  6: WAIT 500 ;
  ```

#### Adding Loops and Conditions
- For repeating the task (e.g., 10 times), use TP’s label and jump commands:
  - Initialize a counter, e.g., `R[1]=0`.
  - Use `LBL[1]` for the loop start and `JMP LBL[1]` with a condition like `IF R[1]<10`.
  - Example:
    ```
    1: R[1]=0 ;
    2: LBL[1] ;
    3: L P[1] 100% FINE ;
    4: DO[1]=ON ;
    5: WAIT 500 ;
    6: L P[2] 100% FINE ;
    7: DO[1]=OFF ;
    8: WAIT 500 ;
    9: R[1]=R[1]+1 ;
    10: IF R[1]<10, JMP LBL[1] ;
    ```
- For external inputs (e.g., checking if an object is present via DI[1]), add conditions like `IF DI[1]=ON, L P[1] 100% FINE ; ELSE JMP LBL[2]`.

#### Testing and Error Handling
- Test using the teach pendant to step through lines, ensuring movements and I/O work as expected.
- Use FANUC’s ROBOGUIDE for simulation to verify logic without physical movement.
- Handle errors by checking teach pendant error codes and consulting the manual for fixes, ensuring safety by avoiding collisions.

#### Unexpected Detail
Did you know Karel can also manage network communications, like reading data from a vision system, enhancing your program for dynamic environments? This could be useful for adapting to varying object positions.

---

### Survey Note: Detailed Analysis of FANUC Robot Programming for Pick-and-Place Tasks

To address the task of creating a program to control a FANUC robot for picking up an object and placing it elsewhere using TP or Karel, a step-by-step approach is necessary, given the user’s rustiness in FANUC programming. This note provides a comprehensive guide, leveraging the user’s familiarity with basic robotics and programming concepts, and integrates insights from industrial practices.

#### Background on TP and Karel
- **TP (Teach Pendant Language)**: Used for basic programming, it involves defining positions and simple motion commands, suitable for tasks like pick-and-place. It lacks advanced constructs like loops but supports labels and jumps for basic repetition.
- **Karel**: A higher-level language, similar to Pascal, enabling complex logic, loops, conditionals, and integration with external systems. It’s ideal for tasks requiring dynamic decision-making or extensive data handling.

For a simple pick-and-place, TP is sufficient, but for advanced features, combining TP with Karel is recommended, aligning with the user’s task requirements.

#### Step-by-Step Programming Guide

##### 1. Teaching Positions
- **Process**: Use the teach pendant to manually jog the robot to the pick-up location and record it as P[1], then to the place location and record as P[2]. This involves:
  - Selecting the position register (e.g., P[1]) via the pendant.
  - Using jog controls (e.g., COORD mode for Cartesian movement) to position the robot.
  - Pressing the “Record” button to save the position, ensuring no collisions or unreachable points.
- **Safety Consideration**: Ensure the path is clear of obstacles and operators, as safety is paramount in industrial settings.

##### 2. Writing the TP Program for Basic Motion
- **Motion Commands**: 
  - **J (Joint Motion)**: Faster, moves in joint space, less precise for pick-and-place.
  - **L (Linear Motion)**: Moves in a straight line in Cartesian space, ideal for precision, used with speed (e.g., 100%) and termination (e.g., FINE for precise stop).
- **Initial Program**: For a basic pick-and-place:
  ```
  1: L P[1] 100% FINE ;
  2: WAIT 1000 ;  (Simulate pick-up, e.g., 1 second)
  3: L P[2] 100% FINE ;
  4: WAIT 1000 ;  (Simulate place, e.g., 1 second)
  ```
- **Gripper Control**: FANUC robots use digital outputs (DO) for external devices. Update the program to include gripper actions:
  ```
  1: L P[1] 100% FINE ;
  2: DO[1]=ON ;  (Close gripper)
  3: WAIT 500 ;  (Wait 0.5 seconds for gripper to close)
  4: L P[2] 100% FINE ;
  5: DO[1]=OFF ;  (Open gripper)
  6: WAIT 500 ;  (Wait 0.5 seconds for gripper to open)
  ```
- **Reasoning**: Linear motion ensures precision, and waits simulate mechanical actions, aligning with industrial pick-and-place tasks.

##### 3. Adding Loops for Repetition
- **TP Limitation**: No native loops, so use labels and jumps. For 10 cycles:
  ```
  1: R[1]=0 ;  (Initialize cycle count in numeric register R[1])
  2: LBL[1] ;
  3: L P[1] 100% FINE ;
  4: DO[1]=ON ;
  5: WAIT 500 ;
  6: L P[2] 100% FINE ;
  7: DO[1]=OFF ;
  8: WAIT 500 ;
  9: R[1]=R[1]+1 ;  (Increment counter)
  10: IF R[1]<10, JMP LBL[1] ;  (Jump back if less than 10 cycles)
  ```
- **Logic**: R[1] starts at 0, increments after each cycle. After 9 cycles, R[1]=9, jumps back; after 10th cycle, R[1]=10, IF 10<10 is false, stops. This ensures 10 cycles, aligning with repetitive industrial tasks.

##### 4. Handling External Inputs
- **Conditionals in TP**: Use digital inputs (DI) for decisions, e.g., checking if an object is present via DI[1]:
  - Modify line 2: `2: IF DI[1]=ON, L P[1] 100% FINE ; ELSE JMP LBL[2] ;`
  - Add after line 8: `9: LBL[2] ;`
- **Complexity**: For multiple conditions, TP’s label/jump system can get messy. Karel is better for such scenarios, offering WHILE loops and IF-THEN-ELSE, e.g.:
  ```
  PROGRAM pick_place_loop
  VAR
      cycle_count: INTEGER = 0 ;
      max_cycles: INTEGER = 10 ;
  BEGIN
      WHILE cycle_count < max_cycles DO
          IF GET_INPUT(1) = TRUE THEN  (Check DI[1])
              CALL_PROG('pick_place') ;
          ENDIF
          cycle_count = cycle_count + 1 ;
      ENDWHILE
  END pick_place_loop
  ```
- **Recommendation**: For basic tasks, TP suffices; for dynamic inputs, combine with Karel for clarity.

##### 5. Testing and Error Handling
- **Testing**: Use the teach pendant to step through lines, observing movements and I/O. FANUC’s ROBOGUIDE ([FANUC ROBOGUIDE](https://www.fanucamerica.com/products/robotics/robot-simulation-software/roboguide)) allows simulation, verifying logic without physical risk.
- **Error Handling**: FANUC displays error codes on the pendant (e.g., position errors, I/O faults). Refer to the manual for diagnostics, ensuring safety by checking for collisions or unreachable positions.

##### 6. Additional Considerations
- **Robot Model Variations**: Different FANUC models (e.g., LR Mate, M-10iA) may have additional axes or motion types. Check the manual for specifics.
- **System Integration**: For larger systems, Karel can manage communication (e.g., via EtherNet/IP, Profinet) with other machines, ensuring coordination. Example Karel routine for network:
  ```
  ROUTINE read_vision_data
  VAR
      vision_ip: STRING = '192.168.0.100' ;
      vision_port: INTEGER = 5000 ;
      data: STRING ;
  BEGIN
      OPEN_TCP(vision_ip, vision_port) ;
      READ_LINE(data) ;
      CLOSE_TCP() ;
  END read_vision_data
  ```
- **Safety**: Ensure paths avoid human operators, using safety interlocks (e.g., e-stops) as per standards like ISO 13849.

#### Unexpected Detail
Karel’s ability to handle network communications, like reading from a vision system, adds flexibility for dynamic environments, potentially adapting pick positions based on object detection, enhancing industrial applications beyond static tasks.


---

### Key Citations
- [FANUC Robot Simulation Software Overview](https://www.fanucamerica.com/products/robotics/robot-simulation-software/roboguide)

---

Transitioning from ROS (Robot Operating System) to FANUC robot programming can feel like a shift from a high-level, modular framework to a more hardware-specific, industrial approach. FANUC robots primarily use two programming languages: **TP (Teach Pendant)** and **Karel**. Since TP is the most commonly used for basic operations and is required for all FANUC robot applications, I’ll focus on introducing its must-know concepts and then provide real-life application examples to bridge your ROS experience with FANUC TP programming.

---

### Must-Know Concepts of FANUC TP Language

FANUC TP is a structured, text-based language accessed and edited via the robot’s teach pendant (a handheld device with a screen and buttons). Unlike ROS, which uses a publish-subscribe model with nodes and abstraction layers, TP is more direct and tied to the robot’s physical operations. Here are the essentials:

1. **Program Structure**:
   - TP programs are a sequence of numbered lines, each containing a single instruction (e.g., motion, logic, or I/O control).
   - Example:
     ```
     1: J P[1] 100% FINE ;
     2: L P[2] 500mm/sec CNT10 ;
     ```
     - `J` = joint move, `L` = linear move, `P[1]` = position 1, `100%` = speed, `FINE` = stop precisely at the point.

2. **Motion Instructions**:
   - **Joint (J)**: Moves the robot’s joints to a position, fastest but less predictable path.
   - **Linear (L)**: Moves the tool in a straight line, ideal for precision tasks.
   - **Circular (C)**: Moves along an arc, used for curved paths.
   - **Speed**: Specified as a percentage (e.g., `50%`) or absolute value (e.g., `500mm/sec`).
   - **Termination**: `FINE` (stop exactly at the point) or `CNT` (continuous, blends into the next move, e.g., `CNT10` for a small blend radius).
   - Example: `L P[3] 200mm/sec FINE` moves linearly to position 3 at 200 mm/s and stops precisely.

3. **Positions and Registers**:
   - **Position Registers (P[n])**: Store robot positions (x, y, z, and orientation). You teach these via the pendant by jogging the robot to a spot and recording it.
   - **Numeric Registers (R[n])**: Store numbers (e.g., counters, speeds). Example: `R[1]=5` sets register 1 to 5.
   - Unlike ROS, where positions might be dynamically computed via transforms, TP relies on pre-taught or manually adjusted points.

4. **Logic and Flow Control**:
   - **IF Statements**: Basic conditional logic, e.g., `IF R[1]=1, JMP LBL[10]` jumps to label 10 if register 1 equals 1.
   - **Labels (LBL[n]) and Jumps (JMP)**: TP lacks WHILE loops, so you use `JMP LBL[n]` to repeat or skip code sections. This is a key difference from ROS’s more modern control structures.
   - Example:
     ```
     1: IF DI[1]=ON, JMP LBL[5] ;
     2: L P[1] 100mm/sec FINE ;
     3: JMP LBL[10] ;
     4: LBL[5] ;
     5: L P[2] 100mm/sec FINE ;
     6: LBL[10] ;
     ```
     - If digital input 1 is ON, it skips to position 2; otherwise, it goes to position 1.

5. **Input/Output (I/O)**:
   - **Digital Inputs (DI[n])** and **Outputs (DO[n])**: Control external devices (e.g., grippers). Example: `DO[1]=ON` turns on output 1.
   - Similar to ROS’s ability to interface with hardware via topics, but TP is more manual and register-based.

6. **Comments**:
   - Use `!` to add comments, e.g., `! Move to home position`. This helps readability, much like documentation in ROS nodes.

7. **Tool and User Frames**:
   - **Tool Frame**: Defines the robot’s tool tip (e.g., gripper). Set via `UTOOL[n]`.
   - **User Frame**: Defines a custom coordinate system for the workspace. Set via `UFRAME[n]`.
   - Comparable to ROS’s TF (transform) system, but static and manually configured in TP.

8. **Execution**:
   - Programs run line-by-line on the teach pendant. You can step through them for debugging, unlike ROS’s real-time execution across nodes.

---

### Real-Life Application Examples

Here are three practical examples to connect TP programming to real-world tasks, leveraging your ROS background:

#### 1. Pick-and-Place (Material Handling)
- **Scenario**: A FANUC robot picks parts from a conveyor and places them on a pallet, similar to a ROS node subscribing to a conveyor sensor topic and publishing to a gripper.
- **TP Program**:

```
  1: UFRAME[1] ; ! Set workspace frame
  2: UTOOL[1] ; ! Set gripper tool
  3: LBL[1] ; ! Start loop
  4: IF DI[1]=OFF, JMP LBL[10] ; ! Wait for part detection
  5: L P[1] 500mm/sec FINE ; ! Move to pick position
  6: DO[1]=ON ; ! Close gripper
  7: WAIT 0.5 ; ! Wait 0.5s for grip
  8: L P[2] 500mm/sec FINE ; ! Move to place position
  9: DO[1]=OFF ; ! Open gripper
  10: JMP LBL[1] ; ! Loop back
  11: LBL[10] ; ! End
```

- **ROS Analogy**: DI[1] is like a sensor topic triggering the pick action, and DO[1] mimics an actuator command. The loop mimics a ROS node’s main loop.

#### 2. Dispensing (Glue Application)
- **Scenario**: The robot applies glue along a straight path, akin to a ROS path planner sending waypoints.
- **TP Program**:

```
  1: UTOOL[2] ; ! Set dispenser tool
  2: L P[1] 200mm/sec FINE ; ! Move to start
  3: DO[2]=ON ; ! Start dispensing
  4: L P[2] 100mm/sec CNT0 ; ! Linear path with glue on
  5: DO[2]=OFF ; ! Stop dispensing
  6: L P[3] 500mm/sec FINE ; ! Move away
```

- **ROS Analogy**: P[1] to P[2] is like a MoveIt trajectory, and DO[2] is a service call to activate the dispenser. CNT0 ensures a smooth path, similar to interpolation in ROS.

#### 3. Testing (Quality Inspection)
- **Scenario**: The robot moves a camera to inspect a part, counting defects with a counter, similar to a ROS node processing image data.
- **TP Program**:

```
  1: R[1]=0 ; ! Initialize defect counter
  2: L P[1] 300mm/sec FINE ; ! Move to inspection point 1
  3: IF DI[2]=ON, CALL INCREMENT ; ! Defect detected, call subroutine
  4: L P[2] 300mm/sec FINE ; ! Move to point 2
  5: IF DI[2]=ON, CALL INCREMENT ;
  6: IF R[1]>2, DO[3]=ON ; ! Signal too many defects
  -- Subroutine --
  7: INCREMENT: ;
  8: R[1]=R[1]+1 ;
  9: END ;
```

- **ROS Analogy**: DI[2] is like a topic from a vision system, R[1] is a variable in a node, and the subroutine is akin to a callback function.

---

### Key Differences from ROS
- **Abstraction**: ROS abstracts hardware details with nodes and messages; TP requires direct control of registers and I/O.
- **Real-Time**: ROS runs distributed processes; TP is sequential and tied to the pendant.
- **Flexibility**: ROS excels in dynamic environments; TP is rigid but reliable for repetitive tasks.

### Tips for Learning FANUC TP
- **Start Simple**: Use the teach pendant to jog the robot and record positions, then build basic motion programs.
- **Experiment**: Try small loops with JMP and LBL to mimic ROS loops.
- **Debug**: Step through lines on the pendant to see what each does, akin to ROS logging.

With your ROS background, you’ll adapt quickly by treating TP as a lower-level, hardware-focused scripting language. If you have access to a FANUC robot or simulator (e.g., ROBOGUIDE), practice these examples to solidify your understanding. Let me know if you’d like more specific guidance!


---

Comparing FANUC’s TP (Teach Pendant) language to a compiled language like C++ in the context of industrial robots highlights their distinct purposes and trade-offs. TP is a domain-specific, interpreted scripting language tailored for FANUC robots, while C++ is a general-purpose, compiled language often used in robotics (e.g., via ROS or custom firmware). Since production-ready robots prioritize real-time performance and reliability, let’s break down the advantages of TP over C++ in this specific domain, while acknowledging where C++ shines.

---


### Advantages of TP Over C++ in FANUC Robot Programming

1. **Simplicity and Accessibility**:
   - **TP Advantage**: TP is designed for operators and technicians, not just software engineers. Its syntax is straightforward (e.g., `L P[1] 500mm/sec FINE`), and it’s edited directly on the teach pendant without needing external tools like IDEs or compilers.
   - **C++ Context**: C++ requires advanced programming knowledge (pointers, memory management, object-oriented design) and a development environment (e.g., Visual Studio, gcc), making it less accessible to non-programmers on the factory floor.
   - **Real-Life Impact**: A maintenance tech can quickly adjust a TP program to tweak a pick-and-place position during a shift, whereas modifying C++ code might require a software engineer, recompilation, and redeployment—disrupting production.

2. **Native Integration with FANUC Hardware**:
   - **TP Advantage**: TP is tightly coupled to FANUC’s controller architecture (e.g., R-30iB). It directly interfaces with the robot’s registers, I/O, and motion planner, ensuring seamless, optimized execution without additional abstraction layers.
   - **C++ Context**: In C++, you’d need to interface with the robot via APIs (e.g., FANUC’s PC Interface or ROS drivers), adding overhead and potential latency. You’d also have to manage low-level details (e.g., communication protocols) manually.
   - **Real-Life Impact**: For a material transport task, TP’s `DO[1]=ON` instantly triggers a conveyor output, while C++ might require a custom driver and error handling, risking delays or mismatches.

3. **Built-In Real-Time Guarantees**:
   - **TP Advantage**: TP runs on FANUC’s real-time operating system (RTOS), which is pre-tuned for deterministic motion control and safety. Motion commands (e.g., `J P[1] 100% CNT10`) are executed with guaranteed timing by the controller’s firmware.
   - **C++ Context**: C++ offers flexibility but no inherent real-time guarantees unless paired with an RTOS (e.g., VxWorks, QNX) and carefully optimized. Poorly written C++ code can introduce jitter or crashes, unacceptable in high-speed production.
   - **Real-Life Impact**: In a fastening application (e.g., screwing bolts at 500 rpm), TP ensures the robot moves precisely in sync with the tool, while C++ might require extensive tuning to avoid timing errors.

4. **Rapid Development and Debugging**:
   - **TP Advantage**: TP is interpreted, so changes take effect immediately—no compilation step. You can step through lines on the pendant, jog the robot to test positions, and adjust on the fly.
   - **C++ Context**: C++ requires writing, compiling, linking, and uploading code to the robot, a slower cycle. Debugging often involves logs or external tools, not direct hardware interaction.
   - **Real-Life Impact**: For a dispensing task, a technician can tweak TP’s speed (`200mm/sec` to `150mm/sec`) mid-shift to fix glue flow, while C++ changes might take hours to redeploy.

5. **Reliability Through Constraints**:
   - **TP Advantage**: TP’s limited feature set (no pointers, no dynamic memory allocation) reduces the risk of bugs like memory leaks or segmentation faults, common in C++. It’s a “walled garden” optimized for repetitive, industrial tasks.
   - **C++ Context**: C++’s power comes with complexity—unmanaged resources or race conditions can crash a program, a disaster in a 24/7 production line.
   - **Real-Life Impact**: In a testing application counting defects, TP’s simple `IF DI[1]=ON, R[1]=R[1]+1` is foolproof, while C++ might need mutexes or thread safety, risking subtle errors.

6. **Standardization Across FANUC Ecosystem**:
   - **TP Advantage**: TP is the universal language for FANUC robots, ensuring consistency across models and facilities. Programs are portable between FANUC systems with minimal tweaks.
   - **C++ Context**: C++ code is custom-built, so a program for one robot might not work on another without significant refactoring, especially across vendors.
   - **Real-Life Impact**: A pick-and-place TP script can be reused across FANUC robots in different plants, while C++ would need adaptation to each controller’s API.

---

### Where C++ Has the Edge (For Context)
- **Flexibility**: C++ can handle complex algorithms (e.g., path planning, vision processing) that TP can’t. FANUC often pairs TP with Karel or external PCs running C++ for advanced tasks.
- **Performance**: For non-real-time tasks (e.g., offline simulation), C++’s compiled efficiency beats TP’s interpreted execution.
- **Scalability**: C++ integrates with ROS or networked systems for multi-robot coordination, while TP is single-robot focused.

---

### Real-Life Application Comparison

#### Pick-and-Place Task
- **TP Approach**:
```
  1: L P[1] 500mm/sec FINE ; ! Pick position
  2: DO[1]=ON ; ! Gripper on
  3: L P[2] 500mm/sec FINE ; ! Place position
  4: DO[1]=OFF ; ! Gripper off
```
  - Advantage: Quick to write, runs directly on the controller, zero setup beyond teaching positions.
- **C++ Approach**: 
  - Write a program using FANUC’s PC SDK, define joint angles, manage socket communication, handle exceptions, compile, deploy.
  - Advantage: Could integrate sensor fusion or dynamic path adjustment, but overkill for a simple task.

#### Outcome
- TP wins for speed, reliability, and ease in this repetitive, real-time scenario—critical in a production line cranking out thousands of parts daily.

---

### Why TP Fits Production Robots
FANUC robots are embedded systems built for **reliability**, **repeatability**, and **real-time control** in demanding environments (e.g., automotive assembly). TP aligns with this by:
- Minimizing abstraction to keep latency low.
- Simplifying programming to reduce human error.
- Leveraging the controller’s optimized RTOS for motion and I/O.

C++ is better suited for research, prototyping, or complex systems where flexibility trumps simplicity (e.g., a ROS-based autonomous robot). In contrast, TP is the “assembly line workhorse”—less elegant but laser-focused on getting the job done in a factory.

---

### Bridging Your ROS Experience
Since you’re familiar with ROS, think of TP as a “node” that’s pre-built into the FANUC controller. It lacks ROS’s modularity but compensates with direct hardware control and a no-nonsense approach. If you ever need more power (e.g., vision integration), FANUC’s Karel (a Pascal-like language) or external C++ via PC Interface can complement TP—similar to how ROS nodes call C++ libraries.

Let me know if you’d like a deeper dive into TP’s internals or a specific use case!


---


Since Karel is mentioned in the job description alongside FANUC TP, it’s worth understanding its role in FANUC robot programming, especially as a complement to TP. While TP (Teach Pendant) is the go-to for basic motion and I/O control, Karel is a more powerful, structured programming language used for advanced tasks on FANUC robots. Let’s dive into what Karel is, its must-know concepts, and how it fits into real-life applications—especially with your ROS background in mind.

---

### What is Karel?

Karel is FANUC’s proprietary, high-level programming language, inspired by Pascal. Unlike TP, which is a simple, line-by-line scripting language edited on the teach pendant, Karel is compiled and offers greater flexibility for complex logic, data handling, and integration. It runs on the FANUC robot controller (e.g., R-30iB) but requires a development environment like ROBOGUIDE or a text editor with FANUC’s compiler tools.

- **Purpose**: Karel handles tasks that TP can’t easily manage, such as custom algorithms, file I/O, advanced decision-making, or communication with external systems.
- **Comparison to TP**: TP is like a macro recorder for quick robot moves; Karel is a full programming language for sophisticated applications.
- **Comparison to ROS**: Karel is closer to writing C++ nodes in ROS but is still FANUC-specific and less modular than ROS’s ecosystem.

---

### Must-Know Concepts of Karel

Here’s a breakdown of Karel’s core elements, tailored to help you grasp it quickly:

1. **Program Structure**:
   - Karel programs have a defined structure: a header, variable declarations, and a main body.
   - Example:
   ```
     PROGRAM pick_place
     %STACKSIZE = 4000  -- Memory allocation
     VAR
        counter: INTEGER  -- Variable declaration
     BEGIN
        counter = 0
        WRITE('Starting pick and place', CR)
     END pick_place
   ```
   - Similar to C++’s `main()` or a ROS node’s entry point, but with Pascal-like syntax (`BEGIN`/`END`).

2. **Variables and Data Types**:
   - Supports integers (`INTEGER`), reals (`REAL`), booleans (`BOOLEAN`), strings (`STRING`), and arrays.
   - Example: `position: POSITION` stores robot positions (like TP’s `P[n]` but more flexible).
   - Unlike TP’s limited registers (`R[n]`), Karel allows named variables and complex data structures.

3. **Motion Control**:
   - Karel can command robot motion using built-in routines, offering more control than TP’s simple `J`/`L` commands.
   - Example:
   ```
     MOVE TO home_pos  -- Moves to a predefined position
     SET_JOINT_POS(j1, j2, j3, j4, j5, j6)  -- Sets joint angles directly
   ```
   - Think of this as a ROS `MoveIt` call but executed within the controller.

4. **Logic and Control Flow**:
   - Offers full programming constructs: `IF-THEN-ELSE`, `FOR`, `WHILE`, and `CASE`.
   - Example:
   ```
     IF counter < 10 THEN
        MOVE TO pick_pos
        counter = counter + 1
     ELSE
        MOVE TO home_pos
     ENDIF
   ```
   - Far more powerful than TP’s `IF/JMP LBL`, akin to C++ or Python logic in ROS nodes.

5. **Input/Output (I/O)**:
   - Controls digital/analog I/O with routines like `SET_OUTPUT` or `GET_INPUT`.
   - Example: `SET_OUTPUT(1, TRUE)` turns on DO[1], similar to TP’s `DO[1]=ON`.
   - Can also read/write files (e.g., logs), unlike TP.

6. **Subroutines and Functions**:
   - Define reusable procedures with parameters.
   - Example:
   ```
     ROUTINE close_gripper
     BEGIN
        SET_OUTPUT(1, TRUE)
        DELAY(500)  -- Wait 500ms
     END close_gripper
   ```
   - Comparable to ROS service calls or C++ functions.

7. **Integration with TP**:
   - Karel programs can call TP programs (and vice versa) using `CALL_PROG('TP_PROG_NAME')`.
   - This hybrid approach leverages TP’s simplicity for motion and Karel’s logic for decision-making.

8. **Error Handling**:
   - Uses `CONDITION` blocks to monitor faults (e.g., I/O states, robot errors).
   - Example:
   ```
     CONDITION[1]:
        WHEN DI[1] = FALSE DO
           WRITE('Sensor failed', CR)
        ENDWHEN
     ENDCONDITION
   ```
   - Similar to ROS’s error callbacks but tied to the controller’s event system.

---

### Advantages of Karel Over TP and C++
- **Vs. TP**: Karel offers loops, complex math, and external communication (e.g., TCP/IP), while TP is limited to basic sequencing and I/O.
- **Vs. C++**: Karel runs natively on the FANUC controller, avoiding external PC dependencies or API latency. It’s also simpler than C++ for robot-specific tasks, with built-in motion and I/O routines.

---

### Real-Life Application Examples

Here’s how Karel fits into industrial scenarios, with ties to your ROS experience:

#### 1. Material Pick-and-Place with Counter
- **Scenario**: A robot picks parts and tracks cycles, stopping after 100 picks for maintenance—too complex for TP’s basic logic.
- **Karel Program**:
```
  PROGRAM pick_counter
  VAR
     pick_count: INTEGER
     pick_pos, place_pos: POSITION
  BEGIN
     pick_count = 0
     WHILE pick_count < 100 DO
        MOVE TO pick_pos
        SET_OUTPUT(1, TRUE)  -- Gripper on
        DELAY(500)
        MOVE TO place_pos
        SET_OUTPUT(1, FALSE) -- Gripper off
        pick_count = pick_count + 1
        WRITE('Picks: ', pick_count, CR)
     ENDWHILE
     WRITE('Maintenance required', CR)
  END pick_counter
```
- **ROS Analogy**: The `WHILE` loop is like a ROS node’s main loop, and `WRITE` mimics `ROS_INFO` logging.

#### 2. Dispensing with Dynamic Adjustments
- **Scenario**: The robot adjusts dispensing speed based on a sensor reading (e.g., material thickness), requiring real-time computation.
- **Karel Program**:
```
  PROGRAM dispense_control
  VAR
     speed: REAL
     thickness: REAL
  BEGIN
     thickness = GET_ANALOG_INPUT(1)  -- Read sensor
     speed = 100 + (thickness * 50)   -- Adjust speed
     MOVE TO start_pos
     SET_OUTPUT(2, TRUE)  -- Start dispensing
     MOVE TO end_pos AT speed
     SET_OUTPUT(2, FALSE)
  END dispense_control
```
- **ROS Analogy**: `GET_ANALOG_INPUT` is like subscribing to a sensor topic, and the speed calculation mirrors a control algorithm in a ROS node.

#### 3. Testing with File Logging
- **Scenario**: The robot inspects parts and logs defect data to a file for predictive maintenance—beyond TP’s capabilities.
- **Karel Program**:
```
  PROGRAM defect_logger
  VAR
     defect_count: INTEGER
     file_handle: FILE
  BEGIN
     defect_count = 0
     OPEN_FILE(file_handle, 'DEFECTS.TXT', 'W')
     FOR i = 1 TO 10 DO
        MOVE TO inspect_pos[i]
        IF GET_INPUT(2) = TRUE THEN
           defect_count = defect_count + 1
        ENDIF
     ENDFOR
     WRITE_FILE(file_handle, 'Defects: ', defect_count, CR)
     CLOSE_FILE(file_handle)
  END defect_logger
```
- **ROS Analogy**: File I/O is like saving data to a ROS bag, and the `FOR` loop resembles iterating over a waypoint list.

---

### How Karel Fits the Job Description
In the context of “Create, Standardize, Debug and Troubleshoot programs of Fanuc Robots,” Karel is likely used for:
- **Complex Tasks**: Writing custom logic (e.g., predictive maintenance algorithms) that TP can’t handle.
- **Standardization**: Creating reusable Karel routines (e.g., gripper control) called by multiple TP programs.
- **Debugging**: Adding detailed logging or error handling to diagnose issues in production.

---

### Bridging from ROS
- **Similarities**: Karel’s structured syntax (`IF`, `FOR`) will feel familiar from C++/Python in ROS. Its motion commands are like ROS action clients sending goals to a robot.
- **Differences**: Karel is monolithic (no nodes/topics), runs on the controller, and lacks ROS’s dynamic reconfiguration. It’s more like a single, embedded C++ program.

### Learning Tips
- **Start with TP**: Since Karel often integrates with TP, master TP first (motion, I/O) before tackling Karel’s logic.
- **Use ROBOGUIDE**: FANUC’s simulation software lets you write and test Karel programs offline.
- **Experiment**: Try a simple Karel program (e.g., a counter with motion) to see how it extends TP.

Karel gives you the power to go beyond TP’s limits while staying within FANUC’s ecosystem—perfect for the advanced automation tasks in your job description. Let me know if you’d like a sample program expanded or more details!


---



Let’s create a summary of TP and Karel, then weave them into a real-life story with a sample program that integrates both. This will give you a practical sense of how they work together in a FANUC robot environment, tailored to your interest in industrial automation and your ROS background.

---

### Summary of TP and Karel

- **TP (Teach Pendant) Language**:
  - **What it is**: A simple, interpreted scripting language for FANUC robots, edited directly on the teach pendant.
  - **Strengths**: Quick to write, ideal for basic motion (e.g., `J P[1] 100% FINE`), I/O control (`DO[1]=ON`), and repetitive tasks. Runs natively on the controller with real-time reliability.
  - **Limitations**: Lacks advanced logic (e.g., no `WHILE` loops, only `JMP LBL`), minimal data handling (registers like `R[n]`), and no file I/O.
  - **Use Case**: Perfect for operators to program straightforward pick-and-place or dispensing tasks on the factory floor.

- **Karel**:
  - **What it is**: A compiled, Pascal-like programming language for FANUC robots, offering structured programming capabilities.
  - **Strengths**: Supports complex logic (`IF-THEN-ELSE`, `FOR`, `WHILE`), named variables, file I/O, and custom routines. Integrates with TP for hybrid workflows.
  - **Limitations**: Requires compilation (e.g., via ROBOGUIDE), less intuitive for quick edits, and still FANUC-specific (not as flexible as C++/ROS).
  - **Use Case**: Used by engineers for advanced tasks like data logging, dynamic adjustments, or multi-step processes.

- **Integration**: TP handles the “what” (move here, turn this on), while Karel handles the “why” or “how” (decide based on conditions, log results). A TP program can call a Karel program with `CALL_PROG`, blending simplicity with sophistication.

---

### Real-Life Story: The Widget Assembly Line

Imagine you’re an automation engineer at a factory producing widgets—small gadgets assembled from parts picked off a conveyor, fastened with screws, and tested for quality. The production line uses a FANUC robot, and your job is to program it to handle these tasks efficiently, meeting the demands of the job description: “Create, Standardize, Debug and Troubleshoot programs of Fanuc Robots” for processes like material pick-and-place, fastening, and testing.

#### The Challenge
The line runs 24/7, and the robot needs to:
1. Pick parts from a conveyor and place them in a fixture (a simple, repetitive task).
2. Monitor how many parts it’s processed and log defective ones to a file for predictive maintenance (a complex task requiring data handling).
3. Stop after 50 picks to signal maintenance, ensuring reliability.

TP alone can handle the pick-and-place, but it can’t manage the counter, logging, or conditional stopping effectively. Karel steps in to fill those gaps.

#### The Solution
You write a TP program for the core motion and I/O, then a Karel program for logic and logging, integrating them seamlessly.

---

### Sample Program: TP + Karel Integration

#### TP Program: `WIDGET_PICK`
This TP script moves the robot to pick and place parts, calling a Karel program to handle the counter and logging.
```
 1: UFRAME[1] ; ! Conveyor workspace frame
 2: UTOOL[1] ; ! Gripper tool
 3: LBL[1] ; ! Start loop
 4: IF DI[1]=OFF, JMP LBL[10] ; ! Wait for part sensor
 5: L P[1] 500mm/sec FINE ; ! Move to pick position
 6: DO[1]=ON ; ! Close gripper
 7: WAIT 0.5 ; ! Wait 0.5s
 8: L P[2] 500mm/sec FINE ; ! Move to place position
 9: DO[1]=OFF ; ! Open gripper
10: CALL_PROG('CHECK_COUNT') ; ! Call Karel to update counter/log
11: JMP LBL[1] ; ! Loop back
12: LBL[10] ; ! End
```
- **What it does**: Loops to pick parts when the conveyor sensor (DI[1]) detects them, then calls the Karel program after each cycle.

#### Karel Program: `CHECK_COUNT`
This Karel program tracks picks, logs defects, and stops the robot after 50 cycles.
```
PROGRAM check_count
%STACKSIZE = 4000
VAR
   pick_count: INTEGER
   defect_count: INTEGER
   log_file: FILE
   sensor_state: BOOLEAN
BEGIN
   -- Initialize on first run (persisted via registers)
   IF pick_count = 0 THEN
      pick_count = GET_REG(1)  -- Read TP register R[1]
      defect_count = GET_REG(2) -- Read TP register R[2]
      OPEN_FILE(log_file, 'WIDGET_LOG.TXT', 'A')  -- Append mode
   ENDIF

   -- Increment pick counter
   pick_count = pick_count + 1
   SET_REG(1, pick_count)  -- Update TP register R[1]

   -- Check defect sensor (DI[2]) and log
   sensor_state = GET_INPUT(2)
   IF sensor_state = TRUE THEN
      defect_count = defect_count + 1
      SET_REG(2, defect_count)
      WRITE_FILE(log_file, 'Defect at pick #', pick_count, CR)
   ENDIF

   -- Stop after 50 picks
   IF pick_count >= 50 THEN
      SET_OUTPUT(3, TRUE)  -- Signal maintenance (DO[3])
      WRITE_FILE(log_file, 'Maintenance required at ', pick_count, ' picks', CR)
      CLOSE_FILE(log_file)
      ABORT  -- Stop the TP program
   ENDIF

   CLOSE_FILE(log_file)
END check_count
```
- **What it does**: Updates a pick counter, logs defects to a file, and halts the robot after 50 picks by triggering an output and aborting the TP loop.

---

#### The Story Unfolds
You deploy the TP program `WIDGET_PICK` to the FANUC robot’s controller via the teach pendant, teaching positions P[1] (conveyor) and P[2] (fixture) by jogging the robot. It starts picking parts smoothly, with the gripper (DO[1]) snapping shut and open like clockwork. But the real magic happens when `CALL_PROG('CHECK_COUNT')` fires.

The Karel program `CHECK_COUNT` kicks in after each pick. It reads the conveyor sensor (DI[1]) and a defect sensor (DI[2]), tracking counts in registers (R[1], R[2]) that TP can access. If a part fails inspection, Karel logs it to `WIDGET_LOG.TXT`—data you’ll later analyze for predictive maintenance, a task from your job description. After 50 picks, Karel flips DO[3] to alert the maintenance team and stops the robot, preventing wear-related failures.

One day, a supervisor notices the robot stopping early. You step through the TP program on the pendant and see it’s looping fine, so you check Karel’s log file via ROBOGUIDE. The defect count spiked—turns out a sensor was misaligned. You tweak the Karel logic to filter noise (`IF sensor_state = TRUE FOR 100ms`), recompile, and redeploy. Problem solved, production resumes, and you’ve just debugged and troubleshooted like a pro.

---

### Why This Works
- **TP’s Role**: Handles the real-time, repetitive pick-and-place with minimal fuss, leveraging the controller’s motion planner for reliability—perfect for the factory’s 24/7 demands.
- **Karel’s Role**: Adds intelligence—counting, logging, and decision-making—without bogging down TP’s simplicity. It’s standardized (the `CHECK_COUNT` routine could be reused) and supports the job’s data analysis goals.
- **Integration**: The `CALL_PROG` bridges them, much like ROS nodes calling services, but all within FANUC’s embedded ecosystem.

---

### Your ROS Lens
From your ROS perspective, TP is like a pre-built node with fixed topics (`P[n]`, `DO[n]`), while Karel is a custom C++ node you’d write for logic. Together, they’re a streamlined, production-ready duo—no external PCs or network latency, just raw efficiency.

This story and sample program show how TP and Karel tackle real-world automation tasks hand-in-hand. Want to tweak this example further or explore a different scenario? Let me know!

---

