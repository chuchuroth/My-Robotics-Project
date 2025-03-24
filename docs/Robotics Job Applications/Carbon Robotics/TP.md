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
