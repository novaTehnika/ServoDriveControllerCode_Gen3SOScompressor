## Introduction
This project aims to develop IEC Structured Text source code for a Yaskawa MP2600iec servo drive controller.
The application is a third generation, pre-clinical medical device being used to study an intervention for hypoxia in a university research context.
The intervention is the infusion of a saline solution with oxygen micro bubbles into the blood stream.
The device produces and delivers this solution by dissolving oxygen into a saline solution at a high pressure and discharging the solution to near atmospheric pressure through micro-orifices (leading to the nucleation of micro-bubbles).
The servo system provides the motive force for this process.

## Application process
The process the device performs is as follows.
Step 1: Saline is drawn into a compression chamber, comprised of a cylinder and piston, in excess of the specified volume.
The saline source is closed off from the chamber.

Step 2: A waste stream outlet is opened. Air and excess saline are purged from the compression chamber leaving the specified volume of saline.
The waste stream outlet is closed.

Step 3: An oxygen source inlet is opened to the chamber.
Oxygen is drawn into the chamber.
A pressure transducer in communication with the compression chamber senses the pressure.
The density of the gas is estimated, assuming room temperature.
The volume required to give the specified mass of oxygen is calculated, giving the target end-position for the piston.
The piston moves to that position. The oxygen source inlet is closed.

Step 4: The piston moves to compress the two phases to a pressure sufficient to dissolve the oxygen in the saline solution.
A magnetic stir-bar in the compression chamber is driven by a rotating magnet attached to a DC motor embedded within the piston but outside the compression chamber. This accelerated diffusion of the dissolved oxygen.

Step 5: Concurrent with Step 4, a peristaltic pump is used to drive saline through the passages leading to and away from the compression chamber to flush out residual air.

Step 6: After a sufficient amount of time, the solution is ready for discharge and delivery.
Valves leading to the micro-orifice(s) are opened, the intermediate volume is pressurized, and solution is driven through the orifices.
The piston continues to move and apply pressure to the fluid.
Once the residual fluid in the intermediate volume is driven out, the super-oxygenated saline begins reaches and flow through the orifices, producing the oxygen micro-bubble/saline mixture.

## Servo System
The controller is controlling a Yaskawa Sigma-7 Servo Drive (model SGD7S2R8FE0A000300), driving a single axis.
The axis drives a linear electromechanical actuator (Tolomatic RSA64).
The servo motor is a 400W Yaskawa Sigma-7 motor with a safety brake option and an absolute encoder (model SGM7J-04A6A6C).

The controller is planned to be programmed as a slave to a Simulink Desktop Real Time program.
The servo controller slave shall implement basic modes of operation at the command of the master.
The Simulink Desktop Real Time environment as the master enables faster iteration and a familiar interface for the engineering graduate student/researchers, while enabling a rough but workable interface for operators (via dashboard components or, at a later point an app via Simulink's App Designer).

The master controller is interfacing with the system through a National Instruments (NI) PCI 6251 DAQ, accompanied by a NI SCB-68a breakout board.

We have a MotionWorksIEC Express license for programming the servo controller and drive, which has a smaller feature set than MotionWorksIEC Pro but is adequate this application.

The MP2600iec has:
- eight digital inputs
- eight digital outputs
- one analog input
- one analog output

The servo drive connects to the servo encoder directly without needing to use any of the eight digital inputs.

Control of the brake is being handled by a separate E-stop circuit with input from the NI PIC 6251.
That E-stop circuit controls the brake and the Safety Torque Off (STO) inputs to the drive's IGBT's.
One of the slave's digital outputs will be wired to an additional relay that provides input to the brake's actuation without affecting the STO.
(The brake is normally engaged and the circuit provides 24V power to disengage it allow servo motion.)


## Functionality
There are two primary modes of operation required: motion control (position/velocity) and torque control.
Additional modes include an idle state, a braked holding state, a mode for homing to a limit switch, and a mode for homing to the end-of-travel for the piston in the cylinder.

Three digital inputs to the servo controller from the NI DAQ will be used to set the mode of operation.
Three bits will be used as such:
- 000: idle
- 001: brake holding
- 010: position control
- 011: velocity control
- 100: torque control
- 101: not used
- 110: home to limit switch
- 111: home to end-of-travel

A master-slave handshake will be used to ensure agreement.
Three digital outputs from the servo controller to the NI DAQ will confirm the mode of operation.
An digital input to the slave from the master will then allow motion to occur.

The slave will have one digital output dedicated to disengaging the brake, allowing for
1) verification that the servo drive is working before releasing the brake and
2) activation of the brake in the brake holding mode.

For motion control the master will provide a -10V to 10V analog reference signal for position/velocity.
The servo controller will condition and pass the signal to the servo drive.

For the torque control mode, the master will be reading the pressure transducer signal to determine the pressure in the compression chamber and will send a -10V to 10V signal to the slave to as a reference for its torque control.
The master will determine this reference from a combining feed-forward and feed-back signals.
The desired compression chamber pressure, the area of the piston, the gearing of the lead screw and gearbox to the servo motor, and a simple model of the friction in the system will be used for calculating a feed-forward term while error between the pressure transducer signal and the desired pressure will be used to provide compensation.
From the slave's perspective, it receives, conditions, and passes the torque reference to the servo drive.

The servo controller will use its analog output pin to communicate the actuator position to the master in all modes.

The servo controller will also use a digital output to communicate its performance depending on it's control mode.
For motion and torque control, it will communicate when it is limited in its motion due to constraints other than it's control objective (e.g. when torque control, the pin will be active when it's motion is velocity or position limited).
For the homing modes, the pin will communicate when it has completed it's routine and the master should note the position.

Finally, a digital output pin will be used to communicate when the controller enters an error or fault state.

## Implementation
The program for the MP2600 iec implements a state machine for the basic modes of operation given in the section Functionality, error or fault states, and transition states.
The state machine lives in `PRG_Main` (Structured Text). Built-in motion function blocks (`MC_Power`, `MC_Stop`, `MC_Reset`, `MC_MoveAbsolute`, `MC_MoveVelocity`, `MC_ReadActual*`, `Y_DirectControl`, and `AbsolutePositionManager`) are instantiated in a Ladder Diagram POU and exchange data with `PRG_Main` through structured globals (`G_cmd*` for commands, `G_sta*` for status). `Y_DirectControl` is used to communicate reference commands and control modes to the servo drive during the motion/torque operating modes.
The structure of the code follows best practices for IEC 61131-3 compliant Structured Text, with custom logic encapsulated in function blocks under `src/FB/`.