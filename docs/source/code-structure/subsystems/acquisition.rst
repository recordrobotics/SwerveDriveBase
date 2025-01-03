Acquisition
=============================
``public class Acquisition extends KillableSubsystem``
Section of the Robot resposible for collecting notes from the field

Spins 1 DC motor

Aquisition can be spinning into the note channel (IN), out of the note channel (REVERSE), or not moving(OFF).
On Start up defaults to off
The state of the acquisition is changed through calling ``toggle(AcquisitionStates state)``.

**public void toggle(AcquisitionStates state, double speed))**
	Changes the speed of the acquisition in the inputed direction
**public void toggle(AcquisitionStates state)**
	Wrapper around general toggle this function turns the acquisition in the inputed direction and sets it to the default speed

