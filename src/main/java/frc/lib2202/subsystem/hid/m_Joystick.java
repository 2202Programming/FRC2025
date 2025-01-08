// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.lib2202.subsystem.hid;


import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;


/**
* Extension of the Joystick class that adds support for additional buttons on
* Thrustmaster T16000M
*
* @see Joystick
*/
public class m_Joystick extends GenericHID {
 /** Default X axis channel. */
 public static final byte kDefaultXChannel = 0;


 /** Default Y axis channel. */
 public static final byte kDefaultYChannel = 1;


 /** Default Z axis channel. */
 public static final byte kDefaultZChannel = 2;


 /** Default twist axis channel. */
 public static final byte kDefaultTwistChannel = 2;


 /** Default throttle axis channel. */
 public static final byte kDefaultThrottleChannel = 3;


 /** Represents an analog axis on a joystick. */
 public enum AxisType {
   /** X axis. */
   kX(0),
   /** Y axis. */
   kY(1),
   /** Z axis. */
   kZ(2),
   /** Twist axis. */
   kTwist(3),
   /** Throttle axis. */
   kThrottle(4);


   /** AxisType value. */
   public final int value;


   AxisType(int value) {
     this.value = value;
   }
 }


 /** Represents a digital button on a joystick. */
 public enum ButtonType {
   Trigger(1),
   UpTop(2), LeftTop(3), RightTop(4),


   LeftOne(5), LeftTwo(6), LeftThree(7),
   LeftFour(8), LeftFive(9), LeftSix(10),


   RightOne(11), RightTwo(12), RightThree(13),
   RightFour(14), RightFive(15), RightSix(16);


   /** ButtonType value. */
   public final int value;


   ButtonType(int value) {
     this.value = value;
   }
 }


 private final byte[] m_axes = new byte[AxisType.values().length];


 /**
  * Construct an instance of a joystick.
  *
  * @param port The port index on the Driver Station that the joystick is plugged
  *             into.
  */
 public m_Joystick(final int port) {
   super(port);


   m_axes[AxisType.kX.value] = kDefaultXChannel;
   m_axes[AxisType.kY.value] = kDefaultYChannel;
   m_axes[AxisType.kZ.value] = kDefaultZChannel;
   m_axes[AxisType.kTwist.value] = kDefaultTwistChannel;
   m_axes[AxisType.kThrottle.value] = kDefaultThrottleChannel;


   HAL.report(tResourceType.kResourceType_Joystick, port + 1);
 }


 /**
  * Get the X value of the joystick. This depends on the mapping of the joystick
  * connected to the
  * current port.
  *
  * @return The X value of the joystick.
  */
 public final double getX() {
   return getRawAxis(m_axes[AxisType.kX.value]);
 }


 /**
  * Get the Y value of the joystick. This depends on the mapping of the joystick
  * connected to the
  * current port.
  *
  * @return The Y value of the joystick.
  */
 public final double getY() {
   return getRawAxis(m_axes[AxisType.kY.value]);
 }


 /**
  * Get the z position of the HID.
  *
  * @return the z position
  */
 public final double getZ() {
   return getRawAxis(m_axes[AxisType.kZ.value]);
 }


 /**
  * Get the twist value of the current joystick. This depends on the mapping of
  * the joystick
  * connected to the current port.
  *
  * @return The Twist value of the joystick.
  */
 public final double getTwist() {
   return getRawAxis(m_axes[AxisType.kTwist.value]);
 }


 /**
  * Get the throttle value of the current joystick. This depends on the mapping
  * of the joystick
  * connected to the current port.
  *
  * @return The Throttle value of the joystick.
  */
 public final double getThrottle() {
   return getRawAxis(m_axes[AxisType.kThrottle.value]);
 }


 /**
  * Get the magnitude of the direction vector formed by the joystick's current
  * position relative to
  * its origin.
  *
  * @return The magnitude of the direction vector
  */
 public double getMagnitude() {
   return Math.hypot(getX(), getY());
 }


 /**
  * Get the direction of the vector formed by the joystick and its origin in
  * radians.
  *
  * @return The direction of the vector in radians
  */
 public double getDirectionRadians() {
   return Math.atan2(getX(), -getY());
 }


 /**
  * Get the direction of the vector formed by the joystick and its origin in
  * degrees.
  *
  * @return The direction of the vector in degrees
  */
 public double getDirectionDegrees() {
   return Math.toDegrees(getDirectionRadians());
 }


 /*---------------------- BUTTONS -------------------- */
 /**
  * Read the value of buttons on the ThrustMaster.
  *
  * @button the button to read the state of.
  * @return The state of the button.
  */
 public boolean getButton(ButtonType button) {
   return getRawButton(button.value);
 }


 /**
  * Constructs an event instance around buttons digital signal.
  *
  * @param button the button to read the state of.
  * @param loop   the event loop instance to attach the event to.
  * @return an event instance representing the Sw11 button's digital signal
  *         attached to the given
  *         loop.
  */
 public BooleanEvent Button(ButtonType button, EventLoop loop) {
   return new BooleanEvent(loop, () -> getButton(button));
 }


 /**
  * Whether the button was pressed since the last check.
  *
  * @return Whether the button was pressed since the last check.
  */
 public boolean getButtonPressed(ButtonType button) {
   return getRawButtonPressed(button.value);
 }


 /**
  * Whether the top button was released since the last check.
  *
  * @return Whether the button was released since the last check.
  */
 public boolean getButtonReleased(ButtonType button) {
   return getRawButtonReleased(button.value);
 }
}


