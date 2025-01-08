package frc.lib2202.subsystem.hid;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;


/**
* Extension of the CommandJoystick class that adds support for additional
* buttons on Thrustmaster T16000M
*
* @see CommandJoystick
*/
public class ThrustMaster extends CommandGenericHID {


 private m_Joystick joystick;


 public ThrustMaster(final int port) {
   super(port);
   joystick = new m_Joystick(port);
 }


 /**
  * Get the x position of the HID.
  *
  * @return the x position
  */
 public double getX() {
   return joystick.getX();
 }


 /**
  * Get the y position of the HID.
  *
  * @return the y position
  */
 public double getY() {
   return joystick.getY();
 }


 /**
  * Get the z position of the HID.
  *
  * @return the z position
  */
 public double getZ() {
   return joystick.getZ();
 }


 /**
  * Get the twist value of the current joystick. This depends on the mapping of
  * the joystick
  * connected to the current port.
  *
  * @return The Twist value of the joystick.
  */
 public double getTwist() {
   return joystick.getTwist();
 }


 /**
  * Get the throttle value of the current joystick. This depends on the mapping
  * of the joystick
  * connected to the current port.
  *
  * @return The Throttle value of the joystick.
  */
 public double getThrottle() {
   return joystick.getThrottle();
 }
 /*---------------------- EXTENSION BUTTONS -------------------- */


 /**
  * Constructs an event instance around designated buttons digital signal.
  *
  * @return an event instance representing designated digital signal attached to
  *         the {@link
  *         CommandScheduler#getDefaultButtonLoop() default scheduler button
  *         loop}.
  * @see #Button(EventLoop)
  */
 public Trigger trigger(m_Joystick.ButtonType button) {
   return trigger(button, CommandScheduler.getInstance().getDefaultButtonLoop());
 }


 /**
  * Constructs an event instance around designated digital signal.
  *
  * @param loop the event loop instance to attach the event to.
  * @return an event instance representing designated digital signal attached to
  *         the given
  *         loop.
  */
 public Trigger trigger(m_Joystick.ButtonType button, EventLoop loop) {
   return joystick.Button(button, loop).castTo(Trigger::new);
 }
}


