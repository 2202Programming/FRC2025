package frc.lib2202.util;

import static frc.lib2202.Constants.DT;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * PIDFController - extends current (2020) pidcontroller to include a feed
 * forward gain which is not currently part of the WPILib version.
 * 
 * This is useful for holding values for devices like the talon SRX or sparkMax
 * which may have a feed forward gain or Izone.
 * 
 * 2/16/21 added CopyTo helper functions
 * 
 */
public class PIDFController extends PIDController {
    // hardware refs if used
    SparkClosedLoopController sparkMaxController = null;
    double m_smartMaxVel = 0.1;
    double m_smartMaxAccel = .01;

    double m_Kf = 0.0;
    
    public PIDFController(double Kp, double Ki, double Kd, double Kf) {
        this(Kp, Ki, Kd, Kf, DT);
    }

    public PIDFController(double Kp, double Ki, double Kd, double Kf, double period) {
        super(Kp, Ki, Kd, period);
        setF(Kf);
    }

    public PIDFController(PIDFController src) {
        this(src.getP(), src.getI(), src.getD(), src.getF(), src.getPeriod());
    }

    public void setPIDF(double kP, double kI, double kD, double kF) {
        setPID(kP, kI, kD);
        setF(kF);
    }

    // Accessors for the Kf
    public double getF() {
        return m_Kf;
    }

    public void setF(double Kf) {
        m_Kf = Kf;
    }
    
    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint    The new setpoint of the controller.
     */
    @Override
    public double calculate(double measurement, double setpoint) {
        return super.calculate(measurement, setpoint) + (m_Kf * setpoint);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     */
    @Override
    public double calculate(double measurement) {
        return calculate(measurement, getSetpoint());
    }

    /**
     * Copied from base class and feed forward added.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("f", this::getF, this::setF);
        builder.addDoubleProperty("iZone", this::getIZone, this::setIZone);
    }

    public boolean equals(PIDFController other) {
        return getP() == other.getP() && getI() == other.getI() && getD() == other.getD() && getF() == other.getF();
    }

    /**
     * 
     * copyTo() copies this pid's values down to a hardward PID implementation
     * 
     * @param dest          device
     * @param slot          control slot on device
     * 
     *                      optional smartMax vel and accel limits may be given
     * @param smartMaxVel   optional, 0.1 [units/s]
     * @param smartMaxAccel optional 0.01 [units/s^2]
     */
    public void copyTo(SparkMax motorController) {
        copyTo(motorController, m_smartMaxVel, m_smartMaxAccel);
    }

    public void copyTo(SparkMax motorController, double smartMaxVel, double smartMaxAccel) {
    
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        //need to check - if we just update a few parameters in SparkMaxConfig, do the rest stay the same as previously set?
        //otherwise do we need to pull all the prior parameters out of the motorController's sparkmaxconfig and reapply them?
        motorConfig.closedLoop.pidf(this.getP(), this.getI(), this.getD(), this.getF());
        motorConfig.closedLoop.iZone(this.getIZone());
        motorConfig.closedLoop.maxMotion.maxAcceleration(smartMaxAccel);
        motorConfig.closedLoop.maxMotion.maxVelocity(smartMaxVel);

        REVLibError driveError = motorController.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        if(driveError != REVLibError.kOk)
        System.out.println("*** ERROR *** SparkMax Flash Failed during copyTo command. Error val=" + driveError);
    }

    // compares an updated PIDF with this one and updates it and the hardware
    public void copyChangesTo(SparkMax dest, PIDFController updated) {

        Boolean changed = false;

        // update pid values that have changed
        if (getP() != updated.getP()) {
            setP(updated.getP());
            changed = true;
        }

        if (getI() != updated.getI()) {
            setI(updated.getI());
            changed = true;
        }

        if (getD() != updated.getD()) {
            setD(updated.getD());
            changed = true;
        }

        if (getF() != updated.getF()) {
            setF(updated.getF());
            changed = true;
        }

        if (getIZone() != updated.getIZone()) {
            setIZone(updated.getIZone());
            changed = true;
        }

        if (changed) {
            copyTo(dest);
        }
    }

}
