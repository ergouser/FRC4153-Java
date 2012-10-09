/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package com.team4153.oppie2012;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Oppie extends SimpleRobot {
    
    /**Bridge Balance Kp parameter. */
    public static final double KP = 1.5;
    /**Bridge Balance Ki parameter. */
    public static final double KI = 0.6;
    /**Bridge Balance Kd parameter. */
    public static final double KD = 0.2;
    
    public static final int LEFT_FRONT_MOTOR = 2;
    public static final int LEFT_REAR_MOTOR = 6;
    public static final int RIGHT_FRONT_MOTOR = 4;
    public static final int RIGHT_REAR_MOTOR = 8;
    public static final int LEFT_JOYSTICK = 1;
    public static final int RIGHT_JOYSTICK = 2;
    public static final int MANIPULATOR_JOYSTICK = 3;
    protected Joystick leftStick = new Joystick(LEFT_JOYSTICK);
    protected Joystick rightStick = new Joystick(RIGHT_JOYSTICK);
    protected Joystick manipulatorStick = new Joystick(MANIPULATOR_JOYSTICK);
    protected RobotDrive drive;
    public static Oppie robotInstance;
    public ADXL345_I2C accelerometer;
    public Gyro gyro;
    
    /** The slot into which the accelerometer is plugged. */
    public static int ACCELEROMETER_SLOT = 1;
    /** The slot into which the gyro is plugged. */
    public static int GYRO_SLOT = 1;
    /** The gyro analog channel. */
    public static int GYRO_ANALOG_CHANNEL = 2;
    
    /** The PIDController. */
    public PIDController pidController;

    /**
     * Create the state model for the shooter.
     */
    protected void robotInit() {
        super.robotInit();
        // oops, wait, it's a singleton, don't need to do this.
        try {
            drive = new RobotDrive(new CANJaguar(LEFT_FRONT_MOTOR), new CANJaguar(LEFT_REAR_MOTOR), new CANJaguar(RIGHT_FRONT_MOTOR), new CANJaguar(RIGHT_REAR_MOTOR));
            drive.setSafetyEnabled(false);
        } catch (Exception any) {
            any.printStackTrace();
        }
        robotInstance = this;
        accelerometer = new ADXL345_I2C(ACCELEROMETER_SLOT, ADXL345_I2C.DataFormat_Range.k4G);
        gyro = new Gyro(GYRO_SLOT, GYRO_ANALOG_CHANNEL);
        DataServer.getDataServer();  // make sure this is listening.
    }

    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        System.err.println("Entering Teleop");
        StateModel stateModel = StateModel.getStateModel();
        stateModel.reset();
        DataServer dataServer = DataServer.getDataServer();
        while (isOperatorControl() && isEnabled()) {
            try {
//            System.err.println("Mode " + dataServer.mode + " percent " + dataServer.shooterHeadPercent + " speed " + dataServer.shooterSpeed);
//               if (dataServer.mode == DataServer.CALIBRATION_MODE) {  // calibration mode
//                    if (manipulatorStick.getRawButton(1)) {
//                        // shoot with the provided parameters.
//                        stateModel.shooterHeadSpeed = dataServer.shooterSpeed / 100f;
//                        ShooterHead.getShooterHead().setPosition(dataServer.shooterHeadPercent);
//                        stateModel.shoot(true);
//                    }
//                    Timer.delay(0.005);
//                }
                if (manipulatorStick.getRawButton(3)) {  // autoshoot...
                        ShooterHead.getShooterHead().autoShoot();
                    Timer.delay(0.005);
//                }
//               else if (manipulatorStick.getRawButton(9)) {  // autobalance...
//                   if ( pidController == null ) {
//                  PIDOutput driveOutput =  new PIDOutput () {
//
//                        public void pidWrite(double output) {
//                            drive.tankDrive(-output, -output);
//                        }
//                    
//                    
//                    };
//                 PIDSource accelSource =  new PIDSource () {
//
//                        public double pidGet() {
//                            return accelerometer.getAcceleration(ADXL345_I2C.Axes.kX);
//                        }
//                    
//                    };
//                    pidController = new PIDController(KP, KI, KD, accelSource, driveOutput, 0.01);
//                   }
                } else { // normal operation.
                    if ( pidController != null ) {
                        pidController.disable();
                        pidController = null;
                    }
                    System.out.println ("To Robot drive " + rightStick.getY() + " " +  leftStick.getY() );
                    double right = rightStick.getY();
                    if ( right >  0.55 ) {
                        right = 0.5;
                    }
                    double left = leftStick.getY();
                    if ( left > 0.55 ) {
                        left = 0.5;
                    }
                    drive.tankDrive(right, left);
                    //drive.tankDrive(rightStick, leftStick);
                    //System.err.println("Teleop");
                    //StateModel.getStateModel().runRoller(manipulatorStick.getRawButton(3));
                    //StateModel.getStateModel().runBelt(manipulatorStick.getRawButton(4));
                    //StateModel.getStateModel().runMetering(manipulatorStick.getRawButton(5));
                    stateModel.task();
                    stateModel.shoot(manipulatorStick.getRawButton(1));
                    //StateModel.getStateModel().runRoller(manipulatorStick.getRawButton(2));
                    if (manipulatorStick.getRawButton(8)) {
                        stateModel.reset();
                    }
                    //Gyro g = new Gyro();
                    //System.err.println("Teleop - running BMS");
                    BMS.getBMS().task();
                    ShooterHead.getShooterHead().task();
                    Timer.delay(0.005);
                }
            } catch (Exception ex) {
                ex.printStackTrace();
            }
        }
        
    }
}
