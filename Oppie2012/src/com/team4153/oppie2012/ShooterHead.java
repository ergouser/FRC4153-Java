/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.team4153.oppie2012;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import java.util.Vector;

class CalibrationData {

    public double distance;  // inches
    public double shooterHeadPosition; // volts
    public double shooterSpeed;  // percent

    public CalibrationData(double distance, double shooterHeadPosition, double shooterSpeed) {
        this.distance = distance;
        this.shooterHeadPosition = shooterHeadPosition;
        this.shooterSpeed = shooterSpeed;
    }
}

/**
 *
 * Control of the shooter head. low limit 1.19 high limit 4.72
 */
public class ShooterHead {

    /**
     * The top limit - 4.72V.
     */
    public static final double TOP_LIMT = 4.72;
    /**
     * The lower limit - 1.19V.
     */
    public static final double LOWER_LIMT = 1.19;
    /**
     * Top limit switch
     */
    public static final int ANALOG_INPUT = 1;
    /**
     * JAG for the shooter.
     */
    public static final int SHOOTER_JAG = 5;
    /**
     *
     * /**
     * The motor that moves the shooter head.
     */
    protected CANJaguar headDrive;
    /**
     * The analog input that gives the position.
     */
    protected AnalogChannel positionInput;
    /**
     * The bridge management system singleton.
     */
    protected static ShooterHead shooter;
    /**
     * The calibration data. This is a Vector of CalibrationData. It must be
     * ordered by increasing distance.
     */
    protected Vector calibrationData;

    /**
     * State model is a singleton.
     */
    public static ShooterHead getShooterHead() {
        if (shooter == null) {
            shooter = new ShooterHead();
        }
        return shooter;
    }

    /**
     * Constructor.
     */
    protected ShooterHead() {
        try {
            positionInput = new AnalogChannel(ANALOG_INPUT);
            headDrive = new CANJaguar(SHOOTER_JAG);
        } catch (Exception e) {
            e.printStackTrace();
        }
        calibrationData = new Vector();
        // so, irritating hand filling of the Vector, but it's easy and it works..
        // DUMMY DATA SO FAR
        calibrationData.addElement(new CalibrationData(24, 4.0, 0.25));
        calibrationData.addElement(new CalibrationData(120, 4.0, 0.75));
    }

    /**
     * Called periodically to check the current status of the bms.
     */
    public void task() {
        boolean buttonPressed = Oppie.robotInstance.manipulatorStick.getRawButton(6) || Oppie.robotInstance.manipulatorStick.getRawButton(11);
        //System.err.println("Shooter head throttle " + " " + buttonPressed + " x " + Oppie.robotInstance.manipulatorStick.getX() + " y " + Oppie.robotInstance.manipulatorStick.getY() + " " + positionInput.getAverageVoltage());
        if (buttonPressed) {
            try {
                double amount = Oppie.robotInstance.manipulatorStick.getY();
                if (positionInput.getAverageVoltage() < TOP_LIMT && amount < 0) {
                    System.err.println("Shooter head throttle " + Oppie.robotInstance.manipulatorStick.getThrottle() + " " + positionInput.getAverageVoltage());
                    headDrive.setX(amount);
                } else {
                    if (amount < 0) {
                        headDrive.setX(0);
                    }

                }
                if (positionInput.getAverageVoltage() > LOWER_LIMT && amount > 0) {
                    System.err.println("Shooter head throttle " + Oppie.robotInstance.manipulatorStick.getThrottle() + " " + positionInput.getAverageVoltage());
                    headDrive.setX(amount);
                } else {
                    if (amount > 0) {
                        headDrive.setX(0);
                    }

                }
            } catch (CANTimeoutException ex) {
                ex.printStackTrace();
            }
        }
    }

    void setPosition(int shooterHeadPercent) throws CANTimeoutException {
        double targetVoltage = (TOP_LIMT - LOWER_LIMT) * shooterHeadPercent / 100f;
        double amount = 0.5;  // don't know which is up and which is down, so someone need to CAREFULLY test this.
        // do this until the two match (within ~0.3% - we can probably do better than that...  A2D is 10 bit (I think) so, 1 in 1024 ~0.001V
        while (targetVoltage > (positionInput.getAverageVoltage() + 0.01) || targetVoltage < (positionInput.getAverageVoltage() - 0.01)) {
            if (targetVoltage > positionInput.getAverageVoltage()) {
                // move down (maybe not, that's why both limits are in here.  I'm not sure of the sign of amount to move in the down direction
                if (positionInput.getAverageVoltage() < TOP_LIMT && positionInput.getAverageVoltage() > LOWER_LIMT && positionInput.getAverageVoltage() < targetVoltage) {
                    System.err.println("Shooter head throttle " + Oppie.robotInstance.manipulatorStick.getThrottle() + " " + positionInput.getAverageVoltage());
                    headDrive.setX(amount);
                } else {
                    headDrive.setX(0);
                }
            } else {
                if (positionInput.getAverageVoltage() < TOP_LIMT && positionInput.getAverageVoltage() > LOWER_LIMT && positionInput.getAverageVoltage() > targetVoltage) {
                    System.err.println("Shooter head throttle " + Oppie.robotInstance.manipulatorStick.getThrottle() + " " + positionInput.getAverageVoltage());
                    headDrive.setX(-amount);
                } else {
                    headDrive.setX(0);
                }

            }
        }
        headDrive.setX(0);
    }

    /**
     * This will move the shooter head. It should be checked perioidically to
     * ensure that the limits have not been reached.
     */
    void moveHead(double targetVoltage, double speed) throws CANTimeoutException {
        try {
            if (targetVoltage > (positionInput.getAverageVoltage() + 0.01) || targetVoltage < (positionInput.getAverageVoltage() - 0.01)) {
                if (targetVoltage > positionInput.getAverageVoltage()) {
                    // move down (maybe not, that's why both limits are in here.  I'm not sure of the sign of amount to move in the down direction
                    if (positionInput.getAverageVoltage() < TOP_LIMT && positionInput.getAverageVoltage() > LOWER_LIMT && positionInput.getAverageVoltage() < targetVoltage) {
                        //System.err.println("Shooter head throttle " + Oppie.robotInstance.manipulatorStick.getThrottle() + " " + positionInput.getAverageVoltage());
                        headDrive.setX(speed);
                    } else {
                        headDrive.setX(0);
                    }
                } else {
                    if (positionInput.getAverageVoltage() < TOP_LIMT && positionInput.getAverageVoltage() > LOWER_LIMT && positionInput.getAverageVoltage() > targetVoltage) {
                        //System.err.println("Shooter head throttle " + Oppie.robotInstance.manipulatorStick.getThrottle() + " " + positionInput.getAverageVoltage());
                        headDrive.setX(-speed);
                    } else {
                        headDrive.setX(0);
                    }

                }
            }
        } finally {
            headDrive.setX(0);
        }
    }

    /**
     * Here's where we auto-target and shoot. The shooter head is trashed, so
     * rather than try to shoot from our current position, we're going to move
     * until we are 52" from the target (image is 63048 sq. pixels) and aligned
     * with it and we'll shoot from there.
     */
    void autoShoot() throws CANTimeoutException {
        DataServer dataServer = DataServer.getDataServer();
        StateModel stateModel = StateModel.getStateModel();
        RobotDrive drive = Oppie.robotInstance.drive;
        int currentInchesFromTarget = -1;
        System.out.println("AutoShoot: " + dataServer.distanceToTarget + " " + dataServer.angleToTarget);
        while (Oppie.robotInstance.manipulatorStick.getRawButton(3) && dataServer.distanceToTarget > 62 && dataServer.distanceToTarget > 0) {
            System.out.println("AutoShoot: " + dataServer.distanceToTarget + " " + dataServer.angleToTarget);
            if (dataServer.distanceToTarget > 62) {
                // move forward (we hope).
                int factor;
                if (dataServer.angleToTarget < 0 ) {
                    factor = 1;
                }
                else {
                    factor = -1;
                }
                drive.tankDrive(-0.36-factor*0.1, -0.36+factor*0.1);
            } else if (dataServer.distanceToTarget < 62) {
                // move backwards
                drive.tankDrive(0.3, 0.3);
            }
         }
        drive.tankDrive(0, 0);
        if ( dataServer.distanceToTarget < 62 ){
            StateModel.getStateModel().shoot(true);
        }
        drive.tankDrive(0, 0);

    }
//     /**
//     * Here's where we auto-target and shoot. We need the angle and distance
//     * from the vision system. The angle is used to re-orient the robot, then
//     * the distance is used to calculate the shooter head position and the
//     * speed.
//     */
//    void autoShoot() throws CANTimeoutException {
//        DataServer dataServer = DataServer.getDataServer();
//        StateModel stateModel = StateModel.getStateModel();
//        try {
//            //OK, we're going to start the robot moving, then we're going to start the head moving.
//            if (dataServer.angleToTarget >= 1000) {  // not doing anything, don't have any data.
//                return;
//            }
//            double targetVoltage = -1;
//            double shooterHeadSpeed = -1;
//            for (int counter = 0; counter < calibrationData.size() - 1; counter++) {
//                CalibrationData cd = (CalibrationData) calibrationData.elementAt(counter);
//                CalibrationData nextCd = (CalibrationData) calibrationData.elementAt(counter + 1);
//                if (dataServer.distanceToTarget > cd.distance) {
////estimatedDistance = (areas[d1] -boundingArea)/(areas[d1]-areas[d1+1]) * (distances[d1+1]-distances[d1]) + distances[d1];
//                    double fraction = (dataServer.distanceToTarget - cd.distance) / (nextCd.distance - cd.distance);
//                    shooterHeadSpeed = fraction * (nextCd.shooterSpeed - cd.shooterSpeed) + cd.shooterSpeed;
//                    targetVoltage = fraction * (nextCd.shooterHeadPosition - cd.shooterHeadPosition) + cd.shooterHeadPosition;
//                    break;
//                }
//            }
//            System.err.println("Shooter head angle " + targetVoltage + " and speed " + shooterHeadSpeed + " Vision Angle " + dataServer.angleToTarget + " distance " + dataServer.distanceToTarget);
//            if (targetVoltage < 0 || shooterHeadSpeed < 0) {
//                return;
//            }
//            while (dataServer.angleToTarget > 5 && dataServer.angleToTarget < -5 && dataServer.angleToTarget < 1000) {
//                // rotate the drive 
//                if (dataServer.angleToTarget < 0) {
//                    Oppie.robotInstance.drive.tankDrive(0.25, -0.25);
//                } else {
//                    Oppie.robotInstance.drive.tankDrive(-0.25, 0.25);
//                }
//                moveHead(targetVoltage, 0.5);
//            }
//            // everything is ready....
//            stateModel.meteringDrive.set(Relay.Value.kReverse);
//            stateModel.shootDrive.setX(shooterHeadSpeed);
//            while ( Oppie.robotInstance.manipulatorStick.getRawButton(3) ) {
//                System.err.println ("If the shot is off, you can let go of the button");
//            }
//                    
//        } finally {
//            stateModel.meteringDrive.set(Relay.Value.kOff);
//            stateModel.shootDrive.setX(0);
//            headDrive.setX(0);
//        }
//    }
//
}