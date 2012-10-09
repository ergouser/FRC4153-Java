/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.team4153.oppie2012;

import edu.wpi.first.wpilibj.ADXL345_I2C;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import javax.microedition.io.Connector;
import javax.microedition.io.ServerSocketConnection;
import javax.microedition.io.SocketConnection;

public class DataServer {
    
    public static final int CALIBRATION_MODE  = 1;

    // received values.  These are read in the order they are defined here.  The 
    // values will be populated by any client calling in on this socket.
    /**
     * The mode of operation. Currently, 1 is calibration mode, everything else
     * is "normal" mode.
     */
    public int mode;
    /**
     * The angle to the target. This is provided by the vision system and is the
     * number of pixels the robot must rotate in order to be aligned with the
     * target. Any value greater than 1000 is ignored. That is, anything other
     * than the vision system should send in a large angle (except for testing).
     */
    public float angleToTarget;
    /**
     * The distance to the target. This is provided by the vision system and is
     * used in conjunction with the calibration data to determine the head angle
     * and shooter speed to use when firing the ball. Any value < 0 is ignored,
     * so any system that is not the vision system should send in a negative
     * value.
     */
    public float distanceToTarget;
    /**
     * The speed to run the shooter motor when the "shoot" button is pressed.
     * This is used only in calibration mode (this.mode=1) and is otherwise
     * ignored. The value is a percentage (0-100) of the power applied to the
     * motor. That is, the encoder is assumed broken.
     */
    public int shooterSpeed;
    /**
     * The angle of the shooter head. This is assumed a percentage from 0-100.
     * Zero is at the bottom of the travel and 100 at the top. Values are read
     * from the shooter head analog input.
     */
    public int shooterHeadPercent;
    
    /** The thread that is running that data server. */
    Thread listeningThread;
    
    /** The data server singleton. */
    protected static DataServer dataServer;
    
    /** Returns the DataServer Singleton. */
    public static DataServer getDataServer () {
        if ( dataServer == null ) {
          dataServer = new DataServer();  
        }
        return dataServer;
    }

    /**
     * Default constructor.
     */
    protected DataServer() {
        Runnable r = new Runnable() {

            public void run() {
                while (true) {
                    DataInputStream dis = null;
                    DataOutputStream dos = null;
                    SocketConnection socketConnection = null;
                    ServerSocketConnection socket = null;
                    try {
                        System.err.println("Waiting for connection");
                        // I think "socket" is correct, but "serversocket" seems to be needed for the simulator.
                        ///socket = (ServerSocketConnection)Connector.open("socket://:5000");
                        socket = (ServerSocketConnection) Connector.open("serversocket://:5000");

                        // Wait for a connection.
                        socketConnection = (SocketConnection) socket.acceptAndOpen();
                        // I really can't be bothered to track these connections so I'm going to require a reconnect
                        // everytime.  There's a minor extra overhead on this (not much, this is the normal behavior of http).
                        // There's also possible problems with simultaneous connections.  Depends on the TCP stack, but
                        // I'm building a debug tool not an enterprise server, so I really don't care.
                        // What happens next is we read values from the stream (here we simulate two ints a float )
                        // After that we send the values that have been stored in this object
                        // then close and clean up everything..
                        System.err.println("Connection accepted from: " + socketConnection.getAddress());
                        dis = new DataInputStream(socketConnection.openInputStream());
                        dos = new DataOutputStream(socketConnection.openOutputStream());

                        mode = dis.readInt();

                        float tAngle = dis.readFloat();
                        if (tAngle < 1000) {
                            angleToTarget = tAngle;
                        }
                        float tDistance = dis.readFloat();
                        if (tDistance > 0) {
                            distanceToTarget = tDistance;
                        }
                       System.out.println ("AutoShoot: " + tDistance + " " +angleToTarget);
                     shooterSpeed = dis.readInt();
                        shooterHeadPercent = dis.readInt();

                        // OK, so current list is 4 doubles (x,y,z,angle).
                        
                        ADXL345_I2C.AllAxes axes = Oppie.robotInstance.accelerometer.getAccelerations();
                       
                        dos.writeDouble (axes.XAxis);
                        dos.writeDouble (axes.YAxis);
                        dos.writeDouble (axes.ZAxis);
                        dos.writeDouble (Oppie.robotInstance.gyro.getAngle());
                        
                    } catch (Exception e) {
                        e.printStackTrace();
                    } finally {
                        if (dis != null) {
                            try {
                                dis.close();
                            } catch (IOException e) {
                                e.printStackTrace();
                            }
                            dis = null;
                        }
                        if (dos != null) {
                            try {
                                dos.close();
                            } catch (IOException e) {
                                e.printStackTrace();
                            }
                            dos = null;
                        }
                        if (socketConnection != null) {
                            try {
                                socketConnection.close();
                            } catch (IOException e) {
                                e.printStackTrace();
                            }
                            socketConnection = null;
                        }
                        if (socket != null) {
                            try {
                                socket.close();
                            } catch (IOException e) {
                                e.printStackTrace();
                            }
                            socket = null;
                        }
                    }
                }
            }
        };

        if (listeningThread == null) {
            listeningThread = new Thread(r, "Data Thread");
            listeningThread.start();
        }

    }
}