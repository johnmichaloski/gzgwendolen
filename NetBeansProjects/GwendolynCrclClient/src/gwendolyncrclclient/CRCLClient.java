/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package gwendolyncrclclient;

/* 
 * Compile with :
 * javac -cp ../../crcl4java-utils/target/crcl4java-utils-1.0-SNAPSHOT-jar-with-dependencies.jar CRCLClient.java
 * 
 * Run with:
 * java -cp ../../crcl4java-utils/target/crcl4java-utils-1.0-SNAPSHOT-jar-with-dependencies.jar:.  CRCLClient
 * 
 */
import crcl.base.*;
import static crcl.utils.CRCLPosemath.point;
import static crcl.utils.CRCLPosemath.pose;
import static crcl.utils.CRCLPosemath.vector;
import static crcl.utils.CRCLPosemath.toPose;

import java.math.BigInteger;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import crcl.utils.CRCLSocket;
import java.util.concurrent.locks.ReentrantLock;
// Pose math
import rcs.posemath.PmCartesian;
import rcs.posemath.PmEulerZyx;
import rcs.posemath.PmException;
import rcs.posemath.PmHomogeneous;
import rcs.posemath.PmPose;
import rcs.posemath.PmRotationMatrix;
import rcs.posemath.PmRotationVector;
import rcs.posemath.PmRpy;
import rcs.posemath.Posemath;
import rcs.posemath.PmQuaternion;

/**
 * Example CRCL client written in java.
 *
 * @author Will Shackleford{@literal <william.shackleford@nist.gov> }
 */
public class CRCLClient implements Runnable {

    public CRCLClient() {
        try {

            // Connect to the server
            s = new CRCLSocket("localhost", CRCLSocket.DEFAULT_PORT);

            // Create an instance to wrap all commands.
            instance = new CRCLCommandInstanceType();
            id = 0;

        } catch (Exception ex) {
            Logger.getLogger(CRCLClient.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public void startStatus() {
        Thread statusthread = new Thread();
        statusthread.start();
    }

    public int waitDone(long commandid) {

        long IDback = 1;
        CommandStatusType cmdStat = null;
        try {
            do {
                // Create and send getStatus request.
//                GetStatusType getStat = new GetStatusType();
//                getStat.setCommandID(incrementId(1));
//                instance.setCRCLCommand(getStat);
//                s.writeCommand(instance);

                // Read status from server
                CRCLStatusType stat = s.readStatus();

                // Print out the status details.
                cmdStat = stat.getCommandStatus();
                IDback = cmdStat.getCommandID();
                System.out.println("Status:");
                System.out.println("CommandID = " + IDback);
                System.out.println("State = " + cmdStat.getCommandState());
                PointType pt = stat.getPoseStatus().getPose().getPoint();
                System.out.println("pose = " + pt.getX() + "," + pt.getY() + "," + pt.getZ());
                JointStatusesType jst = stat.getJointStatuses();
                if (null != jst) {
                    List<JointStatusType> l = jst.getJointStatus();
                    System.out.println("Joints:");
                    for (JointStatusType js : l) {
                        System.out.println("Num=" + js.getJointNumber() + " Pos=" + js.getJointPosition());
                    }
                }
            } while (!(IDback == commandid) || cmdStat.getCommandState().equals(CommandStateEnumType.CRCL_WORKING));
        } catch (Exception ex) {
            Logger.getLogger(CRCLClient.class.getName()).log(Level.SEVERE, null, ex);
            return -1;
        }
        return 0;
    }
    
    

    @Override
    public void run() {
        try {
            long IDback = 1;
            CommandStatusType cmdStat = null;
            do {
                // Create and send getStatus request.
                GetStatusType getStat = new GetStatusType();
                getStat.setCommandID(incrementId(1));
                instance.setCRCLCommand(getStat);
                s.writeCommand(instance);

                // Read status from server
                CRCLStatusType stat = s.readStatus();

                // Print out the status details.
                cmdStat = stat.getCommandStatus();
                IDback = cmdStat.getCommandID();
                System.out.println("Status:");
                System.out.println("CommandID = " + IDback);
                System.out.println("State = " + cmdStat.getCommandState());
                PointType pt = stat.getPoseStatus().getPose().getPoint();
                System.out.println("pose = " + pt.getX() + "," + pt.getY() + "," + pt.getZ());
                JointStatusesType jst = stat.getJointStatuses();
                if (null != jst) {
                    List<JointStatusType> l = jst.getJointStatus();
                    System.out.println("Joints:");
                    for (JointStatusType js : l) {
                        System.out.println("Num=" + js.getJointNumber() + " Pos=" + js.getJointPosition());
                    }
                }
            } while (!IsQuit());
        } catch (Exception ex) {
            Logger.getLogger(CRCLClient.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public void setLengthUnitsType(String units) {
        try {
            // Create and send init command.
            SetLengthUnitsType lenuits = new SetLengthUnitsType();
            lenuits.setCommandID(incrementId(1));
            if (units.toLowerCase().startsWith("meter")) {
                lenuits.setUnitName(LengthUnitEnumType.METER);
            }
            instance.setCRCLCommand(lenuits);
            s.writeCommand(instance);
            waitDone(lenuits.getCommandID());
        } catch (Exception ex) {
            Logger.getLogger(CRCLClient.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
    public void dwell(double seconds) {
        try {
            // Create and send init command.
            DwellType dwell = new DwellType();
            dwell.setCommandID(incrementId(1));
            dwell.setDwellTime(seconds);
            instance.setCRCLCommand(dwell);
            s.writeCommand(instance);
            waitDone(dwell.getCommandID());
        } catch (Exception ex) {
            Logger.getLogger(CRCLClient.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public void setGripper(double position) {
        try {
            // FIXME: check if <0 or >1
            // Create and send init command.
            SetEndEffectorType ee = new SetEndEffectorType();
            ee.setCommandID(incrementId(1));
            ee.setSetting(position);
            instance.setCRCLCommand(ee);
            s.writeCommand(instance);
            waitDone(ee.getCommandID());

        } catch (Exception ex) {
            Logger.getLogger(CRCLClient.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public void init() {
        try {
            // Create and send init command.
            InitCanonType init = new InitCanonType();
            init.setCommandID(incrementId(1));
            instance.setCRCLCommand(init);
            s.writeCommand(instance);
            waitDone(init.getCommandID());
       } catch (Exception ex) {
            Logger.getLogger(CRCLClient.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
    public void status() {
        try {
            // Create and send getStatus request.
            GetStatusType getStat = new GetStatusType();
            getStat.setCommandID(incrementId(1));
            instance.setCRCLCommand(getStat);
            s.writeCommand(instance);

            // Read status from server
            CRCLStatusType stat = s.readStatus();
            List<ModelsStatusType> models = stat.getModelStatus();
            for (int i = 0; i < models.size(); i++) 
            {
                ModelsStatusType model= models.get(i);
                String str=model.getName();
                str += ","+ model.getPose().getPoint().getX();
                str += ","+ model.getPose().getPoint().getY();
                str += ","+ model.getPose().getPoint().getZ();
                System.out.println(str);
            }
            
            
            
        } catch (Exception ex) {
            Logger.getLogger(CRCLClient.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public void move(PmCartesian p, PmQuaternion q, boolean bstraight) {
        try {
            PmPose pmpose = new PmPose(p, q);
            PoseType pose = toPose(pmpose);
            // Create and send MoveTo command.
            MoveToType moveTo = new MoveToType();
            moveTo.setCommandID(incrementId(1));
            //PoseType pose = p; // pose(point(0.65, 0.05, 0.15), vector(1, 0, 0), vector(0, 0, 1));
            moveTo.setEndPosition(pose);
            moveTo.setMoveStraight(bstraight);
            instance.setCRCLCommand(moveTo);
            s.writeCommand(instance, true);
            waitDone(moveTo.getCommandID());

        } catch (Exception ex) {
            Logger.getLogger(CRCLClient.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public void test() {
        try {

            // Create and send init command.
            InitCanonType init = new InitCanonType();
            init.setCommandID(incrementId(1));
            instance.setCRCLCommand(init);
            s.writeCommand(instance);

            // Create and send MoveTo command.
            MoveToType moveTo = new MoveToType();
            moveTo.setCommandID(incrementId(1));
            PoseType pose = pose(point(0.65, 0.05, 0.15), vector(1, 0, 0), vector(0, 0, 1));
            moveTo.setEndPosition(pose);
            moveTo.setMoveStraight(false);
            instance.setCRCLCommand(moveTo);
            s.writeCommand(instance, true);

            MessageType message = new MessageType();
            message.setCommandID(incrementId(1));
            message.setMessage("some message");
            long IDback = 1;
            CommandStatusType cmdStat = null;

            do {
                // Create and send getStatus request.
                GetStatusType getStat = new GetStatusType();
                getStat.setCommandID(incrementId(1));
                instance.setCRCLCommand(getStat);
                s.writeCommand(instance);

                // Read status from server
                CRCLStatusType stat = s.readStatus();

                // Print out the status details.
                cmdStat = stat.getCommandStatus();
                IDback = cmdStat.getCommandID();
                System.out.println("Status:");
                System.out.println("CommandID = " + IDback);
                System.out.println("State = " + cmdStat.getCommandState());
                PointType pt = stat.getPoseStatus().getPose().getPoint();
                System.out.println("pose = " + pt.getX() + "," + pt.getY() + "," + pt.getZ());
                JointStatusesType jst = stat.getJointStatuses();
                if (null != jst) {
                    List<JointStatusType> l = jst.getJointStatus();
                    System.out.println("Joints:");
                    for (JointStatusType js : l) {
                        System.out.println("Num=" + js.getJointNumber() + " Pos=" + js.getJointPosition());
                    }
                }
            } while (!(IDback == moveTo.getCommandID()) || cmdStat.getCommandState().equals(CommandStateEnumType.CRCL_WORKING));
        } catch (Exception ex) {
            Logger.getLogger(CRCLClient.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
    CRCLSocket s;
    CRCLCommandInstanceType instance;
    static long id;
    static boolean bQuit = false;
    static ReentrantLock counterLock = new ReentrantLock(true); // enable fairness policy

    static long incrementId(int n) {
        // Always good practice to enclose locks in a try-finally block
        try {
            counterLock.lock();
            id = id + n;
        } finally {
            counterLock.unlock();
        }
        return id;
    }

    static boolean IsQuit() {
        boolean bflag;
        try {
            counterLock.lock();
            bflag = bQuit;
        } finally {
            counterLock.unlock();
        }
        return bflag;
    }

    static void setQuit(boolean bflag) {
        try {
            counterLock.lock();
            bQuit = bflag;
        } finally {
            counterLock.unlock();
        }
    }
}
