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
import crcl.utils.CRCLPosemath;

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

    public static CShapes scene;

    public CRCLClient() {
        try {
            
 
            // Connect to the server
            if (!Globals.bLoopback)
            {
                s = new CRCLSocket("localhost", CRCLSocket.DEFAULT_PORT);
            }
            else
            {
                KittingDemo.fakeSetup();
             }

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
        if (Globals.bLoopback) {
            try {
                for(int i=0; i< 1; i++)
                     Thread.sleep(1000);
                Globals.mutex.lock();
                Globals.curStatusCmdId = Globals.latestCmdId;
                Globals.crclCommandStatus = CommandStateEnumType.CRCL_DONE ; // done

            } 
            catch (Exception ex) {
            
                
            }
            finally {
                Globals.mutex.unlock();
            }
            return 0;
        }
        try {
            do {
                // Create and send getStatus request.
//                GetStatusType getStat = new GetStatusType();
//                getStat.setCommandID(incrementId(1));
//                instance.setCRCLCommand(getStat);
//                issueCrclCommand();

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
                 List<ModelsStatusType> models = stat.getModelStatus();
                 for(ModelsStatusType model : models)
                 {
                     String name = model.getName();
                     PmPose pose = CRCLPosemath.toPmPose(model.getPose());
                     CShapes.storeInstance(name,pose);
                 }
             
                try {
                    Globals.mutex.lock();
                    Globals.curStatusCmdId = cmdStat.getCommandID();
                    Globals.crclCommandStatus=stat.getCommandStatus().getCommandState();

                } finally {
                    Globals.mutex.unlock();
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
                issueCrclCommand();

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
                // now parse any all models
                
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
            issueCrclCommand();
            waitDone(lenuits.getCommandID());
        } catch (Exception ex) {
            Logger.getLogger(CRCLClient.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
    public void doDwell(double seconds) {
        try {
            // Create and send init command.
            DwellType dwell = new DwellType();
            dwell.setCommandID(incrementId(1));
            dwell.setDwellTime(seconds);
            instance.setCRCLCommand(dwell);
            issueCrclCommand();
            waitDone(dwell.getCommandID());
        } catch (Exception ex) {
            Logger.getLogger(CRCLClient.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public void closeGripper() {
        setGripper(0.0);
    }

    public void openGripper() {
        setGripper(1.0);
    }
    
    public void setGripper(double position) {
        try {
            // FIXME: check if <0 or >1
            // Create and send init command.
            SetEndEffectorType ee = new SetEndEffectorType();
            ee.setCommandID(incrementId(1));
            ee.setSetting(position);
            instance.setCRCLCommand(ee);
            issueCrclCommand();
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
            issueCrclCommand();
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
            issueCrclCommand();

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
   public void moveTo(PmCartesian p, PmQuaternion q) {
        moveTo(p, q, true);
    }
    public void moveTo(PmPose p) {
        moveTo(p.tran, p.rot, true);
    }
    public void moveTo(PmCartesian p, PmQuaternion q, boolean bstraight) {
        try {
            PmPose pmpose = new PmPose(p, q);
            lastmoveto=pmpose.clone();
            PoseType pose = toPose(pmpose);
            // Create and send MoveTo command.
            MoveToType moveTo = new MoveToType();
            moveTo.setCommandID(incrementId(1));
            //PoseType pose = p; // pose(point(0.65, 0.05, 0.15), vector(1, 0, 0), vector(0, 0, 1));
            moveTo.setEndPosition(pose);
            moveTo.setMoveStraight(bstraight);
            instance.setCRCLCommand(moveTo);
            //s.writeCommand(instance, true);
            issueCrclCommand();
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
            issueCrclCommand();

            // Create and send MoveTo command.
            MoveToType moveTo = new MoveToType();
            moveTo.setCommandID(incrementId(1));
            PoseType pose = pose(point(0.65, 0.05, 0.15), vector(1, 0, 0), vector(0, 0, 1));
            moveTo.setEndPosition(pose);
            moveTo.setMoveStraight(false);
            instance.setCRCLCommand(moveTo);
            // true is additional validation
            //s.writeCommand(instance, true);
            issueCrclCommand();


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
                issueCrclCommand();

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

    public static long incrementId(int n) {
        // Always good practice to enclose locks in a try-finally block
        try {
            Globals.mutex.lock();
            id = id + n;
            Globals.latestCmdId = id;
            rcs_robot.latestCmdId = id;
        } finally {
            Globals.mutex.unlock();
        }
        return id;
    }

    public static boolean IsQuit() {
        boolean bflag;
        try {
            counterLock.lock();
            bflag = bQuit;
        } finally {
            counterLock.unlock();
        }
        return bflag;
    }

    public static void setQuit(boolean bflag) {
        try {
            counterLock.lock();
            bQuit = bflag;
        } finally {
            counterLock.unlock();
        }
    }
    
       public void issueCrclCommand() {
        if (Globals.bLoopback) {
            // we don't actually send to socket if loopback
            System.out.print("issueCrclCommand "+instance.getCRCLCommand().getClass());
             if(instance.getCRCLCommand().getClass().toString().equalsIgnoreCase("class crcl.base.MoveToType"))
            {
                try {
                    MoveToType m = (MoveToType) instance.getCRCLCommand();
                    System.out.print(KittingDemo.dumpPmPose(lastmoveto));
                   // lastmoveto = CRCLPosemath.toPmPose(m.getEndPosition());
                } catch (Exception ex) {
                    Logger.getLogger(CRCLClient.class.getName()).log(Level.SEVERE, null, ex);
                }
                
            }
             else if(instance.getCRCLCommand().getClass().toString().equalsIgnoreCase("class crcl.base.DwellType"))
            {
                DwellType d = (DwellType) instance.getCRCLCommand();
                System.out.print(":"+d.getDwellTime());
                
            }
             else  if(instance.getCRCLCommand().getClass().toString().equalsIgnoreCase("class crcl.base.SetEndEffectorType"))
            {
                SetEndEffectorType e = (SetEndEffectorType) instance.getCRCLCommand();
                if (e.getSetting() == 0.0) {
                    System.out.print(" close");
                    graspedgear=KittingDemo.closestGear(lastmoveto);
                    if(graspedgear == null)
                    {
                        System.out.println("No grasping gear found");
                    }
                } else {
                    // update graspgear location
                    System.out.print(" open");
                    if(graspedgear != null)
                    {
                        CShape.inference_type inference = KittingDemo._kit.findInference(KittingDemo._openslot.name());
                        PmPose slotloc = Globals.convertTranPose(inference.location);

                        CShape gear = CShapes.findInstance(CShapes.instances, graspedgear.name());
                        gear._location=slotloc; // offset?
                        
                        System.out.print(KittingDemo.dumpPmPose( gear._location));
                        KittingDemo.fakeFirstOrderLogic();
                        System.out.print(KittingDemo.dumpInferences());
                    }
                   
                }
            }
            System.out.println();
        } else {
            try {
                issueCrclCommand();
            } catch (Exception ex) {
                System.out.println("Failed issueCrclCommand "+instance.getName());
                Logger.getLogger(CRCLClient.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
    }
       
    public static PmPose lastmoveto=new PmPose();
    public static CShape graspedgear;
    // varaibles
    public static double _mygraspdwell=2.0;
    public static double _mydwell=0.1;
    CRCLSocket s;
    CRCLCommandInstanceType instance;
    static long id=0;
    static boolean bQuit = false;
    static ReentrantLock counterLock = new ReentrantLock(true); // enable fairness policy
}
