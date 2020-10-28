/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package gwendolyncrclclient;
import crcl.base.*;
import java.util.*;
import java.util.logging.Level;
import java.util.logging.Logger;
import rcs.posemath.*;
import rcs.posemath.PmCartesian;
import rcs.posemath.PmQuaternion;
/**
 *
 * @author michalos
 */
public  class rcs_robot {
    
    public static void hardcode() {
        try {
            Retract = new PmPose(new PmCartesian(0.0, 0.0, 0.04), new PmQuaternion(1.0, 0.0, 0.0, 0.0));
            RetractInv = rcs.posemath.Posemath.inv(new PM_POSE(Retract.tran, Retract.rot));
            BasePose = new PmPose(new PmCartesian(-0.169, -1.140, 0.934191), new PmQuaternion(1.0, 0.0, 0.0, 0.0));
            BasePoseInv = rcs.posemath.Posemath.inv(new PM_POSE(BasePose.tran, BasePose.rot));
            QBend= new PmQuaternion(0.0, 1.0, 0.0, 0.0);
            GripperOffset.put("sku_part_large_gear", 
                    new PmPose(new PmCartesian( 0.0, 0.0, -0.015),
                    new PmQuaternion(1.0, 0.0, 0.0, 0.0)));
            GripperOffset.put("sku_part_medium_gear", 
                    new PmPose(new PmCartesian( 0.0, 0.0, -0.015),
                    new PmQuaternion(1.0, 0.0, 0.0, 0.0)));
            GripperOffset.put("sku_part_small_gear", 
                    new PmPose(new PmCartesian( 0.0, 0.0, -0.015),
                    new PmQuaternion(1.0, 0.0, 0.0, 0.0)));
            
        } catch (Exception ex) {
            Logger.getLogger(CRCLClient.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
     /**
     * @brief isBusy read the current status to determine if
     * the robot is still busy executing a command
     * @return true if busy
     */
    public static boolean isBusy()
    {
        // FIXME: read crcl status for done
        return false;
    }

    
    // latest update
    //crcl_rosmsgs::CrclStatusMsg::ConstPtr _status;

    // Canned poses
    public static PmPose Retract;         /**< pose offset for retract */
    public static PmPose RetractInv; /**< inverse pose offset for retract */
    public static PmQuaternion QBend; /**< rotation to achieve pose rotation for grasping */
    public static PmPose currentPose;
    public static PmPose BasePose;
    public static PmPose BasePoseInv;
    public static java.util.Map<String, PmPose> GripperOffset= new  HashMap<>();       
/// gripper offset for each part


    // CRCL status numbers
    public static long curStatusCmdId;
    public static CommandStateEnumType crclCommandStatus;
    public static long latestCmdId;
}
