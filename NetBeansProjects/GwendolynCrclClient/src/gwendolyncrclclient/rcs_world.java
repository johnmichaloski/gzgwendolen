/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package gwendolyncrclclient;

import java.util.*;
import java.util.logging.Level;
import java.util.logging.Logger;
import rcs.posemath.PmPose;
import rcs.posemath.PmCartesian;
import rcs.posemath.PmQuaternion;

/**
 *
 * @author michalos
 */
public class rcs_world {

    /// gripper offset for container slot
    public static Map<String, PmPose> slotoffset = new HashMap<>();

    public static void hardcode() {
        try {
            slotoffset.put("vessel", new PmPose(new PmCartesian(0.00, 0.00, -0.035),
                    new PmQuaternion(1.0, 0.0, 0.0, 0.0)));
        } catch (Exception ex) {
            Logger.getLogger(CRCLClient.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

}
