/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package gwendolyncrclclient;
// Pose math

import java.util.logging.Level;
import java.util.logging.Logger;
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
 *
 * @author michalos
 */
public class GwendolynCrclClient {

    public static CRCLClient crcl;

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        // TODO code application logic here
        crcl = new CRCLClient();
        //crcl.test(); // kinda works
        
        ///statuskitting();
        
        // test of fanuc kitting publisher/client
        fanuckitting();
    }
    public static void statuskitting() {
        try {
               crcl.init();
               while(true)
               {
                   crcl.status();
                   Thread.sleep(4000);
               }
         
        
            } catch (Exception ex) {
            Logger.getLogger(GwendolynCrclClient.class.getName()).log(Level.SEVERE, null, ex);
        }

    }
    public static void fanuckitting() {
        try {
            crcl.init();
            crcl.setLengthUnitsType("METER");
            
            System.out.println("move = 0.40, -0.06, 0.05 ");

            crcl.move(new PmCartesian(0.40, -0.06, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.40, -0.06, 0.01),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.setGripper(0.0);
            crcl.dwell(2.0);
            crcl.move(new PmCartesian(0.40, -0.06, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.41, 0.04, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.41, 0.04, 0.02),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.setGripper(1.);
            crcl.move(new PmCartesian(0.41, 0.04, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.32, -0.06, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.32, -0.06, 0.01),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.setGripper(0.);
            crcl.dwell(2.);

            crcl.move(new PmCartesian(0.41, 0.12, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.41, 0.12, 0.02),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.setGripper(1.);
            crcl.dwell(2.);
            crcl.move(new PmCartesian(0.41, 0.12, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.32, -0.14, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.32, -0.14, 0.01),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.setGripper(0.);
            crcl.dwell(2.);
            crcl.move(new PmCartesian(0.32, -0.14, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.62, 0.05, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.62, 0.05, 0.02),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.setGripper(1.);
            crcl.dwell(2.);
            crcl.move(new PmCartesian(0.62, 0.05, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.40, -0.14, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.40, -0.14, 0.01),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.setGripper(0.);
            crcl.dwell(2.);
            crcl.move(new PmCartesian(0.40, -0.14, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.62, 0.13, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.62, 0.13, 0.02),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.setGripper(1.);
            crcl.dwell(2.);
            crcl.move(new PmCartesian(0.62, 0.13, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.56, -0.07, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.56, -0.07, 0.01),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.setGripper(0.);
            crcl.dwell(2.);
            crcl.move(new PmCartesian(.56, -0.07, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.53, 0.10, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.53, 0.10, 0.02),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.setGripper(1.);
            crcl.dwell(2.);
            crcl.move(new PmCartesian(0.53, 0.10, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.56, -0.18, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.56, -0.18, 0.01),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.setGripper(0.);
            crcl.dwell(2.);
            crcl.move(new PmCartesian(0.56, -0.18, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.31, 0.09, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.move(new PmCartesian(0.31, 0.09, 0.02),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);
            crcl.setGripper(1.);
            crcl.dwell(2.);
            crcl.move(new PmCartesian(0.31, 0.09, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.dwell(0.1);

        } catch (Exception ex) {
            Logger.getLogger(GwendolynCrclClient.class.getName()).log(Level.SEVERE, null, ex);
        }

    }

}
