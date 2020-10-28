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
        try {
             // TODO code application logic here
            Globals.bLoopback = true;
            CShapes.initDefinitions();  // define gears trays object properties
            // Seems to work
            //System.out.print(KittingDemo.dumpDefinitions());
            rcs_robot.hardcode();  // define robot
            rcs_world.hardcode();
            
            crcl = new CRCLClient();
            
            KittingDemo kittingdemo = new KittingDemo(crcl);
         // Seems to work
            //System.out.print(KittingDemo.dumpInstances());
            KittingDemo.fakeFirstOrderLogic();
            // Seems to work
            //System.out.print(KittingDemo.dumpInferences());
 
            // read CRCL status, report
            ///statuskitting();
            
            // test of hard coded fanuc kitting publisher/client
            // fanuckitting();

            int demostate = 0;
            int bozo;
 
            do {
                if (kittingdemo.isDone(demostate)) {
                    demostate = 0;
                }
                bozo = kittingdemo.issueRobotCommands(demostate);
                demostate=bozo;
                Thread.sleep(50);
            } while (demostate >= 0);
        } catch (Exception ex) {
            Logger.getLogger(GwendolynCrclClient.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public static void statuskitting() {
        try {
            crcl.init();
            while (true) {
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
            // THese motions are all relative to the base origin of the robot,
            // which is 0,0,0 mapped from 0.169, -1.140, 0.934191

            crcl.moveTo(new PmCartesian(0.40, -0.06, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.40, -0.06, 0.01),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.setGripper(0.0);
            crcl.doDwell(2.0);
            crcl.moveTo(new PmCartesian(0.40, -0.06, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.41, 0.04, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.41, 0.04, 0.02),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.setGripper(1.);
            crcl.moveTo(new PmCartesian(0.41, 0.04, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.32, -0.06, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.32, -0.06, 0.01),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.setGripper(0.);
            crcl.doDwell(2.);

            crcl.moveTo(new PmCartesian(0.41, 0.12, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.41, 0.12, 0.02),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.setGripper(1.);
            crcl.doDwell(2.);
            crcl.moveTo(new PmCartesian(0.41, 0.12, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.32, -0.14, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.32, -0.14, 0.01),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.setGripper(0.);
            crcl.doDwell(2.);
            crcl.moveTo(new PmCartesian(0.32, -0.14, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.62, 0.05, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.62, 0.05, 0.02),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.setGripper(1.);
            crcl.doDwell(2.);
            crcl.moveTo(new PmCartesian(0.62, 0.05, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.40, -0.14, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.40, -0.14, 0.01),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.setGripper(0.);
            crcl.doDwell(2.);
            crcl.moveTo(new PmCartesian(0.40, -0.14, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.62, 0.13, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.62, 0.13, 0.02),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.setGripper(1.);
            crcl.doDwell(2.);
            crcl.moveTo(new PmCartesian(0.62, 0.13, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.56, -0.07, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.56, -0.07, 0.01),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.setGripper(0.);
            crcl.doDwell(2.);
            crcl.moveTo(new PmCartesian(.56, -0.07, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.53, 0.10, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.53, 0.10, 0.02),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.setGripper(1.);
            crcl.doDwell(2.);
            crcl.moveTo(new PmCartesian(0.53, 0.10, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.56, -0.18, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.56, -0.18, 0.01),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.setGripper(0.);
            crcl.doDwell(2.);
            crcl.moveTo(new PmCartesian(0.56, -0.18, 0.05),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.31, 0.09, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.moveTo(new PmCartesian(0.31, 0.09, 0.02),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);
            crcl.setGripper(1.);
            crcl.doDwell(2.);
            crcl.moveTo(new PmCartesian(0.31, 0.09, 0.06),
                    new PmQuaternion(0.000, 1.000, 0.000, 0.000),
                    false);
            crcl.doDwell(0.1);

        } catch (Exception ex) {
            Logger.getLogger(GwendolynCrclClient.class.getName()).log(Level.SEVERE, null, ex);
        }

    }

}
