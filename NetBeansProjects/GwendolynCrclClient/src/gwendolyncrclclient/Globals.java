/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package gwendolyncrclclient;

import java.util.concurrent.locks.ReentrantLock;
import crcl.base.*;
import java.util.StringTokenizer;
import java.util.logging.Level;
import java.util.logging.Logger;
import rcs.posemath.PmPose;
import rcs.posemath.PmQuaternion;

/**
 *
 * @author michalos
 */
public class Globals {

    public static boolean bReadAllInstances;
    public static ReentrantLock mutex = new ReentrantLock();
    public static long latestCmdId;
    public static long curStatusCmdId;
    public static CommandStateEnumType crclCommandStatus;
    public static boolean bLoopback = true;

    public static PmPose convertTranPose(String tran) {
        PmPose p = new PmPose();
        try {
            StringTokenizer st = new StringTokenizer(tran, ",");
            p.tran.x = Double.parseDouble(st.nextToken());
            p.tran.y = Double.parseDouble(st.nextToken());
            p.tran.z = Double.parseDouble(st.nextToken());
            p.rot = new PmQuaternion(1., 0.0, 0.0, 0.0);
        } catch (Exception ex) {
            Logger.getLogger(CRCLClient.class.getName()).log(Level.SEVERE, null, ex);
            return null;
        }
        return p;
    }

}
