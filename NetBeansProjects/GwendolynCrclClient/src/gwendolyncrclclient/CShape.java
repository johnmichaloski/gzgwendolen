package gwendolyncrclclient;
import java.util.*;  
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


/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
/**
 *
 * @author michalos
 */
public class CShape implements Cloneable {

    public Object clone() throws CloneNotSupportedException {
        return super.clone();
    }
    // public static Vector<CShape> now_instances = new Vector<CShape>();
    public static class inference_type {

        public String name;
        public String type;
        public String state;
        public String location;

        public String parent;
        public String slot;
    };
   public Vector<inference_type> inferences = new Vector<inference_type>();

    public CShape() {
       _contains = new ArrayList< CShape> ();
   } 

    public CShape(String name,
            String type,
            PmPose pose,
            CShape parent) {
        _name = name;
        _type = type;
        _location = pose;
        _parent = parent;
       _contains = new ArrayList< CShape> ();
    }
    
    public CShape(String name,
            String type,
            PmPose pose) {
        _name = name;
        _type = type;
        _location = pose;
       _updated="";
       _contains = new ArrayList< CShape> ();
   }
    public String name() {
        return _name;
    }

    public void setName(String name) {
        _name = name;
    }

    public String type() {
        return _type;
    }

    public void setType(String type) {
        _type = type;
    }

    public PmPose centroid() {
        return _location;
    }

    public void setCentroid(PmPose centroid) {
        _location = centroid;
    }

          public String _updated;
  private String _name;
    private String _type;  // gear, vessel, kit for now
    private CShape _parent;
    public PmPose _dimensions;  // size of xyz in meters
    public PmPose _location;  // location of xyz bottom of object.
    //private PmCartesian _bounding_box[2];
    private double _gripperCloseWidth; // displacement of gripper closed in meters
    public List< CShape> _contains;  // slots in tray
  //  public Map<String, String> _attributes;
  //  public Map<String, Map<String, String>> _properties;
    public double _height; // assume upright!

    public inference_type findInference(String name)
    {
        for(inference_type inference : inferences)
        {
            if(inference.name.equalsIgnoreCase(name))
                return inference;
        }
        return null;
    
    }
    public boolean isKit() {
        if (_name.indexOf("kit") != -1) {
            return true;
        }
        return false;
    }

    public boolean isVessel() // tray
    {
        // kit also has vessel in its name
        if (_name.indexOf("gear_vessel") != -1) {
            return true;
        }
        return false;
    }

    public boolean isGear() {
        if (_name.indexOf("part") != -1) {
            return true;
        }
        return false;
    }

    public boolean isSkuPart() {
        // If not a sku skip
        if (_name.indexOf("sku") != -1) {
            return true;
        }
        return false;
    }

    public boolean inMyWorld() {
        return true;
    }
    public String dumpShape()
    {
        String s;
        s=name() + ":"+ type() + ":"+ KittingDemo.dumpPmPose(_location)+"\n";
        for(CShape slot : this._contains)
        {
               s+="\t"+slot.name() + ":"+ slot.type() + ":"+ KittingDemo.dumpPmPose(slot._location)+"\n";            
        }
        return s;
    }

};

