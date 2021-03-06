//
// This file was generated by the JavaTM Architecture for XML Binding(JAXB) Reference Implementation, v2.3.0 
// See <a href="https://javaee.github.io/jaxb-v2/">https://javaee.github.io/jaxb-v2/</a> 
// Any modifications to this file will be lost upon recompilation of the source schema. 
// Generated on: 2020.09.21 at 09:41:19 AM EDT 
//


package crcl.base;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlType;


/**
 * 
 *                 ActuateJointType is derived from DataThingType.
 *                 An instance of ActuateJointType has the following elements:
 *                 Name (inherited, optional)
 *                 JointNumber
 *                 JointPosition
 *                 JointDetails.
 * 
 *                 JointPosition is the target position for the joint. JointDetails
 *                 provides either (1) the speed and acceleration to use in getting to
 *                 the position or (2) the force or torque and rate of change of force
 *                 or torque to use in getting to the position.
 *             
 * 
 * <p>Java class for ActuateJointType complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="ActuateJointType"&gt;
 *   &lt;complexContent&gt;
 *     &lt;extension base="{}DataThingType"&gt;
 *       &lt;sequence&gt;
 *         &lt;element name="JointNumber" type="{http://www.w3.org/2001/XMLSchema}int"/&gt;
 *         &lt;element name="JointPosition" type="{http://www.w3.org/2001/XMLSchema}double"/&gt;
 *         &lt;element name="JointDetails" type="{}JointDetailsType"/&gt;
 *       &lt;/sequence&gt;
 *     &lt;/extension&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "ActuateJointType", propOrder = {
    "jointNumber",
    "jointPosition",
    "jointDetails"
})
public class ActuateJointType
    extends DataThingType
{

    @XmlElement(name = "JointNumber")
    protected int jointNumber;
    @XmlElement(name = "JointPosition")
    protected double jointPosition;
    @XmlElement(name = "JointDetails", required = true)
    protected JointDetailsType jointDetails;

    /**
     * Gets the value of the jointNumber property.
     * 
     */
    public int getJointNumber() {
        return jointNumber;
    }

    /**
     * Sets the value of the jointNumber property.
     * 
     */
    public void setJointNumber(int value) {
        this.jointNumber = value;
    }

    /**
     * Gets the value of the jointPosition property.
     * 
     */
    public double getJointPosition() {
        return jointPosition;
    }

    /**
     * Sets the value of the jointPosition property.
     * 
     */
    public void setJointPosition(double value) {
        this.jointPosition = value;
    }

    /**
     * Gets the value of the jointDetails property.
     * 
     * @return
     *     possible object is
     *     {@link JointDetailsType }
     *     
     */
    public JointDetailsType getJointDetails() {
        return jointDetails;
    }

    /**
     * Sets the value of the jointDetails property.
     * 
     * @param value
     *     allowed object is
     *     {@link JointDetailsType }
     *     
     */
    public void setJointDetails(JointDetailsType value) {
        this.jointDetails = value;
    }

}
