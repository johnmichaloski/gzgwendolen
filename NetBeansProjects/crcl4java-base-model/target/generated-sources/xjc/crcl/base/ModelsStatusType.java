//
// This file was generated by the JavaTM Architecture for XML Binding(JAXB) Reference Implementation, v2.3.0 
// See <a href="https://javaee.github.io/jaxb-v2/">https://javaee.github.io/jaxb-v2/</a> 
// Any modifications to this file will be lost upon recompilation of the source schema. 
// Generated on: 2020.09.26 at 08:19:21 PM EDT 
//


package crcl.base;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlType;


/**
 * 
 *                 ModelsStatusType reports on the logical models in the environment.
 *                 THe basic logical model is name and pose. Inferred properties
 *                 about a model are also 
 *                 An instance of ModelsStatusType has the following elements:
 *                 Name.
 *                 Pose as a pose type (xyz x rotation, z rotation)
 *                 Twist (optional) 
 *                 Wrench. (optional)
 *                 Properties is a sequence of properties. Each property describes
 *                 either a slot or a gear parent. For a slot, name, type, location,
 *                 and state given. Location is the reoridented position based on the orattion 
 *                 of the parent tray. For a part, the property gives the parent tray name
 *                 hosting the gear (if one) and the slot in the parent tray.
 *                 
 *             
 * 
 * <p>Java class for ModelsStatusType complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="ModelsStatusType"&gt;
 *   &lt;complexContent&gt;
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType"&gt;
 *       &lt;sequence&gt;
 *         &lt;element name="Name" type="{http://www.w3.org/2001/XMLSchema}string"/&gt;
 *         &lt;element name="Pose" type="{}PoseType"/&gt;
 *         &lt;element name="Twist" type="{}TwistType" minOccurs="0"/&gt;
 *         &lt;element name="Wrench" type="{}WrenchType" minOccurs="0"/&gt;
 *         &lt;sequence minOccurs="0"&gt;
 *           &lt;element name="Properties" type="{}MapType"/&gt;
 *         &lt;/sequence&gt;
 *       &lt;/sequence&gt;
 *     &lt;/restriction&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "ModelsStatusType", propOrder = {
    "name",
    "pose",
    "twist",
    "wrench",
    "properties"
})
public class ModelsStatusType {

    @XmlElement(name = "Name", required = true)
    protected String name;
    @XmlElement(name = "Pose", required = true)
    protected PoseType pose;
    @XmlElement(name = "Twist")
    protected TwistType twist;
    @XmlElement(name = "Wrench")
    protected WrenchType wrench;
    @XmlElement(name = "Properties")
    protected MapType properties;

    /**
     * Gets the value of the name property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    public String getName() {
        return name;
    }

    /**
     * Sets the value of the name property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    public void setName(String value) {
        this.name = value;
    }

    /**
     * Gets the value of the pose property.
     * 
     * @return
     *     possible object is
     *     {@link PoseType }
     *     
     */
    public PoseType getPose() {
        return pose;
    }

    /**
     * Sets the value of the pose property.
     * 
     * @param value
     *     allowed object is
     *     {@link PoseType }
     *     
     */
    public void setPose(PoseType value) {
        this.pose = value;
    }

    /**
     * Gets the value of the twist property.
     * 
     * @return
     *     possible object is
     *     {@link TwistType }
     *     
     */
    public TwistType getTwist() {
        return twist;
    }

    /**
     * Sets the value of the twist property.
     * 
     * @param value
     *     allowed object is
     *     {@link TwistType }
     *     
     */
    public void setTwist(TwistType value) {
        this.twist = value;
    }

    /**
     * Gets the value of the wrench property.
     * 
     * @return
     *     possible object is
     *     {@link WrenchType }
     *     
     */
    public WrenchType getWrench() {
        return wrench;
    }

    /**
     * Sets the value of the wrench property.
     * 
     * @param value
     *     allowed object is
     *     {@link WrenchType }
     *     
     */
    public void setWrench(WrenchType value) {
        this.wrench = value;
    }

    /**
     * Gets the value of the properties property.
     * 
     * @return
     *     possible object is
     *     {@link MapType }
     *     
     */
    public MapType getProperties() {
        return properties;
    }

    /**
     * Sets the value of the properties property.
     * 
     * @param value
     *     allowed object is
     *     {@link MapType }
     *     
     */
    public void setProperties(MapType value) {
        this.properties = value;
    }

}
