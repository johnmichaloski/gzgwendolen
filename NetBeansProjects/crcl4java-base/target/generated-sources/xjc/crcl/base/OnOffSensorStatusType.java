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
 *                 SensorStatusType is derived from SensorStatusType
 *                 SensorStatusType reports the status of one on/off sensor.
 *                 An instance of SensorStatusType has the following elements:
 *                 Name (inherited, optional)
 *                 SensorID (inherited)
 *                 SensorParameterSetting (inherited, optional)
 *                 On.
 * 
 *             
 * 
 * <p>Java class for OnOffSensorStatusType complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="OnOffSensorStatusType"&gt;
 *   &lt;complexContent&gt;
 *     &lt;extension base="{}SensorStatusType"&gt;
 *       &lt;sequence&gt;
 *         &lt;element name="On" type="{http://www.w3.org/2001/XMLSchema}boolean"/&gt;
 *       &lt;/sequence&gt;
 *     &lt;/extension&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "OnOffSensorStatusType", propOrder = {
    "on"
})
public class OnOffSensorStatusType
    extends SensorStatusType
{

    @XmlElement(name = "On")
    protected boolean on;

    /**
     * Gets the value of the on property.
     * 
     */
    public boolean isOn() {
        return on;
    }

    /**
     * Sets the value of the on property.
     * 
     */
    public void setOn(boolean value) {
        this.on = value;
    }

}