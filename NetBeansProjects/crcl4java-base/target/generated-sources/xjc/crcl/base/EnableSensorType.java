//
// This file was generated by the JavaTM Architecture for XML Binding(JAXB) Reference Implementation, v2.3.0 
// See <a href="https://javaee.github.io/jaxb-v2/">https://javaee.github.io/jaxb-v2/</a> 
// Any modifications to this file will be lost upon recompilation of the source schema. 
// Generated on: 2020.09.21 at 09:41:19 AM EDT 
//


package crcl.base;

import java.util.ArrayList;
import java.util.List;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlSchemaType;
import javax.xml.bind.annotation.XmlType;
import javax.xml.bind.annotation.adapters.CollapsedStringAdapter;
import javax.xml.bind.annotation.adapters.XmlJavaTypeAdapter;


/**
 * 
 *                 EnableSensorType is derived from MiddleCommandType.
 *                 An instance of EnableSensorType has the following elements:
 *                 Name (inherited, optional)
 *                 CommandID (inherited)
 *                 SensorId
 *                 SensorOptions (optional).
 * 
 *                 This enables the reporting of a sensor.
 *             
 * 
 * <p>Java class for EnableSensorType complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="EnableSensorType"&gt;
 *   &lt;complexContent&gt;
 *     &lt;extension base="{}MiddleCommandType"&gt;
 *       &lt;sequence&gt;
 *         &lt;element name="SensorID" type="{http://www.w3.org/2001/XMLSchema}token"/&gt;
 *         &lt;element name="SensorOption" type="{}ParameterSettingType" maxOccurs="unbounded" minOccurs="0"/&gt;
 *       &lt;/sequence&gt;
 *     &lt;/extension&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "EnableSensorType", propOrder = {
    "sensorID",
    "sensorOption"
})
public class EnableSensorType
    extends MiddleCommandType
{

    @XmlElement(name = "SensorID", required = true)
    @XmlJavaTypeAdapter(CollapsedStringAdapter.class)
    @XmlSchemaType(name = "token")
    protected String sensorID;
    @XmlElement(name = "SensorOption")
    protected List<ParameterSettingType> sensorOption;

    /**
     * Gets the value of the sensorID property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    public String getSensorID() {
        return sensorID;
    }

    /**
     * Sets the value of the sensorID property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    public void setSensorID(String value) {
        this.sensorID = value;
    }

    /**
     * Gets the value of the sensorOption property.
     * 
     * <p>
     * This accessor method returns a reference to the live list,
     * not a snapshot. Therefore any modification you make to the
     * returned list will be present inside the JAXB object.
     * This is why there is not a <CODE>set</CODE> method for the sensorOption property.
     * 
     * <p>
     * For example, to add a new item, do as follows:
     * <pre>
     *    getSensorOption().add(newItem);
     * </pre>
     * 
     * 
     * <p>
     * Objects of the following type(s) are allowed in the list
     * {@link ParameterSettingType }
     * 
     * 
     */
    public List<ParameterSettingType> getSensorOption() {
        if (sensorOption == null) {
            sensorOption = new ArrayList<ParameterSettingType>();
        }
        return this.sensorOption;
    }

}
