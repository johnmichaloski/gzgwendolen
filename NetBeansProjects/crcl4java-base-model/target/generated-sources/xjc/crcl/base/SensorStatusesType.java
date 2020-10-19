//
// This file was generated by the JavaTM Architecture for XML Binding(JAXB) Reference Implementation, v2.3.0 
// See <a href="https://javaee.github.io/jaxb-v2/">https://javaee.github.io/jaxb-v2/</a> 
// Any modifications to this file will be lost upon recompilation of the source schema. 
// Generated on: 2020.09.26 at 08:19:21 PM EDT 
//


package crcl.base;

import java.util.ArrayList;
import java.util.List;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlType;


/**
 * 
 *                 SensorStatusesType is derived from DataThingType.
 *                 An instance of SensorStatusesType has the following elements:
 *                 Name (inherited, optional)
 *                 SensorStatus (multiple).
 *  
 *                 Each SensorStatus element gives the status of one sensor. A robot
 *                 may be associated with any number of internal or external sensors.
 *                 Any custom named internal variable could also be reported with 
 *                 the interface.
 *             
 * 
 * <p>Java class for SensorStatusesType complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="SensorStatusesType"&gt;
 *   &lt;complexContent&gt;
 *     &lt;extension base="{}DataThingType"&gt;
 *       &lt;sequence&gt;
 *         &lt;element name="OnOffSensorStatus" type="{}OnOffSensorStatusType" maxOccurs="unbounded" minOccurs="0"/&gt;
 *         &lt;element name="ScalarSensorStatus" type="{}ScalarSensorStatusType" maxOccurs="unbounded" minOccurs="0"/&gt;
 *         &lt;element name="CountSensorStatus" type="{}CountSensorStatusType" maxOccurs="unbounded" minOccurs="0"/&gt;
 *         &lt;element name="ForceTorqueSensorStatus" type="{}ForceTorqueSensorStatusType" maxOccurs="unbounded" minOccurs="0"/&gt;
 *       &lt;/sequence&gt;
 *     &lt;/extension&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "SensorStatusesType", propOrder = {
    "onOffSensorStatus",
    "scalarSensorStatus",
    "countSensorStatus",
    "forceTorqueSensorStatus"
})
public class SensorStatusesType
    extends DataThingType
{

    @XmlElement(name = "OnOffSensorStatus")
    protected List<OnOffSensorStatusType> onOffSensorStatus;
    @XmlElement(name = "ScalarSensorStatus")
    protected List<ScalarSensorStatusType> scalarSensorStatus;
    @XmlElement(name = "CountSensorStatus")
    protected List<CountSensorStatusType> countSensorStatus;
    @XmlElement(name = "ForceTorqueSensorStatus")
    protected List<ForceTorqueSensorStatusType> forceTorqueSensorStatus;

    /**
     * Gets the value of the onOffSensorStatus property.
     * 
     * <p>
     * This accessor method returns a reference to the live list,
     * not a snapshot. Therefore any modification you make to the
     * returned list will be present inside the JAXB object.
     * This is why there is not a <CODE>set</CODE> method for the onOffSensorStatus property.
     * 
     * <p>
     * For example, to add a new item, do as follows:
     * <pre>
     *    getOnOffSensorStatus().add(newItem);
     * </pre>
     * 
     * 
     * <p>
     * Objects of the following type(s) are allowed in the list
     * {@link OnOffSensorStatusType }
     * 
     * 
     */
    public List<OnOffSensorStatusType> getOnOffSensorStatus() {
        if (onOffSensorStatus == null) {
            onOffSensorStatus = new ArrayList<OnOffSensorStatusType>();
        }
        return this.onOffSensorStatus;
    }

    /**
     * Gets the value of the scalarSensorStatus property.
     * 
     * <p>
     * This accessor method returns a reference to the live list,
     * not a snapshot. Therefore any modification you make to the
     * returned list will be present inside the JAXB object.
     * This is why there is not a <CODE>set</CODE> method for the scalarSensorStatus property.
     * 
     * <p>
     * For example, to add a new item, do as follows:
     * <pre>
     *    getScalarSensorStatus().add(newItem);
     * </pre>
     * 
     * 
     * <p>
     * Objects of the following type(s) are allowed in the list
     * {@link ScalarSensorStatusType }
     * 
     * 
     */
    public List<ScalarSensorStatusType> getScalarSensorStatus() {
        if (scalarSensorStatus == null) {
            scalarSensorStatus = new ArrayList<ScalarSensorStatusType>();
        }
        return this.scalarSensorStatus;
    }

    /**
     * Gets the value of the countSensorStatus property.
     * 
     * <p>
     * This accessor method returns a reference to the live list,
     * not a snapshot. Therefore any modification you make to the
     * returned list will be present inside the JAXB object.
     * This is why there is not a <CODE>set</CODE> method for the countSensorStatus property.
     * 
     * <p>
     * For example, to add a new item, do as follows:
     * <pre>
     *    getCountSensorStatus().add(newItem);
     * </pre>
     * 
     * 
     * <p>
     * Objects of the following type(s) are allowed in the list
     * {@link CountSensorStatusType }
     * 
     * 
     */
    public List<CountSensorStatusType> getCountSensorStatus() {
        if (countSensorStatus == null) {
            countSensorStatus = new ArrayList<CountSensorStatusType>();
        }
        return this.countSensorStatus;
    }

    /**
     * Gets the value of the forceTorqueSensorStatus property.
     * 
     * <p>
     * This accessor method returns a reference to the live list,
     * not a snapshot. Therefore any modification you make to the
     * returned list will be present inside the JAXB object.
     * This is why there is not a <CODE>set</CODE> method for the forceTorqueSensorStatus property.
     * 
     * <p>
     * For example, to add a new item, do as follows:
     * <pre>
     *    getForceTorqueSensorStatus().add(newItem);
     * </pre>
     * 
     * 
     * <p>
     * Objects of the following type(s) are allowed in the list
     * {@link ForceTorqueSensorStatusType }
     * 
     * 
     */
    public List<ForceTorqueSensorStatusType> getForceTorqueSensorStatus() {
        if (forceTorqueSensorStatus == null) {
            forceTorqueSensorStatus = new ArrayList<ForceTorqueSensorStatusType>();
        }
        return this.forceTorqueSensorStatus;
    }

}
