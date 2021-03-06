//
// This file was generated by the JavaTM Architecture for XML Binding(JAXB) Reference Implementation, v2.3.0 
// See <a href="https://javaee.github.io/jaxb-v2/">https://javaee.github.io/jaxb-v2/</a> 
// Any modifications to this file will be lost upon recompilation of the source schema. 
// Generated on: 2020.10.23 at 10:21:09 AM EDT 
//


package crcl.base;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlSchemaType;
import javax.xml.bind.annotation.XmlType;


/**
 * 
 *                 StopMotionType is derived from MiddleCommandType.
 *                 An instance of StopMotionType has the following elements:
 *                 Name (inherited, optional)
 *                 CommandID (inherited)
 *                 StopCondition.
 * 
 *                 StopCondition is an enumerated value indicating how the stop
 *                 should occur.
 *             
 * 
 * <p>Java class for StopMotionType complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="StopMotionType"&gt;
 *   &lt;complexContent&gt;
 *     &lt;extension base="{}MiddleCommandType"&gt;
 *       &lt;sequence&gt;
 *         &lt;element name="StopCondition" type="{}StopConditionEnumType"/&gt;
 *       &lt;/sequence&gt;
 *     &lt;/extension&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "StopMotionType", propOrder = {
    "stopCondition"
})
public class StopMotionType
    extends MiddleCommandType
{

    @XmlElement(name = "StopCondition", required = true)
    @XmlSchemaType(name = "token")
    protected StopConditionEnumType stopCondition;

    /**
     * Gets the value of the stopCondition property.
     * 
     * @return
     *     possible object is
     *     {@link StopConditionEnumType }
     *     
     */
    public StopConditionEnumType getStopCondition() {
        return stopCondition;
    }

    /**
     * Sets the value of the stopCondition property.
     * 
     * @param value
     *     allowed object is
     *     {@link StopConditionEnumType }
     *     
     */
    public void setStopCondition(StopConditionEnumType value) {
        this.stopCondition = value;
    }

}
