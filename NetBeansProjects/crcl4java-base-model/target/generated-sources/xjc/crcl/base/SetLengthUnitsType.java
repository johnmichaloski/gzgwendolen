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
import javax.xml.bind.annotation.XmlSchemaType;
import javax.xml.bind.annotation.XmlType;


/**
 * 
 *                 SetLengthUnitsType is derived from MiddleCommandType.
 *                 An instance of SetLengthUnitsType has the following elements:
 *                 Name (inherited, optional)
 *                 CommandID (inherited)
 *                 UnitName.
 * 
 *                 UnitName is a string that can be only the literals 'meter',
 *                 'millimeter', or 'inch'. This tells the robot that all further
 *                 commands giving position or length values will implicitly use the
 *                 named unit. 
 *             
 * 
 * <p>Java class for SetLengthUnitsType complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="SetLengthUnitsType"&gt;
 *   &lt;complexContent&gt;
 *     &lt;extension base="{}MiddleCommandType"&gt;
 *       &lt;sequence&gt;
 *         &lt;element name="UnitName" type="{}LengthUnitEnumType"/&gt;
 *       &lt;/sequence&gt;
 *     &lt;/extension&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "SetLengthUnitsType", propOrder = {
    "unitName"
})
public class SetLengthUnitsType
    extends MiddleCommandType
{

    @XmlElement(name = "UnitName", required = true)
    @XmlSchemaType(name = "NMTOKEN")
    protected LengthUnitEnumType unitName;

    /**
     * Gets the value of the unitName property.
     * 
     * @return
     *     possible object is
     *     {@link LengthUnitEnumType }
     *     
     */
    public LengthUnitEnumType getUnitName() {
        return unitName;
    }

    /**
     * Sets the value of the unitName property.
     * 
     * @param value
     *     allowed object is
     *     {@link LengthUnitEnumType }
     *     
     */
    public void setUnitName(LengthUnitEnumType value) {
        this.unitName = value;
    }

}
