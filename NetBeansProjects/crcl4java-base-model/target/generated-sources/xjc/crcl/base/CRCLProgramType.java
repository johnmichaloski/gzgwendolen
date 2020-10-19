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
 *         CRCLProgramType is derived from DataThingType.
 *         An instance of CRCLProgramType has the following elements:
 *           Name (inherited, optional)
 *           InitCanon
 *           MiddleCommand (optional, multiple)
 *           EndCanon.
 * 
 *         CRCLProgramType defines a CRCL program as a sequence of CRCL
 *         commands. The CRCL commands in an instance of CRCLProgramType
 *         must be executed in the order given. Using a CRCL program is
 *         intended for testing and demos, not normal operation.
 * 
 *         A CRCL program must start with an InitCanon command and end
 *         with an EndCanon command. It may have zero to many middle
 *         commands between the InitCanon and the EndCanon.
 *       
 * 
 * <p>Java class for CRCLProgramType complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="CRCLProgramType"&gt;
 *   &lt;complexContent&gt;
 *     &lt;extension base="{}DataThingType"&gt;
 *       &lt;sequence&gt;
 *         &lt;element name="InitCanon" type="{}InitCanonType"/&gt;
 *         &lt;element name="MiddleCommand" type="{}MiddleCommandType" maxOccurs="unbounded" minOccurs="0"/&gt;
 *         &lt;element name="EndCanon" type="{}EndCanonType"/&gt;
 *       &lt;/sequence&gt;
 *     &lt;/extension&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "CRCLProgramType", propOrder = {
    "initCanon",
    "middleCommand",
    "endCanon"
})
public class CRCLProgramType
    extends DataThingType
{

    @XmlElement(name = "InitCanon", required = true)
    protected InitCanonType initCanon;
    @XmlElement(name = "MiddleCommand")
    protected List<MiddleCommandType> middleCommand;
    @XmlElement(name = "EndCanon", required = true)
    protected EndCanonType endCanon;

    /**
     * Gets the value of the initCanon property.
     * 
     * @return
     *     possible object is
     *     {@link InitCanonType }
     *     
     */
    public InitCanonType getInitCanon() {
        return initCanon;
    }

    /**
     * Sets the value of the initCanon property.
     * 
     * @param value
     *     allowed object is
     *     {@link InitCanonType }
     *     
     */
    public void setInitCanon(InitCanonType value) {
        this.initCanon = value;
    }

    /**
     * Gets the value of the middleCommand property.
     * 
     * <p>
     * This accessor method returns a reference to the live list,
     * not a snapshot. Therefore any modification you make to the
     * returned list will be present inside the JAXB object.
     * This is why there is not a <CODE>set</CODE> method for the middleCommand property.
     * 
     * <p>
     * For example, to add a new item, do as follows:
     * <pre>
     *    getMiddleCommand().add(newItem);
     * </pre>
     * 
     * 
     * <p>
     * Objects of the following type(s) are allowed in the list
     * {@link MiddleCommandType }
     * 
     * 
     */
    public List<MiddleCommandType> getMiddleCommand() {
        if (middleCommand == null) {
            middleCommand = new ArrayList<MiddleCommandType>();
        }
        return this.middleCommand;
    }

    /**
     * Gets the value of the endCanon property.
     * 
     * @return
     *     possible object is
     *     {@link EndCanonType }
     *     
     */
    public EndCanonType getEndCanon() {
        return endCanon;
    }

    /**
     * Sets the value of the endCanon property.
     * 
     * @param value
     *     allowed object is
     *     {@link EndCanonType }
     *     
     */
    public void setEndCanon(EndCanonType value) {
        this.endCanon = value;
    }

}