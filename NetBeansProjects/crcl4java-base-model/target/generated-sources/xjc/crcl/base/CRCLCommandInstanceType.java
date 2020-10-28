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
import javax.xml.bind.annotation.XmlType;


/**
 * 
 *         CRCLCommandInstanceType is derived from DataThingType.
 *         An instance of CRCLCommandInstanceType has the following elements:
 *           Name (inherited, optional)
 *           CRCLCommand
 *           ProgramFile (optional)
 *           ProgramIndex (optional)
 *           ProgramLength (optional).
 * 
 *           ProgramFile provides an optional reference if the currently executing command
 *              is known to have come from a particular file. 
 *           ProgramIndex provoides an optional reference to the element within a program. If the 
 *              currently executing command is known to have come from a particular file. The InitCanon command will have 
 *              index 0, and first MiddleCommand will have index 1.
 *           ProgramLength is the number of commands in the current program if known.
 *           
 *         CRCLCommandInstanceType contains a single CRCL command.
 *       
 * 
 * <p>Java class for CRCLCommandInstanceType complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="CRCLCommandInstanceType"&gt;
 *   &lt;complexContent&gt;
 *     &lt;extension base="{}DataThingType"&gt;
 *       &lt;sequence&gt;
 *         &lt;element name="CRCLCommand" type="{}CRCLCommandType"/&gt;
 *         &lt;element name="ProgramFile" type="{http://www.w3.org/2001/XMLSchema}string" minOccurs="0"/&gt;
 *         &lt;element name="ProgramIndex" type="{http://www.w3.org/2001/XMLSchema}int" minOccurs="0"/&gt;
 *         &lt;element name="ProgramLength" type="{http://www.w3.org/2001/XMLSchema}int" minOccurs="0"/&gt;
 *       &lt;/sequence&gt;
 *     &lt;/extension&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "CRCLCommandInstanceType", propOrder = {
    "crclCommand",
    "programFile",
    "programIndex",
    "programLength"
})
public class CRCLCommandInstanceType
    extends DataThingType
{

    @XmlElement(name = "CRCLCommand", required = true)
    protected CRCLCommandType crclCommand;
    @XmlElement(name = "ProgramFile")
    protected String programFile;
    @XmlElement(name = "ProgramIndex")
    protected Integer programIndex;
    @XmlElement(name = "ProgramLength")
    protected Integer programLength;

    /**
     * Gets the value of the crclCommand property.
     * 
     * @return
     *     possible object is
     *     {@link CRCLCommandType }
     *     
     */
    public CRCLCommandType getCRCLCommand() {
        return crclCommand;
    }

    /**
     * Sets the value of the crclCommand property.
     * 
     * @param value
     *     allowed object is
     *     {@link CRCLCommandType }
     *     
     */
    public void setCRCLCommand(CRCLCommandType value) {
        this.crclCommand = value;
    }

    /**
     * Gets the value of the programFile property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    public String getProgramFile() {
        return programFile;
    }

    /**
     * Sets the value of the programFile property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    public void setProgramFile(String value) {
        this.programFile = value;
    }

    /**
     * Gets the value of the programIndex property.
     * 
     * @return
     *     possible object is
     *     {@link Integer }
     *     
     */
    public Integer getProgramIndex() {
        return programIndex;
    }

    /**
     * Sets the value of the programIndex property.
     * 
     * @param value
     *     allowed object is
     *     {@link Integer }
     *     
     */
    public void setProgramIndex(Integer value) {
        this.programIndex = value;
    }

    /**
     * Gets the value of the programLength property.
     * 
     * @return
     *     possible object is
     *     {@link Integer }
     *     
     */
    public Integer getProgramLength() {
        return programLength;
    }

    /**
     * Sets the value of the programLength property.
     * 
     * @param value
     *     allowed object is
     *     {@link Integer }
     *     
     */
    public void setProgramLength(Integer value) {
        this.programLength = value;
    }

}
