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
import javax.xml.bind.annotation.adapters.CollapsedStringAdapter;
import javax.xml.bind.annotation.adapters.XmlJavaTypeAdapter;


/**
 * 
 *                 ParameterSettingType is derived from DataThingType. 
 *                 An instance of ParameterSettingType has the following elements:
 *                 Name (inherited, optional)
 *                 ParameterName
 *                 ParameterValue.
 * 
 *                 ParameterSettingType is used to set values of parameters. The
 *                 ParameterName and ParameterValue are both strings. The
 *                 ParameterValue string may represent a data type known to the
 *                 receiving system.
 *             
 * 
 * <p>Java class for ParameterSettingType complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="ParameterSettingType"&gt;
 *   &lt;complexContent&gt;
 *     &lt;extension base="{}DataThingType"&gt;
 *       &lt;sequence&gt;
 *         &lt;element name="ParameterName" type="{http://www.w3.org/2001/XMLSchema}token"/&gt;
 *         &lt;element name="ParameterValue" type="{http://www.w3.org/2001/XMLSchema}token"/&gt;
 *       &lt;/sequence&gt;
 *     &lt;/extension&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "ParameterSettingType", propOrder = {
    "parameterName",
    "parameterValue"
})
public class ParameterSettingType
    extends DataThingType
{

    @XmlElement(name = "ParameterName", required = true)
    @XmlJavaTypeAdapter(CollapsedStringAdapter.class)
    @XmlSchemaType(name = "token")
    protected String parameterName;
    @XmlElement(name = "ParameterValue", required = true)
    @XmlJavaTypeAdapter(CollapsedStringAdapter.class)
    @XmlSchemaType(name = "token")
    protected String parameterValue;

    /**
     * Gets the value of the parameterName property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    public String getParameterName() {
        return parameterName;
    }

    /**
     * Sets the value of the parameterName property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    public void setParameterName(String value) {
        this.parameterName = value;
    }

    /**
     * Gets the value of the parameterValue property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    public String getParameterValue() {
        return parameterValue;
    }

    /**
     * Sets the value of the parameterValue property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    public void setParameterValue(String value) {
        this.parameterValue = value;
    }
    

              
        // Start of code Injected with xjc/maven-jaxb2-plugin  arg -Xinject-code -b inject.xjb

        @Override
        @SuppressWarnings("nullness")
        public String toString() { 
            return "{"+this.parameterName+"="+this.parameterValue+"} ";
        }


        public ParameterSettingType() {
            this.parameterName="noname";
            this.parameterValue="valuenotset";
        }

        // End of code Injected with xjc/maven-jaxb2-plugin  arg -Xinject-code -b inject.xjb

            
}
