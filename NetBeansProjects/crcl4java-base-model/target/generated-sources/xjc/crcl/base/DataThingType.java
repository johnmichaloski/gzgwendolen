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
import javax.xml.bind.annotation.XmlID;
import javax.xml.bind.annotation.XmlSchemaType;
import javax.xml.bind.annotation.XmlSeeAlso;
import javax.xml.bind.annotation.XmlType;
import javax.xml.bind.annotation.adapters.CollapsedStringAdapter;
import javax.xml.bind.annotation.adapters.XmlJavaTypeAdapter;


/**
 * 
 *                 An instance of DataThingType has the following elements:
 *                 Name (optional)
 *                 .
 * 
 *                 DataThingType is an abstract type from which more specific types
 *                 of data thing are derived. That includes all complex data
 *                 types such as Vector, PoseType, etc.
 *             
 * 
 * <p>Java class for DataThingType complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="DataThingType"&gt;
 *   &lt;complexContent&gt;
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType"&gt;
 *       &lt;sequence&gt;
 *         &lt;element name="Name" type="{http://www.w3.org/2001/XMLSchema}ID" minOccurs="0"/&gt;
 *       &lt;/sequence&gt;
 *     &lt;/restriction&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "DataThingType", propOrder = {
    "name"
})
@XmlSeeAlso({
    CRCLStatusType.class,
    CRCLProgramType.class,
    CRCLCommandInstanceType.class,
    JointPositionToleranceSettingType.class,
    JointPositionsTolerancesType.class,
    PointType.class,
    PoseType.class,
    TwistType.class,
    VectorType.class,
    WrenchType.class,
    PoseToleranceType.class,
    ParameterSettingType.class,
    RotAccelType.class,
    RotSpeedType.class,
    TransAccelType.class,
    TransSpeedType.class,
    GuardType.class,
    ActuateJointType.class,
    ConfigureJointReportType.class,
    JointDetailsType.class,
    CRCLCommandType.class,
    CommandStatusType.class,
    JointStatusesType.class,
    JointStatusType.class,
    JointLimitType.class,
    PoseStatusType.class,
    SettingsStatusType.class,
    GripperStatusType.class,
    SensorStatusesType.class,
    GuardsStatusesType.class,
    SensorStatusType.class
})
public abstract class DataThingType {

    @XmlElement(name = "Name")
    @XmlJavaTypeAdapter(CollapsedStringAdapter.class)
    @XmlID
    @XmlSchemaType(name = "ID")
    protected String name;

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
    

              
        // Start of code Injected with xjc/maven-jaxb2-plugin  arg -Xinject-code -b inject.xjb
        @Override
        @SuppressWarnings("nullness")
        public String toString() { 
            return this.getName();
        }

        private static final java.util.concurrent.atomic.AtomicInteger NAME_ID_ATOMIC_INTEGER 
            = new java.util.concurrent.atomic.AtomicInteger();

        protected DataThingType() {
            this.name="thing."+NAME_ID_ATOMIC_INTEGER.incrementAndGet();
        }

        // End of code Injected with xjc/maven-jaxb2-plugin  arg -Xinject-code -b inject.xjb

            
}
