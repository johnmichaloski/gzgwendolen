//
// This file was generated by the JavaTM Architecture for XML Binding(JAXB) Reference Implementation, v2.3.0 
// See <a href="https://javaee.github.io/jaxb-v2/">https://javaee.github.io/jaxb-v2/</a> 
// Any modifications to this file will be lost upon recompilation of the source schema. 
// Generated on: 2020.09.21 at 09:41:19 AM EDT 
//


package crcl.base;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlSeeAlso;
import javax.xml.bind.annotation.XmlType;


/**
 * 
 *                 The abstract MiddleCommandType is derived from CRCLCommandType.
 *                 MiddleCommandType has  the following elements:
 *                 Name (inherited, optional)
 *                 CommandID (inherited).
 * 
 *                 MiddleCommandType is the abstract parent type of specific CRCL
 *                 command types. Only derived types of MiddleCommandType may be
 *                 instantiated.
 *             
 * 
 * <p>Java class for MiddleCommandType complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="MiddleCommandType"&gt;
 *   &lt;complexContent&gt;
 *     &lt;extension base="{}CRCLCommandType"&gt;
 *     &lt;/extension&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "MiddleCommandType")
@XmlSeeAlso({
    ActuateJointsType.class,
    CloseToolChangerType.class,
    ConfigureJointReportsType.class,
    DwellType.class,
    GetStatusType.class,
    MessageType.class,
    MoveScrewType.class,
    MoveThroughToType.class,
    MoveToType.class,
    OpenToolChangerType.class,
    RunProgramType.class,
    SetAngleUnitsType.class,
    SetEndEffectorParametersType.class,
    SetEndEffectorType.class,
    SetEndPoseToleranceType.class,
    SetDefaultJointPositonsTolerancesType.class,
    SetForceUnitsType.class,
    SetIntermediatePoseToleranceType.class,
    SetLengthUnitsType.class,
    SetMotionCoordinationType.class,
    SetRobotParametersType.class,
    SetRotAccelType.class,
    SetRotSpeedType.class,
    SetTorqueUnitsType.class,
    SetTransAccelType.class,
    SetTransSpeedType.class,
    StopMotionType.class,
    ConfigureStatusReportType.class,
    EnableSensorType.class,
    DisableSensorType.class,
    EnableGripperType.class,
    DisableGripperType.class,
    EnableRobotParameterStatusType.class,
    DisableRobotParameterStatusType.class
})
public abstract class MiddleCommandType
    extends CRCLCommandType
{


}
