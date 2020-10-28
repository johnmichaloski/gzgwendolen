//
// This file was generated by the JavaTM Architecture for XML Binding(JAXB) Reference Implementation, v2.3.0 
// See <a href="https://javaee.github.io/jaxb-v2/">https://javaee.github.io/jaxb-v2/</a> 
// Any modifications to this file will be lost upon recompilation of the source schema. 
// Generated on: 2020.10.23 at 10:21:09 AM EDT 
//


package crcl.base;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlType;


/**
 * 
 *                 GetStatusType is derived from MiddleCommandType.
 *                 An instance of GetStatusType has the following elements:
 *                 Name (inherited, optional)
 *                 CommandID (inherited).
 * 
 *                 An instance of GetStatusType is used to indicate that the robot
 *                 should report status immediately. The joint status portion of
 *                 the status report must be as set by the most recent
 *                 ConfigureJointReports command.
 *             
 * 
 * <p>Java class for GetStatusType complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="GetStatusType"&gt;
 *   &lt;complexContent&gt;
 *     &lt;extension base="{}MiddleCommandType"&gt;
 *     &lt;/extension&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "GetStatusType")
public class GetStatusType
    extends MiddleCommandType
{


}
