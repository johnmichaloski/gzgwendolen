//
// This file was generated by the JavaTM Architecture for XML Binding(JAXB) Reference Implementation, v2.3.0 
// See <a href="https://javaee.github.io/jaxb-v2/">https://javaee.github.io/jaxb-v2/</a> 
// Any modifications to this file will be lost upon recompilation of the source schema. 
// Generated on: 2020.09.26 at 08:19:21 PM EDT 
//


package crcl.base;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlType;


/**
 * 
 *                 CloseToolChangerType is derived from MiddleCommandType. 
 *                 An instance of CloseToolChangerType has the following elements:
 *                 Name (inherited, optional)
 *                 CommandID (inherited).
 * 
 *                 After an instance of CloseToolChangerType is executed, it is
 *                 understood that if the tool changer was in position to acquire an
 *                 end effector, the end effector will be mounted on the robot. In
 *                 that case, the controlled point will change.
 *             
 * 
 * <p>Java class for CloseToolChangerType complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="CloseToolChangerType"&gt;
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
@XmlType(name = "CloseToolChangerType")
public class CloseToolChangerType
    extends MiddleCommandType
{


}
