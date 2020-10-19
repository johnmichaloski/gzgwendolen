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
import javax.xml.bind.annotation.XmlType;


/**
 * 
 *                 RotAccelRelativeType is derived from RotAccelType.
 *                 An instance of RotAccelRelativeType has the following elements:
 *                 Name (inherited, optional)
 *                 Fraction.
 * 
 *                 Fraction is a real number that represents the fraction of the
 *                 robot's maximum rotational acceleration that it should use.
 *             
 * 
 * <p>Java class for RotAccelRelativeType complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="RotAccelRelativeType"&gt;
 *   &lt;complexContent&gt;
 *     &lt;extension base="{}RotAccelType"&gt;
 *       &lt;sequence&gt;
 *         &lt;element name="Fraction" type="{}FractionType"/&gt;
 *       &lt;/sequence&gt;
 *     &lt;/extension&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "RotAccelRelativeType", propOrder = {
    "fraction"
})
public class RotAccelRelativeType
    extends RotAccelType
{

    @XmlElement(name = "Fraction")
    protected double fraction;

    /**
     * Gets the value of the fraction property.
     * 
     */
    public double getFraction() {
        return fraction;
    }

    /**
     * Sets the value of the fraction property.
     * 
     */
    public void setFraction(double value) {
        this.fraction = value;
    }

}
