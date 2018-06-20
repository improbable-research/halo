import net.e175.klaus.solarpositioning.DeltaT
import net.e175.klaus.solarpositioning.SPA
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import java.util.*

class SolarModel {
    val solar1 = Vector3D(1.0, (90 - 60.6422)*2.0*Math.PI/360.0, -346.471*2.0*Math.PI/360.0)
//    val solar1 = Vector3D(1.0,60.6422*2.0*Math.PI/360.0, -346.471*2.0*Math.PI/360.0)
    val solar1c = Geometry.sphericalToCartesian(solar1)
    val terrestrial1 = Vector3D(1.77486, 15.7425, -8.80466).normalize()
    val terrestrial1s = Geometry.standardCartesianToSpherical(terrestrial1)

    val solar2 = Vector3D(1.0, (90 - 47.6476)*2.0*Math.PI/360.0,  -62.648*2.0*Math.PI/360.0)
//    val solar2 = Vector3D(1.0, 47.6476*2.0*Math.PI/360.0, -62.648*2.0*Math.PI/360.0)
    val solar2c = Geometry.sphericalToCartesian(solar2)
    val terrestrial2 = Vector3D(12.3559, 13.3273, -0.623976).normalize()
    val terrestrial2s = Geometry.cartesianToSpherical(terrestrial2)

    val solar3 = Vector3D(1.0, (90 - 59.1687)*2.0*Math.PI/360.0, -333.961*2.0*Math.PI/360.0)
//    val solar3 = Vector3D(1.0, 59.1687*2.0*Math.PI/360.0, -333.961*2.0*Math.PI/360.0)
    val solar3c = Geometry.sphericalToCartesian(solar3)
    val terrestrial3 = Vector3D(-0.218503, 15.4721, -9.36315).normalize()
    val terrestrial3s = Geometry.standardCartesianToSpherical(terrestrial3)


}

fun main(args :Array<String>) {
    val s = SolarModel()
    println("angle 1 is ${Math.toDegrees(Math.acos(s.solar1c.dotProduct(s.terrestrial1)))}")
    println("angle 2 is ${Math.toDegrees(Math.acos(s.solar2c.dotProduct(s.terrestrial2)))}")
    println("angle 3 is ${Math.toDegrees(Math.acos(s.solar3c.dotProduct(s.terrestrial3)))}")

    println("${s.solar1} ${s.terrestrial1s} ${s.solar1.subtract(s.terrestrial1s)}")
    println("${s.solar2} ${s.terrestrial2s} ${s.solar2.subtract(s.terrestrial2s)}")
    println("${s.solar3} ${s.terrestrial3s} ${s.solar3.subtract(s.terrestrial3s)}")

    val position = Astronomy.getSolarPosition()
    println("position is ${position}")
}