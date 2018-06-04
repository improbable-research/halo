import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.junit.Test

class SetMirrorNormalTest {
    @Test
    fun testMirrorNormal() {
        val testParams = HelioStatParameters(
                Vector3D(1.0, 1.0, 1.0),
                HelioStatParameters.ServoParameters(0.001, 0.1),
                HelioStatParameters.ServoParameters(0.002, 0.2)
        )

        val navigator = HelioStatNavigator(testParams)
        val model = HelioStat(testParams)
        val pitch = 1000
        val rotation = 345
        val normal = Geometry.sphericalToCartesian(model.servoSignalToUnitSpherical(ServoSetting(rotation, pitch)).getValue())

        val settings = navigator.normalToServoSignal(normal)
        println("Setting rotation/pitch is ${settings.rotation} ${settings.pitch}")
        assert(settings.rotation == rotation)
        assert(settings.pitch == pitch)
    }
}