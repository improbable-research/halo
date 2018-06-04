import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.junit.Test

class RayToPointTest {
    @Test
    fun testCalibrationFromSyntheticData() {
        val testParams = HelioStatParameters(
                Vector3D(1.0, 1.0, 1.0),
                HelioStatParameters.ServoParameters(0.001, 0.1),
                HelioStatParameters.ServoParameters(0.002, 0.2)
        )

        val navigator = HelioStatNavigator(testParams)
        val model = HelioStat(testParams)

        val sunVector = Vector3D(1.0, -0.1, 1.0).normalize()
        val distance = 4.0
        val correctServoSetting = ServoSetting(2345, 1234)
        val target = model.computeTargetFromSourceDirection(
                ConstantDoubleVertex(correctServoSetting.pitch.toDouble()),
                ConstantDoubleVertex(correctServoSetting.rotation.toDouble()),
                ProbabilisticVector3D(sunVector),
                ConstantDoubleVertex(distance)).getValue()

        val servoSetting = navigator.computeServoSettingFromDirection(sunVector, target)
        println("Servo setting pitch/rotation is ${servoSetting.pitch} ${servoSetting.rotation}")
    }

}