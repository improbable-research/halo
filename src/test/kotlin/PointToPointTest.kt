import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.junit.Test
import java.lang.Math.abs

class PointToPointTest {
    @Test
    fun testPointToPointCalculation() {
        val testParams = HelioStatParameters(
                Vector3D(1.0, 1.0, 1.0),
                HelioStatParameters.ServoParameters(0.001, 0.1, 0.01 + Math.PI / 2.0),
                HelioStatParameters.ServoParameters(0.002, 0.2, -Math.PI-0.03, -Math.PI)
        )

        val navigator = HelioStatNavigator(testParams)
        val model = HelioStat(testParams)
        val pitch = 1000
        val rotation = 345


        val sourcePoint = Vector3D(1.0, 2.0, 10.0)
        val distance = 5.5
        val correctServoSetting = ServoSetting(rotation, pitch)
        val target = model.computeTargetFromSourcePoint(
                ConstantDoubleVertex(correctServoSetting.pitch.toDouble()),
                ConstantDoubleVertex(correctServoSetting.rotation.toDouble()),
                sourcePoint,
                ConstantDoubleVertex(distance)
        ).getValue()

        val servoSetting = navigator.computeServoSettingFromPoint(sourcePoint, target,
                ServoSetting(rotation + 100, pitch + 100)
        )
        println("Servo setting pitch/rotation is ${servoSetting.pitch} ${servoSetting.rotation}")
        assert(servoSetting.pitch == pitch)
        assert(servoSetting.rotation == rotation)
    }

    @Test
    fun testPointToPointRealParams() {
        val testParams = HelioStatParameters(
                Vector3D(6.667, -0.695, 0.021),
                HelioStatParameters.ServoParameters(-0.0015, 3.146, 1.566),
                HelioStatParameters.ServoParameters(-0.0015, 4.707, 3.151, 0.012)
        )

        val navigator = HelioStatNavigator(testParams)
        val model = HelioStat(testParams)
        val pitch = 2118
        val rotation = 3322


        val sourcePoint = Vector3D(3.0, 0.0, 0.0)
        val distance = 3.0
        val correctServoSetting = ServoSetting(rotation, pitch)
        val target = model.computeTargetFromSourcePoint(
                ConstantDoubleVertex(correctServoSetting.pitch.toDouble()),
                ConstantDoubleVertex(correctServoSetting.rotation.toDouble()),
                sourcePoint,
                ConstantDoubleVertex(distance)
        ).getValue()
        println("TargetPoint is $target")

        val servoSetting = navigator.computeServoSettingFromPoint(sourcePoint, target,
                ServoSetting(rotation + 0, pitch + 0)
        )
        println("Servo setting pitch/rotation is ${servoSetting.pitch} ${servoSetting.rotation}")
//        assert(abs(servoSetting.pitch - pitch) < 2)
//        assert(abs(servoSetting.rotation - rotation) < 2)
        assert(servoSetting.pitch == pitch)
        assert(servoSetting.rotation == rotation)
    }

}
