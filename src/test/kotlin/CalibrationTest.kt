import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.junit.Test

public class CalibrationTest {
    @Test
    fun testCalibrationFromFile() {

        val dataReader = CalibrationDataReadAndConvert()
        dataReader.readFromFile("calibrationData.json")
        dataReader.randomSubSample(20)

        val calibrator = HelioStatCalibration(dataReader)


        val bestParams = calibrator.inferAllParams()
        println("Modelled params are: $bestParams")
        val r = calibrator.calculateResiduals(bestParams)
        val residual = r.sumByDouble(Vector3D::getNorm) / r.size
        println("average residual is $residual")

        // TODO should be small and uncorrelated - if there's a shape, then stick it in the model and regress to that.
        for (i in 0 until calibrator.size) {
            println("${calibrator[i].control.pitch} ${calibrator[i].control.rotation} ${r[i].x} ${r[i].y} ${r[i].z}")
        }
    }


    @Test
    fun testCalibrationFromSyntheticData() {
        val testParams = HelioStatParameters(
                Vector3D(1.0, 1.0, 1.0),
                HelioStatParameters.ServoParameters(0.001, 0.1),
                HelioStatParameters.ServoParameters(0.002, 0.2)
        )

        val dataReader = CalibrationDataReadAndConvert()
        dataReader.createSyntheticTrainingSet(30, testParams)
        val calibrator = HelioStatCalibration(dataReader)

        val bestParams = calibrator.inferAllParams()
        println("Modelled params are: $bestParams")
        val r = calibrator.calculateResiduals(bestParams)
        val residual = r.sumByDouble(Vector3D::getNorm) / r.size
        println("average residual is $residual")
        assert(residual < 1e-6)
    }
}