import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.junit.Test

public class CalibrationTest {
    @Test
    fun testCalibrationFromFile() {

        val calibrationData = CalibrationData()
        calibrationData.readFromFileFormat2("heliostatData.json")
//        calibrationData.randomSubSample(20)

        val calibrator = HelioStatCalibrator(calibrationData)

        val bestParams = calibrator.inferHelioStatParams()
        println("Modelled params are: $bestParams")
        val r = calibrator.calculateResiduals(bestParams)
        analyzeResiduals(r)

        println()
        println("Individual data point log likelihoods:")
        val helio = HelioStat(bestParams)
        for (dataPoint in calibrationData) {
            val logLikelihood = helio.getLogLikelihood(dataPoint)
            println("$logLikelihood (${Math.exp(logLikelihood)})")
        }


        // TODO should be small and uncorrelated - if there's a shape, then stick it in the model and regress to that.
//        for (i in 0 until calibrator.size) {
//            println("${calibrator[i].control.pitch} ${calibrator[i].control.rotation} ${r[i].x} ${r[i].y} ${r[i].z}")
//        }
    }

    @Test
    fun testRansacCalibrationFromFile() {

        val calibrationData = CalibrationData()
        calibrationData.readFromFileFormat2("heliostatData.json")

        val calibrator = HelioStatCalibrator(calibrationData)

        val bestParams = calibrator.inferHelioStatParamsRANSAC()
        println("Modelled params are: $bestParams")
        val r = calibrator.calculateResiduals(bestParams)
        val residual = r.sumByDouble(Vector3D::getNorm) / r.size
        println("average residual is $residual")

        // TODO should be small and uncorrelated - if there's a shape, then stick it in the model and regress to that.
//        for (i in 0 until calibrator.size) {
//            println("${calibrator[i].control.pitch} ${calibrator[i].control.rotation} ${r[i].x} ${r[i].y} ${r[i].z}")
//        }
    }


    @Test
    fun testCalibrationFromSyntheticData() {

        val testParams = HelioStatParameters(
                Vector3D(1.0, 1.0, 1.0),
                  HelioStatParameters.ServoParameters(0.001, 0.1, Math.PI / 2.0 - 0.01),
                  HelioStatParameters.ServoParameters(0.002, 0.2, Math.PI, 0.0)
        )

        val dataReader = CalibrationData()
        dataReader.createSyntheticTrainingSet(30, testParams)
        val calibrator = HelioStatCalibrator(dataReader)

        val bestParams = calibrator.inferHelioStatParams()
        println("Modelled params are: $bestParams")
        val r = calibrator.calculateResiduals(bestParams)
        val residual = r.sumByDouble(Vector3D::getNorm) / r.size
        println("average residual is $residual")
        assert(residual < 1e-2)
    }

    fun analyzeResiduals(r : List<Vector3D>) {
//        val residual = r.sumByDouble(Vector3D::getNorm) / r.size
//        println("average residual is $residual")

        val residualMean = r.reduce({a,b -> a.add(b)}).scalarMultiply(1.0/r.size)
        println("residual mean is $residualMean")
        val squaredResiduals = r.map({v -> Vector3D(v.x*v.x, v.y*v.y, v.z*v.z)})
        val residualMeanSquared = squaredResiduals.reduce({a,b -> a.add(b)}).scalarMultiply(1.0/r.size)
        println("residual SD is" +
                " ${Math.sqrt(residualMeanSquared.x - residualMean.x*residualMean.x)}" +
                " ${Math.sqrt(residualMeanSquared.y - residualMean.y*residualMean.y)}" +
                " ${Math.sqrt(residualMeanSquared.z - residualMean.z*residualMean.z)}")

    }
}
