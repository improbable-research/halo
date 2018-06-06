import io.improbable.keanu.algorithms.variational.GradientOptimizer
import io.improbable.keanu.network.BayesNet
import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import io.improbable.keanu.vertices.dbl.probabilistic.GaussianVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.apache.commons.math3.optim.SimpleValueChecker
import org.apache.commons.math3.optim.nonlinear.scalar.gradient.NonLinearConjugateGradientOptimizer
import java.util.*

class HelioStatCalibrator(var calibrationData: CalibrationData) {

    private val ransacLogLikelihoodRejectionThreshold = 0.0001

    class DataPoint( val length : Double, val pitch : Double, val rotation : Double, val control: ServoSetting) {
        override fun toString() : String {
            return "${control.pitch} ${control.rotation} $pitch $rotation $length"
        }
    }

//    constructor(data : Array<DataPoint>): this(toArrayList(data))
//    constructor() : super()

//    fun ransac(): HelioStatParameters {
//        List<RandomSubsample>
//    }

//    companion object {
//        fun toArrayList(data: Array<DataPoint>): ArrayList<HelioStatCalibrator.DataPoint> {
//            val list = ArrayList<HelioStatCalibrator.DataPoint>()
//            list.addAll(data)
//            return list
//        }
//    }

    fun ransac() {
        val subsample = calibrationData.randomSubSample(5)
        val params = inferAllParams(subsample)
        val helio = HelioStat(params)

        val inliers = ArrayList<HelioStatCalibrator.DataPoint>()
        for (dataPoint in calibrationData) {
            val logLikelihood = helio.getLogLikelihood(dataPoint)
            if (logLikelihood < ransacLogLikelihoodRejectionThreshold) {
                inliers.add(dataPoint)
            }
        }
    }

    fun inferAllParams(calibrationData: CalibrationData = this.calibrationData) : HelioStatParameters {
        var params = inferServoParamsLinear(calibrationData)
        params = inferServoParams(params, calibrationData)
        params.pivotPoint = inferPivotPoint(calibrationData)
        return params
    }

    fun calculateResiduals(params : HelioStatParameters, calibrationData: CalibrationData = this.calibrationData) : ArrayList<Vector3D> {
        val forwardModel = HelioStat(ProbabilisticHelioStatParameters(params))
        var residual = ArrayList<Vector3D>(calibrationData.size)
        for(dataPoint in calibrationData) {
            val plane = forwardModel.computeHeliostatPlane(
                    ConstantDoubleVertex(dataPoint.control.pitch.toDouble()),
                    ConstantDoubleVertex(dataPoint.control.rotation.toDouble())
            )
            val modelledPlane = Geometry.cartesianToSpherical(plane.getValue())
            val observedPlane = Geometry.cartesianToSpherical(Geometry.sphericalToCartesian(Vector3D(dataPoint.length, dataPoint.pitch, dataPoint.rotation)))
            println("${dataPoint.control.pitch} ${dataPoint.control.rotation} ${observedPlane.y - modelledPlane.y} ${observedPlane.z - modelledPlane.z} ${observedPlane.x - modelledPlane.x}")
            residual.add(modelledPlane.subtract(observedPlane))
        }
        return residual
    }

    private fun inferPivotPoint(calibrationData: CalibrationData) : Vector3D {
        val helioStat = HelioStat(ProbabilisticHelioStatParameters())

        for(dataPoint in calibrationData) {
            val modelledLength = helioStat.computePlaneDistanceFromOrigin(dataPoint.pitch, dataPoint.rotation)
            GaussianVertex(modelledLength, 0.001).observe(dataPoint.length)
//            println("Err ${modelledLength.value - dataPoint.length}")
        }
        val model = BayesNet(helioStat.params.pivotPoint.x.connectedGraph)
        val optimiser = GradientOptimizer(model)
        optimiser.maxAPosteriori(10000,
                NonLinearConjugateGradientOptimizer(
                        NonLinearConjugateGradientOptimizer.Formula.POLAK_RIBIERE,
                        SimpleValueChecker(1e-15, 1e-15)))
        return(helioStat.params.pivotPoint.getValue())
    }

    private fun inferServoParams(initialGuess : HelioStatParameters, calibrationData: CalibrationData) : HelioStatParameters {
        val params = ProbabilisticHelioStatParameters()
        val helioStat = HelioStat(params)

        helioStat.params.pitchParameters.m.value = initialGuess.pitchParameters.m
        helioStat.params.pitchParameters.c.value = initialGuess.pitchParameters.c
        helioStat.params.rotationParameters.m.value = initialGuess.rotationParameters.m
        helioStat.params.rotationParameters.c.value = initialGuess.rotationParameters.c

        for (dataPoint in calibrationData) {
            val cartesianNorm = helioStat.computeHeliostatNormal(dataPoint.control)
            val observedNorm = Geometry.sphericalToCartesian(Vector3D(1.0, dataPoint.pitch, dataPoint.rotation))
            GaussianVertex(cartesianNorm.x, 0.005).observe(observedNorm.x)
            GaussianVertex(cartesianNorm.y, 0.005).observe(observedNorm.y)
            GaussianVertex(cartesianNorm.z, 0.005).observe(observedNorm.z)
        }
        val pmodel = BayesNet(
                helioStat.params.pitchParameters.m.connectedGraph.union(
                        helioStat.params.rotationParameters.m.connectedGraph.union(
                                helioStat.params.pitchParameters.axisPitch.connectedGraph.union(
                                        helioStat.params.rotationParameters.axisPitch.connectedGraph
                                )
                        )
                )
        )
        val poptimiser = GradientOptimizer(pmodel)
        poptimiser.maxAPosteriori(10000,
                NonLinearConjugateGradientOptimizer(
                        NonLinearConjugateGradientOptimizer.Formula.POLAK_RIBIERE,
                        SimpleValueChecker(1e-16, 1e-16)))

        return helioStat.params.getValue()
    }

    private fun inferServoParamsLinear(calibrationData: CalibrationData) : HelioStatParameters {
        val params = ProbabilisticHelioStatParameters()
        params.fixAxisParametersToSpherical()
        val helioStat = HelioStat(params)

        for (dataPoint in calibrationData) {
            val sphericalNorm = helioStat.linearSphericalNormalModel(dataPoint.control)
            GaussianVertex(sphericalNorm.y,0.001).observe(dataPoint.pitch)
            GaussianVertex(sphericalNorm.z, 0.001).observe(dataPoint.rotation)
        }

        val pmodel = BayesNet(helioStat.params.pitchParameters.m.connectedGraph)
        val poptimiser = GradientOptimizer(pmodel)
        poptimiser.maxAPosteriori(10000,
                NonLinearConjugateGradientOptimizer(
                        NonLinearConjugateGradientOptimizer.Formula.POLAK_RIBIERE,
                        SimpleValueChecker(1e-16, 1e-16)))

        val rmodel = BayesNet(helioStat.params.rotationParameters.m.connectedGraph)
        val roptimiser = GradientOptimizer(rmodel)
        roptimiser.maxAPosteriori(10000,
                NonLinearConjugateGradientOptimizer(
                        NonLinearConjugateGradientOptimizer.Formula.POLAK_RIBIERE,
                        SimpleValueChecker(1e-16, 1e-16)))

        return helioStat.params.getValue()
    }
}
