import io.improbable.keanu.algorithms.variational.GradientOptimizer
import io.improbable.keanu.network.BayesNet
import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import io.improbable.keanu.vertices.dbl.probabilistic.GaussianVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.apache.commons.math3.optim.SimpleValueChecker
import org.apache.commons.math3.optim.nonlinear.scalar.gradient.NonLinearConjugateGradientOptimizer
import java.util.*

class HelioStatCalibrator(val calibrationData: CalibrationData) {

    private val ransacSampleSize = 5
    private val ransacLogLikelihoodRejectionThreshold = -6.0
    private val maxAcceptableRejectionRate = 0.3
    private val rejectionRateToInstantlyAccept = 0.1

    fun inferHelioStatParamsRANSAC(): HelioStatParameters {
        val bestCalibrationData = runRansacForMaxInliers()
        val params = inferHelioStatParams(bestCalibrationData)

        val r = calculateResiduals(params, bestCalibrationData)
        val residual = r.sumByDouble(Vector3D::getNorm) / r.size

        println("RANSAC calibrated HelioStatParameters with ${bestCalibrationData.size} of ${calibrationData.size} " +
                "data points and a residual error of $residual")

        val helio = HelioStat(params)
        for (dataPoint in bestCalibrationData) {
            val logLikelihood = helio.getLogLikelihood(dataPoint)
            println("$logLikelihood (${Math.exp(logLikelihood)})")
        }

        return params
    }

    fun inferHelioStatParams(calibrationData: CalibrationData = this.calibrationData): HelioStatParameters {
        var params = inferServoParamsLinear(calibrationData)
        params = inferServoParams(params, calibrationData)
        params.pivotPoint = inferPivotPoint(calibrationData)
        return params
    }

    fun calculateResiduals(params: HelioStatParameters, calibrationData: CalibrationData = this.calibrationData): ArrayList<Vector3D> {
        val forwardModel = HelioStat(ProbabilisticHelioStatParameters(params))
        var residual = ArrayList<Vector3D>(calibrationData.size)
        for (dataPoint in calibrationData) {
            val plane = forwardModel.computeHeliostatPlane(
                    ConstantDoubleVertex(dataPoint.control.pitch.toDouble()),
                    ConstantDoubleVertex(dataPoint.control.rotation.toDouble())
            )
            val modelledPlane = Geometry.cartesianToSpherical(plane.getValue())
            val observedPlane = Geometry.cartesianToSpherical(Geometry.sphericalToCartesian(Vector3D(dataPoint.length, dataPoint.pitch, dataPoint.rotation)))
            println("${dataPoint.control.pitch} ${dataPoint.control.rotation} ${observedPlane.x - modelledPlane.x} ${observedPlane.y - modelledPlane.y} ${observedPlane.z - modelledPlane.z}")
            residual.add(modelledPlane.subtract(observedPlane))
        }
        return residual
    }

    private fun inferPivotPoint(calibrationData: CalibrationData): Vector3D {
        val helioStat = HelioStat(ProbabilisticHelioStatParameters())

        for (dataPoint in calibrationData) {
            val modelledLength = helioStat.computePlaneDistanceFromOrigin(dataPoint.pitch, dataPoint.rotation)
            GaussianVertex(modelledLength, 0.001).observe(dataPoint.length)
        }
        val model = BayesNet(helioStat.params.pivotPoint.x.connectedGraph)
        val optimiser = GradientOptimizer(model)
        optimiser.maxAPosteriori(10000,
                NonLinearConjugateGradientOptimizer(
                        NonLinearConjugateGradientOptimizer.Formula.POLAK_RIBIERE,
                        SimpleValueChecker(1e-15, 1e-15)))
        return (helioStat.params.pivotPoint.getValue())
    }

    private fun inferServoParams(initialGuess: HelioStatParameters, calibrationData: CalibrationData): HelioStatParameters {
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

    private fun inferServoParamsLinear(calibrationData: CalibrationData): HelioStatParameters {
        val params = ProbabilisticHelioStatParameters()
        params.fixAxisParametersToSpherical()
        val helioStat = HelioStat(params)

        for (dataPoint in calibrationData) {
            val sphericalNorm = helioStat.linearSphericalNormalModel(dataPoint.control)
            GaussianVertex(sphericalNorm.y, 0.001).observe(dataPoint.pitch)
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

    private fun runRansacForMaxInliers(): CalibrationData {
        val acceptedParams = mutableMapOf<HelioStatParameters, ArrayList<CalibrationData.DataPoint>>()
        var bestCalibrationData = CalibrationData()

        while (acceptedParams.size < 10) {
            val inliersByParams = ransacIteration()
            val params = inliersByParams.first
            val inliers = inliersByParams.second
            val rejectionRate = inliers.size / calibrationData.size.toDouble()
            println("Rejection rate of $rejectionRate")
            if (rejectionRate <= rejectionRateToInstantlyAccept) {
                bestCalibrationData.addAll(inliers)
            } else if (rejectionRate <= maxAcceptableRejectionRate) {
                acceptedParams.put(params, inliers)
            }
        }

        val bestInliers = acceptedParams.maxBy { (params, inliers) -> inliers.size / calibrationData.size.toDouble() }!!.value
        bestCalibrationData.addAll(bestInliers)
        return bestCalibrationData
    }

    private fun ransacIteration(): Pair<HelioStatParameters, ArrayList<CalibrationData.DataPoint>> {
        val subsample = calibrationData.randomSubSample(ransacSampleSize)
        val params = inferHelioStatParams(subsample)
        val helio = HelioStat(params)

        val inliers = ArrayList<CalibrationData.DataPoint>()
        for (dataPoint in calibrationData) {
            val logLikelihood = helio.getLogLikelihood(dataPoint)
            if (logLikelihood > ransacLogLikelihoodRejectionThreshold) {
//                println("Inlier with logLikelihood of $logLikelihood added")
                inliers.add(dataPoint)
//            } else {
//                println("Inlier with logLikelihood of $logLikelihood rejected")
            }
        }

        return Pair(params, inliers)
    }
}
