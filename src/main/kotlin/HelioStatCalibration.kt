import io.improbable.keanu.algorithms.variational.GradientOptimizer
import io.improbable.keanu.network.BayesNet
import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import io.improbable.keanu.vertices.dbl.probabilistic.GaussianVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.apache.commons.math3.optim.SimpleValueChecker
import org.apache.commons.math3.optim.nonlinear.scalar.gradient.NonLinearConjugateGradientOptimizer
import java.util.*

class HelioStatCalibration : ArrayList<HelioStatCalibration.DataPoint> {

    class DataPoint( val length : Double, val pitch : Double, val rotation : Double, val control: ServoSetting) {}

    constructor(array : List<DataPoint>) : super(array) {}
    constructor(data : Array<DataPoint>) { this.addAll(data) }
    constructor() : super()

    fun inferAllParams() : HelioStatParameters {
        val params = inferServoParams()
        params.pivotPoint = inferPivotPoint()
        return params
    }

    fun inferPivotPoint() : Vector3D {
        val helioStat = HelioStat(ProbabilisticHelioStatParameters())
        for(dataPoint in this) {
            val modelledLength = helioStat.computePlaneDistanceFromOrigin(dataPoint.pitch,dataPoint.rotation)
            GaussianVertex(modelledLength, 0.001).observe(dataPoint.length)
        }
        val model = BayesNet(helioStat.params.pivotPoint.x.connectedGraph)
        val optimiser = GradientOptimizer(model)
        optimiser.maxAPosteriori(10000,
                NonLinearConjugateGradientOptimizer(
                        NonLinearConjugateGradientOptimizer.Formula.POLAK_RIBIERE,
                        SimpleValueChecker(1e-15, 1e-15)))
        return(helioStat.params.pivotPoint.getValue())
    }

    fun inferServoParams() : HelioStatParameters {
        val helioStat = HelioStat(ProbabilisticHelioStatParameters())
        for (dataPoint in this) {
            val sphericalNorm = helioStat.servoSignalToUnitSpherical(dataPoint.control)
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

    fun calculateResiduals(params : HelioStatParameters) : ArrayList<Vector3D> {
        val forwardModel = HelioStat(ProbabilisticHelioStatParameters(params))
        var residual = ArrayList<Vector3D>(this.size)
        for(dataPoint in this) {
            val plane = forwardModel.computeHeliostatPlaneSpherical(
                    ConstantDoubleVertex(dataPoint.control.pitch.toDouble()),
                    ConstantDoubleVertex(dataPoint.control.rotation.toDouble())
            )
            val modelledPlane = plane.getValue()
            residual.add(modelledPlane.subtract(Vector3D(dataPoint.length, dataPoint.pitch, dataPoint.rotation)))
        }
        return residual
    }
}