import io.improbable.keanu.algorithms.variational.GradientOptimizer
import io.improbable.keanu.network.BayesNet
import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import io.improbable.keanu.vertices.dbl.probabilistic.GaussianVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.apache.commons.math3.optim.SimpleValueChecker
import org.apache.commons.math3.optim.nonlinear.scalar.gradient.NonLinearConjugateGradientOptimizer
import java.util.*

class HelioStatCalibration : ArrayList<HelioStatCalibration.DataPoint> {

    class DataPoint( val length : Double, val pitch : Double, val rotation : Double, val control: ServoSetting) {
        override fun toString() : String {
            return "${control.pitch} ${control.rotation} $pitch $rotation $length"
        }
    }

    constructor(array : List<DataPoint>) : super(array) {}
    constructor(data : Array<DataPoint>) { this.addAll(data) }
    constructor() : super()

    fun inferAllParams() : HelioStatParameters {
        var params = inferServoParamsLinear()
        params = inferServoParams(params)
        params.pivotPoint = inferPivotPoint()
        return params
    }

    fun inferPivotPoint() : Vector3D {
        val helioStat = HelioStat(ProbabilisticHelioStatParameters())

        for(dataPoint in this) {
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

    fun inferServoParams(initialGuess : HelioStatParameters) : HelioStatParameters {
        val params = ProbabilisticHelioStatParameters()
        val helioStat = HelioStat(params)

        helioStat.params.pitchParameters.m.value = initialGuess.pitchParameters.m
        helioStat.params.pitchParameters.c.value = initialGuess.pitchParameters.c
        helioStat.params.rotationParameters.m.value = initialGuess.rotationParameters.m
        helioStat.params.rotationParameters.c.value = initialGuess.rotationParameters.c

        for (dataPoint in this) {
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

    fun inferServoParamsLinear() : HelioStatParameters {
        val params = ProbabilisticHelioStatParameters()
        params.fixAxisParametersToSpherical()
        val helioStat = HelioStat(params)

        for (dataPoint in this) {
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


    fun calculateResiduals(params : HelioStatParameters) : ArrayList<Vector3D> {
        val forwardModel = HelioStat(ProbabilisticHelioStatParameters(params))
        var residual = ArrayList<Vector3D>(this.size)
        for(dataPoint in this) {
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
}