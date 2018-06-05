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
        val params = inferServoParamsLinear()
        params.pivotPoint = inferPivotPoint()
        return params
    }

    fun inferPivotPoint() : Vector3D {
        val helioStat = HelioStat(ProbabilisticHelioStatParameters())

//        helioStat.params.pivotPoint.x.value = 1.0
//        helioStat.params.pivotPoint.y.value = 1.0
//        helioStat.params.pivotPoint.z.value = 1.0

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

    fun inferServoParams() : HelioStatParameters {
        val params = ProbabilisticHelioStatParameters()
        val helioStat = HelioStat(params)

        helioStat.params.pitchParameters.m.value = 0.002
        helioStat.params.pitchParameters.c.value = 0.02
        helioStat.params.rotationParameters.m.value = 0.003
        helioStat.params.rotationParameters.c.value = 0.03

        for (dataPoint in this) {
            val cartesianNorm = helioStat.computeHeliostatNormal(dataPoint.control)
            val observedNorm = Geometry.sphericalToCartesian(Vector3D(1.0, dataPoint.pitch, dataPoint.rotation))
            GaussianVertex(cartesianNorm.x, 0.02).observe(observedNorm.x)
            GaussianVertex(cartesianNorm.y, 0.02).observe(observedNorm.y)
            GaussianVertex(cartesianNorm.z, 0.02).observe(observedNorm.z)
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
            val cartesianNorm = helioStat.computeHeliostatNormal(dataPoint.control)
            val observedNorm = Geometry.sphericalToCartesian(Vector3D(1.0, dataPoint.pitch, dataPoint.rotation))
            GaussianVertex(cartesianNorm.x, 0.02).observe(observedNorm.x)
            GaussianVertex(cartesianNorm.y, 0.02).observe(observedNorm.y)
            GaussianVertex(cartesianNorm.z, 0.02).observe(observedNorm.z)
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
            residual.add(modelledPlane.subtract(observedPlane))
        }
        return residual
    }
}