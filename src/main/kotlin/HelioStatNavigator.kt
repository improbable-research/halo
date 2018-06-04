import io.improbable.keanu.algorithms.variational.GradientOptimizer
import io.improbable.keanu.network.BayesNet
import io.improbable.keanu.vertices.dbl.DoubleVertex
import io.improbable.keanu.vertices.dbl.probabilistic.UniformVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.apache.commons.math3.optim.SimpleValueChecker
import org.apache.commons.math3.optim.nonlinear.scalar.gradient.NonLinearConjugateGradientOptimizer
import kotlin.math.roundToInt

class HelioStatNavigator {

    class NavigationQuery (var params: HelioStatParameters, var presentControl: ServoSetting,
                           var targetPoint: Vector3D, var source: Vector3D) {}

    val servoPitchRange : DoubleVertex
    val servoRotationRange : DoubleVertex
    val targetDistance : DoubleVertex
    val model : HelioStat

    constructor(params : HelioStatParameters) {
        servoPitchRange = UniformVertex(-1000.0, 8096.0)
        servoRotationRange = UniformVertex(-1000.0, 8096.0)
        targetDistance = UniformVertex(-100.0, 100.0)
        model = HelioStat(ProbabilisticHelioStatParameters(params))
        servoPitchRange.value = 2000.0
        servoRotationRange.value = 2000.0
        targetDistance.value = 5.0
    }

    private fun computeServoSetting(probabilisticTargetPoint: ProbabilisticVector3D,
                                    desiredTargetPoint: Vector3D): ServoSetting {

        val targetObservationNoise = Vector3D(0.01, 0.01, 0.01)
        probabilisticTargetPoint.noisyObserve(desiredTargetPoint, targetObservationNoise)

        val net = BayesNet(probabilisticTargetPoint.x.connectedGraph)

        val optimiser = GradientOptimizer(net)
        optimiser.maxAPosteriori(10000,
                NonLinearConjugateGradientOptimizer(NonLinearConjugateGradientOptimizer.Formula.POLAK_RIBIERE,
                        SimpleValueChecker(1e-15, 1e-15)))

        return ServoSetting(servoRotationRange.value.roundToInt(), servoPitchRange.value.roundToInt())
    }

    fun computeServoSettingFromDirection(incomingSunDirection: Vector3D,
                                         desiredTargetPoint: Vector3D): ServoSetting {
        targetDistance.value = 5.0
        val probabilisticTargetPoint = model.computeTargetFromSourceDirection(servoPitchRange, servoRotationRange,
                                                                              ProbabilisticVector3D(incomingSunDirection), targetDistance)
        return computeServoSetting(probabilisticTargetPoint, desiredTargetPoint)
    }

    fun computeServoSettingFromPoint(sourcePoint: Vector3D, desiredTargetPoint: Vector3D): ServoSetting {

        val probabilisticTargetPoint = model.computeTargetFromSourcePoint(servoPitchRange, servoRotationRange,
                                                                          sourcePoint, targetDistance)
        return computeServoSetting(probabilisticTargetPoint, desiredTargetPoint)
    }
}