import io.improbable.keanu.algorithms.variational.GradientOptimizer
import io.improbable.keanu.network.BayesNet
import io.improbable.keanu.vertices.dbl.probabilistic.UniformVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.apache.commons.math3.optim.SimpleValueChecker
import org.apache.commons.math3.optim.nonlinear.scalar.gradient.NonLinearConjugateGradientOptimizer
import kotlin.math.roundToInt

class HelioStatNavigator(var params: ProbabilisticHelioStatParameters) {

    val servoPitchRange = UniformVertex(0.0, 4096.0)
    val servoRotationRange = UniformVertex(0.0, 4096.0)
    val targetDistance = UniformVertex(1.0, 20.0)
    val model = HelioStat(params)

    fun computeServoSetting(probabilisticTargetPoint: ProbabilisticVector3D,
                            desiredTargetPoint: Vector3D): ServoSetting {

        val targetObservationNoise = Vector3D(1.0, 1.0, 1.0)
        probabilisticTargetPoint.noisyObserve(desiredTargetPoint, targetObservationNoise)

        val net = BayesNet(probabilisticTargetPoint.x.connectedGraph)

        val optimiser = GradientOptimizer(net)
        optimiser.maxAPosteriori(10000,
                NonLinearConjugateGradientOptimizer(NonLinearConjugateGradientOptimizer.Formula.POLAK_RIBIERE,
                        SimpleValueChecker(1e-15, 1e-15)))

        return ServoSetting(servoRotationRange.value.roundToInt(), servoPitchRange.value.roundToInt())
    }

    fun computeServoSettingFromDirection(incomingSunDirection: ProbabilisticVector3D,
                                         desiredTargetPoint: Vector3D): ServoSetting {
        val probabilisticTargetPoint = model.computeTargetFromSourceDirection(servoPitchRange, servoRotationRange,
                                                                              incomingSunDirection, targetDistance)
        return computeServoSetting(probabilisticTargetPoint, desiredTargetPoint)
    }

    fun computeServoSettingFromPoint(sourcePoint: ProbabilisticVector3D, desiredTargetPoint: Vector3D): ServoSetting {

        val probabilisticTargetPoint = model.computeTargetFromSourcePoint(servoPitchRange, servoRotationRange,
                                                                          sourcePoint, targetDistance)
        return computeServoSetting(probabilisticTargetPoint, desiredTargetPoint)
    }
}