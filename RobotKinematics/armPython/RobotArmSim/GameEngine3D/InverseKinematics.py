import numpy
import numpy as np
import scipy

from RobotArmSim.GameEngine3D.Posing.Transform import Transform


class InverseKinematics:

    @staticmethod
    def calculate_ik(num_joints, fk_func, fk_deriv, target_transform, max_iterations=1000, lin_eps=0.1, ang_eps=0.5):
        lin_error = np.inf
        ang_error = np.inf
        iterations = 0
        theta = numpy.random.rand(num_joints)
        target_transform = target_transform.homogeneous_matrix if hasattr(target_transform, 'homogeneous_matrix') else target_transform

        while (lin_error > lin_eps or ang_error > ang_eps) and iterations < max_iterations:
            fk = fk_func(theta)

            screw = scipy.linalg.logm(np.linalg.inv(fk) @ target_transform)
            screw_vec = Transform.twist_to_six_vector(screw)

            lin_error = np.linalg.norm(screw_vec[3:])
            ang_error = np.linalg.norm(screw_vec[:3])
            J = Transform.adjoint_map(np.linalg.inv(fk)) @ fk_deriv(theta)
            print("--------------------")
            delta_theta = np.linalg.pinv(J) @ screw_vec
            theta += delta_theta
            theta %= 2 * np.pi
            iterations += 1

            print(f"Iteration {iterations}, Linear Error: {lin_error}, Angular Error: {ang_error}")

        theta %= 2 * np.pi

        if iterations >= max_iterations:
            print("IK calculation did not converge within the maximum iterations.")
            print(f"Final linear error: {lin_error}, Final angular error: {ang_error}")
        else:
            print(f"IK calculation converged in {iterations} iterations.")

        return theta

