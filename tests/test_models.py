import math
import unittest

from tvc_rocket.models import RocketParameters, clamp, wrap_angle


class ModelsTestCase(unittest.TestCase):
    def test_clamp_limits_value(self):
        self.assertEqual(clamp(5.0, 0.0, 3.0), 3.0)
        self.assertEqual(clamp(-2.0, 0.0, 3.0), 0.0)
        self.assertEqual(clamp(2.0, 0.0, 3.0), 2.0)

    def test_wrap_angle_returns_principal_interval(self):
        wrapped = wrap_angle(3.0 * math.pi)
        self.assertGreaterEqual(wrapped, -math.pi)
        self.assertLessEqual(wrapped, math.pi)
        self.assertAlmostEqual(abs(wrapped), math.pi, places=7)

    def test_parameter_properties_are_positive(self):
        params = RocketParameters()
        self.assertGreater(params.mass_flow, 0.0)
        self.assertGreater(params.max_gimbal_rad, 0.0)
        self.assertGreater(params.max_alpha_cmd_rad, 0.0)


if __name__ == "__main__":
    unittest.main()
