import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'real_fly/stage2_offline/scripts'))
from verify_departure_geometry import extrema


class PolynomialDepartureTests(unittest.TestCase):
    def test_interior_dip_despite_safe_endpoints(self):
        # z=.25+.2*t-.2*t^2: endpoints alone hide 5 cm of reversal.
        z=np.array([-.2,.2,.25])
        self.assertAlmostEqual(extrema(z,1)[1],.30)
        self.assertAlmostEqual(extrema(np.polyder(z),1)[0],-.2)

    def test_stationary_endpoints_monotone_septic(self):
        # Rest-to-rest PVAJ-continuous seventh-order rise.
        z=np.array([-20.,70.,-84.,35.,0.,0.,0.,0.])
        lo,hi=extrema(z,1)
        self.assertAlmostEqual(lo,0)
        self.assertAlmostEqual(hi,1)
        self.assertGreaterEqual(extrema(np.polyder(z),1)[0],-1e-9)

    def test_time_normalization(self):
        self.assertAlmostEqual(extrema([-.002,.02,.25],10)[1],.30)


if __name__=='__main__':unittest.main()
