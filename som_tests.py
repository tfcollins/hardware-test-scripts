import unittest

import numpy as np
import time
import som_functions

class TestADRV9009ZU11EG(unittest.TestCase):

    measurements = 100
    uri = "ip:192.168.1.217"
    prints = True

    def test_mean_over_time(self):

        LO = 1000000000
        sync = True
        phase_diff1 = som_functions.run_hardware_tests(LO,sync,self.measurements,self.uri)
        var1 = np.var(phase_diff1)
        m1 = np.mean(phase_diff1)

        time.sleep(5)

        sync = False
        phase_diff2 = som_functions.run_hardware_tests(LO,sync,self.measurements,self.uri)
        var2 = np.var(phase_diff2)
        m2 = np.mean(phase_diff2)

        if self.prints:
            print("Run 1 Variance %f (Degress^2)" % (var1))
            print("Run 2 Variance %f (Degress^2)" % (var2))
            print("Run 1 Mean Diff %f (Degress)" % (m1))
            print("Run 2 Mean Diff %f (Degress)" % (m2))

        self.assertAlmostEqual(m1, m2, places=0)

        self.assertLess(var1, 0.05)
        self.assertLess(var2, 0.05)

    def test_lo_change(self):

        LO = 1000000000
        sync = True
        phase_diff1 = som_functions.run_hardware_tests(LO,sync,self.measurements,self.uri)
        var1 = np.var(phase_diff1)
        m1 = np.mean(phase_diff1)

        LO = 2000000000
        sync = False
        som_functions.run_hardware_tests(LO,sync,self.measurements,self.uri)

        LO = 1000000000
        phase_diff2 = som_functions.run_hardware_tests(LO,sync,self.measurements,self.uri)
        var2 = np.var(phase_diff2)
        m2 = np.mean(phase_diff2)

        if self.prints:
            print("Run 1 Variance %f (Degress^2)" % (var1))
            print("Run 2 Variance %f (Degress^2)" % (var2))
            print("Run 1 Mean Diff %f (Degress)" % (m1))
            print("Run 2 Mean Diff %f (Degress)" % (m2))

        self.assertAlmostEqual(m1, m2, places=0)

        self.assertLess(var1, 0.05)
        self.assertLess(var2, 0.05)

if __name__ == '__main__':
    unittest.main()
