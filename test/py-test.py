import unittest

import frankx


class FrankxTest(unittest.TestCase):
    def test_affine(self):
        self.assertEqual('foo'.upper(), 'FOO')


if __name__ == '__main__':
    unittest.main()
    # r = frankx.Robot('172.16.0.2')
    # s = r.current_pose([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # print(s)