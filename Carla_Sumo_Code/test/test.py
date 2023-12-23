import unittest
import os
import sys
directory = os.path.normpath(os.getcwd() + os.sep + os.pardir)
sys.path.insert(0, directory)
import conflictCAVS

class TestSearchForConflictCAVS(unittest.TestCase):
    def test_case_1(self):
        table = [
            [1, 0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1],
            [2, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 0, 2, 2],
            [3, 0, 0, 0, 0, 0, 2, 0, 0, 1, 0, 0, 0, 3, 3],
            [4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 4]
        ]
        egocar = {'id': [2,1,1,3,6,4]}
        ip, index, position = conflictCAVS.search_for_conflictCAVS(table, egocar)
        self.assertEqual(ip, -1)
        self.assertEqual(index, [-1,-1])
        self.assertEqual(position, [-1,-1])
    # def test_case_2(self):
    #     table = [
    #         [1, 0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1],
    #         [2, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 0, 2, 2],
    #         [3, 0, 0, 0, 0, 0, 2, 0, 0, 1, 0, 0, 0, 3, 3],
    #         [4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 4]
    #     ]
    #     egocar = {'id': [1,2,2,2,8,9,10,11]}
    #     ip, index, position = conflictCAVS.search_for_conflictCAVS(table, egocar)
    #     self.assertEqual(ip, -1)
    #     self.assertEqual(index, [-1,-1,-1,-1])
    #     self.assertEqual(position, [-1,-1,-1,-1])
    #
    def test_case_3(self):
        table = [
            [1, 0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1],
            [2, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 0, 2, 2],
            [3, 0, 0, 0, 0, 0, 2, 0, 0, 1, 0, 0, 0, 3, 3],
            [4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 4],
            [5, 0, 4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 6, 5],
            [6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 4, 6],
            [7, 0, 0, 1, 0, 0, 2, 0, 3, 0, 0, 0, 4, 8, 7]
        ]
        egocar = {'id': [2,3,3,5,9,6]}
        ip, index, position = conflictCAVS.search_for_conflictCAVS(table, egocar)
        self.assertEqual(ip, -1)
        self.assertEqual(index, [2,1])
        self.assertEqual(position, [2,1])

if __name__ == '__main__':
    unittest.main()