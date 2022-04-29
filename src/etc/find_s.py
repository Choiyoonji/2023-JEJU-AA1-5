#!/usr/bin/env python
#-*-coding:utf-8-*-

import sys, os

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")

# Module import
from global_path import GlobalPath


# Parameter
# path_name = .npy
# coord = [[x1,y1],
#          [x2,y2]] -> 미션 시작 좌표들(s좌표 알고 싶은 좌표들)
# 이렇게 설정하고 코드 돌리면 coord의 s좌표가 나옴 

class find_s:
    def __init__(self):
        GLOBAL_NPY = path_name
        PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/"
        gp_name = PATH_ROOT + GLOBAL_NPY

        self.GB = GlobalPath(gp_name)

    def main(self):
        num = 0
        for i in coord:
            num += 1
            s_t, p_t = self.GB.xy2sl(i[0], i[1], mode = 1)
            print(num),i, '의 s 좌표는 =>>', s_t


if __name__ == '__main__':
    F = find_s()
    F.main()