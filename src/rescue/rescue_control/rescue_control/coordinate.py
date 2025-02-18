#!/usr/bin/env python3

# 5개의 waypoint -> 중앙 위치, 각 벽의 위치

class coordinate:
    def __init__(self):
        self.coordinates = {
            0: [0.0, 0.0],
            1: [3.1, 1.2],
            2: [3.1, 0.1],
            3: [3.1, -0.9],
            4: [2.0, 1.2],
        }

    def get_coordinate(self):
        return self.coordinates.values()

def main(args=None):
    coord = coordinate()
    coord.get_coordinate()

if __name__ == "__main__":
    main()

# 초기 init 
    # orientation:
    #   x: 0.0
    #   y: 0.0
    #   z: -0.002841892489186113
    #   w: 0.9999959618153865

# home center point
    # position:
    #   x: 1.0406082440422368
    #   y: 2.143217898806981
    #   z: 0.0
    # orientation:
    #   x: 0.0
    #   y: 0.0
    #   z: 0.12451203207870248
    #   w: 0.9922180979339331

# first_goal
    # position:
    #   x: 4.171595050520586
    #   y: 3.513021151832193
    #   z: 0.0
    # orientation:
    #   x: 0.0
    #   y: 0.0
    #   z: 0.9825628613297465
    #   w: 0.18593069551717764

# second_goal
    # position:
    #   x: 2.916289124046191
    #   y: -0.7051426276855381
    #   z: 0.0
    # orientation:
    #   x: 0.0
    #   y: 0.0
    #   z: 0.8197098460012595
    #   w: 0.5727789873664985

# third_goal
    # position:
    #   x: -1.6939824402772348
    #   y: 0.18061153674647
    #   z: 0.0
    # orientation:
    #   x: 0.0
    #   y: 0.0
    #   z: 0.11795598946703863
    #   w: 0.9930188238643072

# final_goal
    # position:
    #   x: -0.9898205787044733
    #   y: 4.635759060827161
    #   z: 0.0
    # orientation:
    #   x: 0.0
    #   y: 0.0
    #   z: -0.6081544983136189
    #   w: 0.7938186859610389
