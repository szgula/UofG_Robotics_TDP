from enum import Enum


class CollisionTypes(Enum):
    NO = 0
    WALL_VERTICAL = 1
    WALL_HORIZONTAL = 2
    WALL_CORNER = 3
    BALL = 4
    PLAYER = 5
