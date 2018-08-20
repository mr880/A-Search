# This is where we bundle information regarding our map points. We have the x/y, and the scalar distance of the point to the goal.
class MapPoint:

    def __init__(self, x, y, goalDistance):
        self.x = x
        self.y = y
        self.goalDistance = goalDistance

    def returnX(self):
        return self.x

    def returnY(self):
        return self.y

    def returnGoalDistance(self):
        return self.goalDistance
