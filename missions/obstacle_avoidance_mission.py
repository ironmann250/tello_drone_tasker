def init(tello):
    """
        initializing the obstacle avoidance, should be called first before calling
        trackObject()
    """
    print("obstacle avoidance initializing...")


def avoidObstacles(tello):
    """
        initializing the obstacle avoidance, should be called after calling
        init()
    """
    print("obstacle avoidance launched...")


def deinit():
    """
        deinitializing the obstacle avoidance mission
    """
    print("obstacle avoidance deinitializing...")


if __name__ == "__main__":
    init()
    trackObject()
    deinit()
