def init():
    """
        initializing the object tracking, should be called first before calling
        trackObject()
    """
    print("object tracking initializing...")


def trackObject():
    """
        initializing the object tracking, should be called after calling
        init()
    """
    print("object tracking launched...")


def deinit():
    """
        deinitializing the object tracking mission
    """
    print("object tracking deinitializing...")


if __name__ == "__main__":
    init()
    trackObject()
    deinit()
