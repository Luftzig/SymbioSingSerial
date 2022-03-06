DEBUG_TAG = "DEBUG: "
DEBUG = False


def debug(*args, **kwargs):
    if DEBUG:
        print("DEBUG:", *args, **kwargs)