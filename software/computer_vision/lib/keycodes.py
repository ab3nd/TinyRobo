from cv2 import waitKey

class KeyCode(object):
    def __init__(self):
        codes = ('abcdefghijklmnopqrstuvwxyz'
                 'ABCDEFGHIJKLMNOPQRSTUVWXYZ')

        for code in codes:
            setattr(self, code, ord(code))

    @property
    def none(self):
        return 255

def keypress():
    return waitKey(1) & 0xFF

key = KeyCode()
