from cv2 import waitKey

key_q = ord('q')
key_w = ord('w')
key_e = ord('e')
key_r = ord('r')
key_t = ord('t')
key_y = ord('y')

key_none = 255

def key():
    return waitKey(1) & 0xFF
