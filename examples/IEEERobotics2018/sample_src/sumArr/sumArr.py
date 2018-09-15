
from ctypes import cdll, c_int
lib = cdll.LoadLibrary("./sumArr.so")

def getCenter(pts):
    return lib.getCenter((c_int * len(pts))(*pts), len(pts))

