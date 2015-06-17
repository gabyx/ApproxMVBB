import io
import numpy as np;

def parseNDArray(string, dt, deli=" ", comm='#'):
    if string is None:
        return np.ndarray(0,dtype=dt);
    fileWrapper = io.BytesIO(string.encode())
    #build structred types (dtype for numpy)
    a = np.atleast_1d(np.genfromtxt(fileWrapper,dtype=dt, delimiter=deli, comments=comm));
    # stupid hack because numpy returns wrong shape for single rows!
    #if not np.shape(a) :
        #a.shape=(1,);
    return a;

