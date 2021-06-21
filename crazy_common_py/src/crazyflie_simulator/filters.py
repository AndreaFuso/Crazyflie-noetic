import math
from crazy_common_py.constants import *


class lpf2pData:
    '''
        Second order low pass filter structure.
        using biquad filter with bilinear z transform

        Laplace continious form:
                            1
            H(s) = -------------------
                   s^2/w^2 + s/w*Q + 1

        Polynomial discrete form:
                   b0 + b1 z^-1 + b2 z^-2
            H(z) = ----------------------
                   a0 + a1 z^-1 + a2 z^-2

        with:
            a0 = 1
            a1 = 2*(K^2 - 1) / (K^2 + K/Q + 1)
            a2 = (K^2 - K/Q + 1) / (K^2 + K/Q + 1)
            b0 = K^2 / (K^2 + K/Q + 1)
            b1 = 2*b0
            b2 = b0
            K = tan(pi*Fc/Fs) ~ pi*Fc/Fs = Ts/(2*tau)
            Fc: cutting frequency
            Fs: sampling frequency
            Ts: sampling period
            tau: time constant (tau = 1/(2*pi*Fc))
            Q: gain at cutoff frequency

        Note that b[0]=b[2], so we don't need to save b[2]
    '''

    def __init__(self, a1, a2, b0, b1, b2, delay_element_1, delay_element_2):
        self.a1 = a1
        self.a2 = a2
        self.b0 = b0
        self.b1 = b1
        self.b2 = b2
        self.delay_element_1 = delay_element_1
        self.delay_element_2 = delay_element_2

def lpf2pInit(lpDataIn, sample_freq, cutoff_freq):
    if cutoff_freq <= 0.0:
        return
    lpDataOut = lpf2pSetCutoffFreq(lpDataIn, sample_freq, cutoff_freq)
    return lpDataOut


def lpf2pSetCutoffFreq(lpDataIn, sample_freq, cutoff_freq):
    fr = sample_freq / cutoff_freq
    ohm = math.tan(M_PI_F / fr)
    c = 1.0 + 2.0 * math.cos(M_PI_F / 4.0) * ohm + ohm * ohm

    lpDataOut = lpDataIn
    lpDataOut.b0 = ohm * ohm / c
    lpDataOut.b1 = 2.0 * lpDataOut.b0
    lpDataOut.b2 = lpDataOut.b0
    lpDataOut.a1 = 2.0 * (ohm * ohm - 1.0) / c
    lpDataOut.a2 = (1.0 - 2.0 * math.cos(M_PI_F / 4.0) * ohm + ohm * ohm) / c
    lpDataOut.delay_element_1 = 0.0
    lpDataOut.delay_element_2 = 0.0

    return lpDataOut

def lpf2pApply(lpDataIn, sample):
    delay_element_0 = sample - lpDataIn.delay_element_1 * lpDataIn.a1 - lpDataIn.delay_element_2 * lpDataIn.a2
    if not math.isfinite(delay_element_0):
        delay_element_0 = sample

    output = delay_element_0 * lpDataIn.b0 + lpDataIn.delay_element_1 * lpDataIn.b1 + lpDataIn.delay_element_2 * lpDataIn.b2
    lpDataOut = lpDataIn
    lpDataOut.delay_element_2 = lpDataIn.delay_element_1
    lpDataOut.delay_element_1 = delay_element_0

    return (lpDataOut, output)

def lpf2pReset(lpDataIn, sample):
    dval = sample / (lpDataIn.b0 + lpDataIn.b1 + lpDataIn.b2)

    lpDataOut = lpDataIn
    lpDataOut.delay_element_1 = dval
    lpDataOut.delay_element_2 = dval

    return lpf2pApply(lpDataOut, sample)