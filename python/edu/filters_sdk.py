import math


class BWBandPassFilter:
    n = 0
    A = None
    d1 = None
    d2 = None
    d3 = None
    d4 = None
    w0 = None
    w1 = None
    w2 = None
    w3 = None
    w4 = None

    def __init__(self, order, sample_rate, fl, fu):
        self.order = order
        self.sample_rate = sample_rate
        self.fl = fl
        self.fu = fu

        if order % 4 != 0:
            print("ERROR:Order has to be multiple of 4")
            return

        if fu <= fl:
            print("ERROR:Lower half-power frequency is smaller than higher half-power frequency")
            return

        self.n = int(order / 4)
        self.A = [0.0] * self.n
        self.d1 = [0.0] * self.n
        self.d2 = [0.0] * self.n
        self.d3 = [0.0] * self.n
        self.d4 = [0.0] * self.n

        self.w0 = [0.0] * self.n
        self.w1 = [0.0] * self.n
        self.w2 = [0.0] * self.n
        self.w3 = [0.0] * self.n
        self.w4 = [0.0] * self.n

        self.reset_sample_rate(sample_rate)

    def filter(self, x):
        for i in range(self.n):
            self.w0[i] = self.d1[i] * self.w1[i] + self.d2[i] * self.w2[i] + self.d3[i] * self.w3[i] + self.d4[i] * \
                         self.w4[i] + x
            x = self.A[i] * (self.w0[i] - 2.0 * self.w2[i] + self.w4[i])
            self.w4[i] = self.w3[i]
            self.w3[i] = self.w2[i]
            self.w2[i] = self.w1[i]
            self.w1[i] = self.w0[i]
        return x

    def reset_sample_rate(self, sample_rate):
        self.sample_rate = sample_rate
        a = math.cos(math.pi * (self.fu + self.fl) / sample_rate) / math.cos(math.pi * (self.fu - self.fl) / sample_rate)
        a2 = a * a
        b = math.tan(math.pi * (self.fu - self.fl) / sample_rate)
        b2 = b * b

        for i in range(self.n):
            r = math.sin(math.pi * (2.0 * i + 1.0) / (4.0 * self.n))
            s = b2 + 2.0 * b * r + 1.0
            self.A[i] = b2 / s
            self.d1[i] = 4.0 * a * (1.0 + b * r) / s
            self.d2[i] = 2.0 * (b2 - 2.0 * a2 - 1.0) / s
            self.d3[i] = 4.0 * a * (1.0 - b * r) / s
            self.d4[i] = -(b2 - 2.0 * b * r + 1.0) / s


class BWBandStopFilter:
    n = 0
    A = None
    d1 = None
    d2 = None
    d3 = None
    d4 = None
    w0 = None
    w1 = None
    w2 = None
    w3 = None
    w4 = None
    r = 0.0
    s = 0.0

    def __init__(self, order, sample_rate, fl, fu):
        self.order = order
        self.sample_rate = sample_rate
        self.fl = fl
        self.fu = fu

        if order % 4 != 0:
            print("ERROR:Order has to be multiple of 4")
            return

        if fu <= fl:
            print("ERROR:Lower half-power frequency is smaller than higher half-power frequency")
            return

        self.n = int(order / 4)
        self.A = [0.0] * self.n
        self.d1 = [0.0] * self.n
        self.d2 = [0.0] * self.n
        self.d3 = [0.0] * self.n
        self.d4 = [0.0] * self.n

        self.w0 = [0.0] * self.n
        self.w1 = [0.0] * self.n
        self.w2 = [0.0] * self.n
        self.w3 = [0.0] * self.n
        self.w4 = [0.0] * self.n

        self.reset_sample_rate(sample_rate)

    def filter(self, x):
        for i in range(self.n):
            self.w0[i] = self.d1[i] * self.w1[i] + self.d2[i] * self.w2[i] + self.d3[i] * self.w3[i] + self.d4[i] * \
                         self.w4[i] + x
            x = self.A[i] * (self.w0[i] - self.r * self.w1[i] + self.s * self.w2[i] - self.r * self.w3[i] + self.w4[i])
            self.w4[i] = self.w3[i]
            self.w3[i] = self.w2[i]
            self.w2[i] = self.w1[i]
            self.w1[i] = self.w0[i]
        return x

    def reset_sample_rate(self, sample_rate):
        self.sample_rate = sample_rate
        a = math.cos(math.pi * (self.fu + self.fl) / sample_rate) / math.cos(math.pi * (self.fu - self.fl) / sample_rate)
        a2 = a * a
        b = math.tan(math.pi * (self.fu - self.fl) / sample_rate)
        b2 = b * b

        self.r = 4.0 * a
        self.s = 4.0 * a2 + 2.0

        for i in range(self.n):
            r = math.sin(math.pi * (2.0 * i + 1.0) / (4.0 * self.n))
            s = b2 + 2.0 * b * r + 1.0
            self.A[i] = 1.0 / s
            self.d1[i] = 4.0 * a * (1.0 + b * r) / s
            self.d2[i] = 2.0 * (b2 - 2.0 * a2 - 1.0) / s
            self.d3[i] = 4.0 * a * (1.0 - b * r) / s
            self.d4[i] = -(b2 - 2.0 * b * r + 1.0) / s


class BWLowPassFilter:
    n = 0
    A = None
    d1 = None
    d2 = None

    w0 = None
    w1 = None
    w2 = None

    def __init__(self, order, sample_rate, f):

        if order % 2 != 0:
            print("ERROR:Order has to be multiple of 2")
            return

        self.n = int(order / 2)
        self.A = [0.0] * self.n
        self.d1 = [0.0] * self.n
        self.d2 = [0.0] * self.n

        self.w0 = [0.0] * self.n
        self.w1 = [0.0] * self.n
        self.w2 = [0.0] * self.n

        a = math.tan(math.pi * f / sample_rate)
        a2 = a * a

        for i in range(self.n):
            r = math.sin(math.pi * (2.0 * i + 1.0) / (4.0 * self.n))
            s = a2 + 2.0 * a * r + 1.0
            self.A[i] = a2 / s
            self.d1[i] = 2.0 * (1.0 - a2) / s
            self.d2[i] = -(a2 - 2.0 * a * r + 1.0) / s

    def filter(self, x):
        for i in range(self.n):
            self.w0[i] = self.d1[i] * self.w1[i] + self.d2[i] * self.w2[i] + x
            x = self.A[i] * (self.w0[i] + 2.0 * self.w1[i] + self.w2[i])
            self.w2[i] = self.w1[i]
            self.w1[i] = self.w0[i]
        return x


class BWHighPassFilter:
    n = 0
    A = None
    d1 = None
    d2 = None

    w0 = None
    w1 = None
    w2 = None

    def __init__(self, order, sample_rate, f):

        if order % 2 != 0:
            print("ERROR:Order has to be multiple of 2")
            return

        self.n = int(order / 2)
        self.A = [0.0] * self.n
        self.d1 = [0.0] * self.n
        self.d2 = [0.0] * self.n

        self.w0 = [0.0] * self.n
        self.w1 = [0.0] * self.n
        self.w2 = [0.0] * self.n

        a = math.tan(math.pi * f / sample_rate)
        a2 = a * a

        for i in range(self.n):
            r = math.sin(math.pi * (2.0 * i + 1.0) / (4.0 * self.n))
            s = a2 + 2.0 * a * r + 1.0
            self.A[i] = 1.0 / s
            self.d1[i] = 2.0 * (1.0 - a2) / s
            self.d2[i] = -(a2 - 2.0 * a * r + 1.0) / s

    def filter(self, x):

        for i in range(self.n):
            self.w0[i] = self.d1[i] * self.w1[i] + self.d2[i] * self.w2[i] + x
            x = self.A[i] * (self.w0[i] - 2.0 * self.w1[i] + self.w2[i])
            self.w2[i] = self.w1[i]
            self.w1[i] = self.w0[i]
        return x
