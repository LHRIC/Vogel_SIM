from numpy.polynomial import Polynomial

class FitParam():
    def __init__(self, coeffs):
        self.coeffs = coeffs
        self.order = len(coeffs) - 1

        self.polyfit = Polynomial(coef=self.coeffs)

    def get(self, x):
        return self.polyfit(x)
    
if __name__ == "__main__":
    a = FitParam([1, 1, 1])
    print(a.get(2))