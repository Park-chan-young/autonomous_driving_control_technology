import numpy as np
from scipy import integrate

# 평균 곡률을 계산할 함수
def f(x):
    return 2.0-2*np.cos(x/10)

# 적분 범위
a = 0
b = 5

# 수치 적분 수행
result, error = integrate.quad(lambda x: np.abs(-np.cos(x / 10) / 50), a, b)

# 평균 곡률 계산
average_curvature = result / (b - a)

# 결과 출력
print(average_curvature)