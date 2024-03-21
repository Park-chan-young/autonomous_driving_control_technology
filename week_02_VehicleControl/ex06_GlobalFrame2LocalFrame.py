import numpy as np
import matplotlib.pyplot as plt

# Global에서 Local로 point 좌표를 변환하는 클래스
class Global2Local(object):
    def __init__(self, num_points):
        # 좌표 수
        self.n = num_points
        # nx2 0행렬 (글로벌 좌표 초기화)
        self.GlobalPoints = np.zeros((num_points,2))
        # nx2 0행렬 (로컬 좌표 초기화)
        self.LocalPoints = np.zeros((num_points,2))
    
    def convert(self, points, Yaw_ego, X_ego, Y_ego):
        # n개의 좌표에 같은 작업을 반복해야 한다.
        for i in range(self.n):
            # 글로벌 좌표 - 로컬 좌표 origin (크기)
            x_diff = points[i][0] - X_ego
            y_diff = points[i][1] - Y_ego
            # 로컬 좌표로 표현하기 위해 회전변환을 수행 (cos sin; -sin cos) (차량 좌표계에서 z 축은 아래방향이므로 부호 반대)
            self.LocalPoints[i][0] = x_diff * np.cos(Yaw_ego) + y_diff * np.sin(Yaw_ego)
            self.LocalPoints[i][1] = - x_diff * np.sin(Yaw_ego) + y_diff * np.cos(Yaw_ego)
            
# input: point / output: coefficient of curvature (vector x)
class PolynomialFitting(object):
    def __init__(self, num_degree, num_points):
        # m 차 곡선
        self.m = num_degree
        # n 개 좌표
        self.n = num_points
        # 최소제곱해 행렬 식 초기화 (Ax=b)
        # A: n x (m+1) 행렬 초기화
        self.A = np.zeros((self.n, self.m+1))
        # b: n x 1 벡터 초기화
        self.b = np.zeros((self.n,1))
        # x: (m+1) x 1 행렬 초기화
        self.coeff = np.zeros((self.m + 1, 1))
        
    def fit(self, points):
        # 측정값 (x_k, y_k)를 A, b 행렬 원소로 채우기
        # A 행렬은 x의 n차 원소로 차곡차곡 채우기
        # b 벡터는 순서대로 y 원소로 채우기
        for i in range(self.n):
            x_val = points[i][0]
            y_val = points[i][1]
            for j in range(self.m+1):
                self.A[i][j] = x_val ** j
            self.b[i] = y_val
            
        # { x = inv(T(A)A)T(A)b } => A: 특징행렬(feature matrix) / b: 응답벡터(종속변수)(response vector)
        # numpy 내장함수에 lstsq 있음 / 끝에 '[0]'은 여러개의 해 중 첫번째 값인 '최적해'를 반환하라는 의미
        self.coeff = np.linalg.lstsq(self.A, self.b, rcond=None)[0]

# input: coefficient of curvature (vector x) / output: fitted points in local frame (x_local, fitted y_local)
class PolynomialValue(object):
    def __init__(self, num_degree, num_points):
        # m 차 곡선
        self.m = num_degree
        # n 개 좌표 (결과값을 받을 저장공간이다. 적당히 넉넉하게 설정해 주는 것이 좋다.)
        self.n = num_points
        # self.x = np.zeros((1, self.nd+1))
        # x 변수 행렬: n x (m+1) 행렬 초기화
        self.x = np.zeros((self.n, self.m+1))
        # y 변수 벡터: n x 1 행렬 초기화
        self.y = np.zeros((self.n, 1))
        # local 좌표 행렬: n x 2 행렬 초기화
        self.points = np.zeros((self.n, 2))
        
    def calculate(self, coeff, x):
        # 위에서 계산한 coeff 값과 local x를 계산한 결과를 반환
        # x 행렬은 x_local의 'x^n' 계산 결과로 차곡차곡 채우기
        # y 벡터는 x_local의 'cx' 계산 결과로 채우기
        for i in range(self.n):
            for j in range(self.m + 1):
                self.x[i][j] = x[i] ** j
        # np.dot => 행렬의 곱셈 연산자
        self.y = np.dot(self.x, coeff)
        self.points[:, 0] = x
        # self.y는 n x 1의 다차원(2차원) 배열이므로 1차원으로 평탄화(flatten())해줘야 한다.
        # 결과적으로, local x 값에 기반한 fitting 된 (계산된) y값을 반환한다.
        self.points[:, 1] = self.y.flatten()


if __name__ == "__main__":
    # 3차 함수 곡선
    num_degree = 3
    # 4개의 좌표
    num_point = 4
    # 변환 할 글로벌 좌표 (1,2), (3,3), (4,4), (5,5)
    points = np.array([[1,2],[3,3],[4,4],[5,5]])
    # local frame의 origin 좌표 및 orientation (Pose)
    X_ego = 2.0
    Y_ego = 0.0
    Yaw_ego = np.pi/4
    # 최종 변환된 결과값의 x좌표 지정 (0부터 10까지 0.5 간격으로) (20개 준비 but 실제론 앞에서부터 4개만 쓰임)
    x_local = np.arange(0.0, 10.0, 0.5)
    
    # 사용할 3개의 클래스 초기화
    frameconverter = Global2Local(num_point)
    polynomialfit = PolynomialFitting(num_degree,num_point)
    polynomialvalue = PolynomialValue(num_degree,np.size(x_local))
    # Input(글로벌 좌표, 로컬좌표 origin 정보) / Output(변환 된 로컬 좌표)
    frameconverter.convert(points, Yaw_ego, X_ego, Y_ego) # => LocalPoints
    # Input(변환 된 로컬 좌표) / Output(곡선 방정식의 계수)
    polynomialfit.fit(frameconverter.LocalPoints) # => coeff
    # Input(곡선 방정식의 계수, 로컬 x좌표) / Output(피팅 된 x,y값)
    polynomialvalue.calculate(polynomialfit.coeff, x_local) # => points[][]
    
    plt.figure(1)
    for i in range(num_point):
        plt.plot(points[i][0],points[i][1],'b.')
    plt.plot(X_ego,Y_ego,'ro',label = "Vehicle")
    plt.plot([X_ego, X_ego+0.2*np.cos(Yaw_ego)],[Y_ego, Y_ego+0.2*np.sin(Yaw_ego)],'r-')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.title("Global Frame")
    plt.grid(True)    
    
    plt.figure(2)
    for i in range(num_point):
        plt.plot(frameconverter.LocalPoints[i][0],frameconverter.LocalPoints[i][1],'b.')
    # points 행렬의 전치행렬 첫번 째 행벡터(x), 두번 째 행벡터(y)로 파란색 점선 그리는 것 (곡선 시각화)
    plt.plot(polynomialvalue.points.T[0],polynomialvalue.points.T[1],'b:')
    plt.plot(0.0, 0.0,'ro',label = "Vehicle")
    plt.plot([0.0, 0.5],[0.0, 0.0],'r-')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend(loc="best")
    plt.axis((-10,10,-10,10))
    plt.title("Local Frame")
    plt.grid(True)   
    
    plt.show()