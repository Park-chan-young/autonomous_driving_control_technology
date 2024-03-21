
# pygame 라이브러리 import
import pygame

# pygame을 초기화 시켜주는 함수
pygame.init()
 
# pygame window size 설정
size  = [400, 300]

# 설정된 windows size를 적용하는 함수
screen= pygame.display.set_mode(size)

# windows title을 정하는 함수
pygame.display.set_caption("Game Title")

# 게임이 끝나면 루프를 종료시키기 위해 선언하는 변수
done = False

# 게임의 루프 주기 (FPS)를 설정하기 위한 시간 객체 생성
clock = pygame.time.Clock()

# 플레이어의 위치 초기화
player_location = [200, 150] # [X좌표, Y좌표]

# 키를 눌렀을때 몇 픽셀만큼 갈것인가 결정하는 변수 (속력 설정)
speed = 10

# 게임의 루프
while not done:
    # 게임의 주기를 설정(FPS) -> 현재 30 이니 30fps 이다.
    clock.tick(30)

    # 게임 이벤트 감지 -> 여기서는 종료 이벤트만 확인하여, 종료 이벤트 발생시 루프 종료 변수인 done을 True로 변경
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    
    # 입력된 키보드값을 가져와서 pressed 변수에 입력
    pressed = pygame.key.get_pressed()

    # 화면좌표는 왼쪽 위가 0,0 이고, 오른쪽으로 갈수록 X 좌표가 커지며, 아래로 갈수록 Y좌표가 커진다.
    # 화살표 위키를 눌렀을 경우 위로 가야하기 때문에 y좌표가 줄어야 하는데, 속력을 루프 전에 지정했으므로, 그만큼 빼준다.
    if pressed[pygame.K_UP]:
        player_location[1] -= speed
    
    # 화살표 아래키를 눌렀을 경우 아래로 가야하기 때문에 Y좌표가 늘어야 한다. 그러므로 속력만큼 Y를 더해준다.
    elif pressed[pygame.K_DOWN]:
        player_location[1] += speed

    # x좌표와 y좌표를 구분하기 위해 각각의 IF로 나눠서 작성하였다. (예를 들면 대각선 이동 같은 상황을 대비)
    # 실제로 아래 if를 위의 if에 편입시킬 시 대각선 이동이 상당히 힘들다. 다만, 나누게 되면 쉽게 해결 가능하다.
    # 화살표 오른쪽 키를 눌렀을 경우 오른쪽으로 가야 하기 때문에 X좌표가 늘어나야 한다. 지정된 속도값만큼 더해준다.
    if pressed[pygame.K_RIGHT]:
        player_location[0] += speed

    # 화살포 왼쪽 키를 눌렀을 경우 왼쪽으로 가야하기 때문에 Y좌표가 늘어나야 한다. 지정된 속도 값만큼 빼준다.
    elif pressed[pygame.K_LEFT]:
        player_location[0] -= speed

    # 랜더링이란 화면에 그림 따위를 구현하는 것을 말한다.
    # 파이게임에서 랜더링은 제일 뒤쪽부터 하나하나 그려나가는 것이다.
    # 나중에 그린 것이 먼저 그린거 보다 항상 위에 존재한다.

    # 배경색을 흰색으로 덮는다.
    # 배경색을 먼저 그렸으므로, 앞으로 그릴 플레이어는 항상 배경 위에 그려져 있을 것이다.
    screen.fill((255,255,255))

    # 키보드로 계산된 플레이어 위치에 플레이어를 그려주는 함수이다. 여기서 플레이어는 단순하게 원으로 대체한다.
    pygame.draw.circle(screen, (0, 0, 255), player_location, 40)

    # 파이게임의 게임 화면을 갱신하는 함수이다.
    pygame.display.flip()

# 파이게임을 종료하는 함수이다.
pygame.quit()
