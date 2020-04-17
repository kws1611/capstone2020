import serial, time

'''
gps ={ 'gprmc': bytearray(),
    'gpvtg':bytearray(),
    'gpgga':bytearray(),
    'gpgsa':bytearray(),
    'gpgsv':bytearray(),
    'gpgll':bytearray()
}
'''

gpss = bytearray()
prevT = 0
curT = 0
ser = serial.serialwin32.Serial(port = "COM4")                   # 라즈베리파이 포트로 바꿔야됨

while True:
    '''
    data = ser.read()
    if data == '$':
        i = i+1
        print(i)
    elif i//2 == 0:
        gps["gprmc"] = gps["gprmc"] + data
    elif i//2 == 1:
        gps["gpvtg"] = gps["gpvtg"] + data
    elif i//2 == 2:
        gps["gpgga"] = gps["gpgga"] + data
    elif i//2 == 3:
        gps["gpgsa"] = gps["gpgsa"] + data
    elif i//2 == 4:
        gps["gpgsv"] = gps["gpgsv"] + data
    elif i//2 == 5:
        gps["gpgll"] = gps["gpgll"] + data
    elif i//2 == 6:
        i = 0
        print(gps)
    '''
    
    data = ser.read()                         # 1 바이트씩 입력받음
    gpss = gpss + data                        # 1바이트씩 더함
    
    curT = time.time_ns()
    tdiff = curT-prevT                        #curT = 현재 시간, prevT = 이전 데이터를 입력받은 시간
    prevT = curT

    if tdiff > 100000000:                     #0.1초보다 길면 그만 더하고 지금까지 받은 것 출력 그리고 gpss (bytearray) 리셋
        str_gps = gpss.decode('utf-8')
        print(tdiff)
        print(str_gps)
        gpss.clear()


'''
gprmc = Recommended minimum specific GPS/Transit data

GPVTG = ground speed???

gpgsa gpgsv =  위성정보

gpgga gpggl = 위도 경도 등 위치정보

eg3. $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
1    = UTC of Position
2    = Latitude
3    = N or S
4    = Longitude
5    = E or W
6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
7    = Number of satellites in use [not those in view]
8    = Horizontal dilution of position
9    = Antenna altitude above/below mean sea level (geoid)
10   = Meters  (Antenna height unit)
11   = Geoidal separation (Diff. between WGS-84 earth ellipsoid and
       mean sea level.  -=geoid is below WGS-84 ellipsoid)
12   = Meters  (Units of geoidal separation)
13   = Age in seconds since last update from diff. reference station
14   = Diff. reference station ID#
15   = Checksum

eg3. $GPGLL,5133.81,N,00042.25,W*75
               1    2     3    4 5

      1    5133.81   Current latitude
      2    N         North/South
      3    00042.25  Current longitude
      4    W         East/West
      5    *75       checksum

http://aprs.gids.nl/nmea/ =========>>>>> more info
'''


'''
>>실행결과<<

825047200
GPRMC,,V,,,,,,,,,,N*53
$GPVTG,,,,,,,,,N*30
$GPGGA,,,,,,0,00,99.99,,,,,,*48
$GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*30
$GPGSV,1,1,02,27,,,23,32,,,24*78
$GPGLL,,,,,,V,N*64
$
814046600
GPRMC,,V,,,,,,,,,,N*53
$GPVTG,,,,,,,,,N*30
$GPGGA,,,,,,0,00,99.99,,,,,,*48
$GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*30
$GPGSV,1,1,03,15,,,23,27,,,23,32,,,24*7C
$GPGLL,,,,,,V,N*64
$
825047100
GPRMC,,V,,,,,,,,,,N*53
$GPVTG,,,,,,,,,N*30
$GPGGA,,,,,,0,00,99.99,,,,,,*48
$GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*30
$GPGSV,1,1,02,27,,,23,32,,,24*78
$GPGLL,,,,,,V,N*64
$
841048100
GPRMC,,V,,,,,,,,,,N*53
$GPVTG,,,,,,,,,N*30
$GPGGA,,,,,,0,00,99.99,,,,,,*48
$GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*30
$GPGSV,1,1,00*79
$GPGLL,,,,,,V,N*64
$
842048100
GPRMC,,V,,,,,,,,,,N*53
$GPVTG,,,,,,,,,N*30
$GPGGA,,,,,,0,00,99.99,,,,,,*48
$GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*30
$GPGSV,1,1,00*79
$GPGLL,,,,,,V,N*64

매번 출력 첫번째 긴 숫자는 gps 입력주기 ----- nano second 단위로 0.8초마다 입력을 받는다

한 바이트씩 받은 출력을 계속 더하고 입력주기가 되면 바이트 변수를 초기화

위치정보가 안나와서 일단 바이트로 받은 정보를 스트링으로 바꾸고 출력한 결과를 받아봄

위치정보를 받을 수 있으면 gpgga 나 gpgll 정보를 받아 위도 경도 정보를 string 분리로 분리하고 integer로 바꾸어 

변수에 저장후 다른 장치에 전달(ros topic)

ros 에 사용될 수 있게 바꿔야됨

'''