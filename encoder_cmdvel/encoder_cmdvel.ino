// 모터 제어 핀 정의
#define DIR1 12      // 첫 번째 모터 방향 핀
#define PWM1 11      // 첫 번째 모터 PWM 핀
#define DIR2 10      // 두 번째 모터 방향 핀
#define PWM2 9       // 두 번째 모터 PWM 핀

#define encoderPinA 2   // 첫 번째 모터의 엔코더 A 핀 (인터럽트 핀)
#define encoderPinB 7   // 첫 번째 모터의 엔코더 B 핀
#define encoderPinC 3   // 두 번째 모터의 엔코더 A 핀 (인터럽트 핀)
#define encoderPinD 5   // 두 번째 모터의 엔코더 B 핀

volatile long encoderCount1 = 0;  // 첫 번째 모터의 엔코더 카운트
volatile long encoderCount2 = 0;  // 두 번째 모터의 엔코더 카운트

// 경로 좌표 배열
const int numWaypoints = 60;
float waypoints[60][2] = {
  {-0.024997711181640625, -0.024997711181640625},
  {-0.021189117374888156, 0.0012596134283739957},
  {-0.021750640820755507, 0.026253510285187076},
  {-0.022446441612373746, 0.051247407142000156},
  {-0.02366714475556364, 0.07621688993594944},
  {-0.025003814697356574, 0.10118026921418277},
  {-0.026538848899917866, 0.1261314414609842},
  {-0.028110504196774855, 0.15108261370778564},
  {-0.029999542310861216, 0.17600937189172328},
  {-0.03176956186848656, 0.2009483371070928},
  {-0.03303604137954608, 0.2259178199010421},
  {-0.03406448377768356, 0.2508934062107073},
  {-0.03458023085568129, 0.2758873030675204},
  {-0.03473892226429598, 0.30088730344004944},
  {-0.034342193742759264, 0.32588730381257847},
  {-0.033472442753236464, 0.35086899363795965},
  {-0.031970977887112895, 0.37582626940047703},
  {-0.024643707270115556, 0.39972763694413516},
  {-0.016910552858007577, 0.42350083065775834},
  {-0.008756255861499085, 0.4471336435099147},
  {-0.0001502987020103319, 0.47060776495345635},
  {0.008882904557594884, 0.4939170914726674},
  {0.018441010168771754, 0.5170188984575361},
  {0.028939057200204843, 0.5397056663737203},
  {0.03994369603606174, 0.5621543971769825},
  {0.05145492667634244, 0.5843467803201747},
  {0.06349716318391074, 0.6062584017404333},
  {0.07644882353315552, 0.6276390172934043},
  {0.0898704545925284, 0.6487327676077257},
  {0.1037742633934613, 0.6695091351048177},
  {0.11833725189171673, 0.6898277389232135},
  {0.13335800406866838, 0.709810649377232},
  {0.14942856094876333, 0.7289634816938815},
  {0.16573105142606437, 0.7479148979919046},
  {0.18239975284632237, 0.7665489314726983},
  {0.20032577850406597, 0.783974468841734},
  {0.21832504635040095, 0.8013267640221784},
  {0.2366539040453972, 0.8183250552910977},
  {0.25614242972642387, 0.833986676618224},
  {0.27588119955180446, 0.849324811612405},
  {0.29561996937718504, 0.864669050122302},
  {0.3159446767112968, 0.8792259351048415},
  {0.33661728444121763, 0.8932823317986731},
  {0.3573020992025704, 0.9073265214610728},
  {0.37823715810827707, 0.9209861896333678},
  {0.39914780295111996, 0.9346885824156743},
  {0.4201866216239978, 0.9481956626950705},
  {0.44136582115834244, 0.9614769128929765},
  {0.4625450206926871, 0.9747642666065985},
  {0.48311997217115277, 0.988961044161897},
  {0.5037620623224939, 1.0030723724971722},
  {0.5243736348952552, 1.017220321926743},
  {0.5441795433935113, 1.0324730077009008},
  {0.5638389675145845, 1.047914902462253},
  {0.5832176299127241, 1.0637108011351302},
  {0.5931114288882782, 1.086672227258532},
  {0.6076438998079539, 1.1070091416240757},
  {0.6250022985041142, 1.1250023059546947},
  {0.6750022992491722, 1.1750023066997528}
};


// 초기 위치 및 목표 인덱스 변수
float currentX = 0.0, currentZ = 0.0;
int waypointIndex = 0;

void setup() {
    Serial.begin(9600);

    pinMode(DIR1, OUTPUT);
    pinMode(PWM1, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(PWM2, OUTPUT);

    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);
    pinMode(encoderPinC, INPUT);
    pinMode(encoderPinD, INPUT);

    attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinC), handleEncoder2, CHANGE);

    Serial.println("경로 추적 시작");
}

void loop() {
    if (Serial.available() > 0) {
        // 시리얼 데이터 수신
        String data = Serial.readStringUntil('\n');
        int commaIndex = data.indexOf(',');

        // 현재 위치 갱신
        currentX = data.substring(0, commaIndex).toFloat();
        currentZ = data.substring(commaIndex + 1).toFloat();

        // 현재 목표 좌표
        float targetX = waypoints[waypointIndex][0];
        float targetZ = waypoints[waypointIndex][1];
        float distance = calculateDistance(currentX, currentZ, targetX, targetZ);
        float angle = calculateAngle(currentX, currentZ, targetX, targetZ);

        // 현재 위치 및 목표 위치, 거리, 각도 출력
        Serial.print("현재 위치: (");
        Serial.print(currentX);
        Serial.print(", ");
        Serial.print(currentZ);
        Serial.println(")");
        
        Serial.print("목표 위치: (");
        Serial.print(targetX);
        Serial.print(", ");
        Serial.print(targetZ);
        Serial.println(")");

        Serial.print("거리: ");
        Serial.println(distance);
        
        Serial.print("각도: ");
        Serial.println(angle);

        // 모터 제어
        moveToTarget(distance, angle);

        // 목표 지점에 도달 확인 (거리 기준)
        if (distance < 0.05) {  // 임계값 (0.05) 조정 가능
            Serial.println("목표 지점에 도달");
            waypointIndex++;  // 다음 좌표로 이동

            if (waypointIndex >= numWaypoints) {  // 모든 경로 완료 시
                stopMotors();
                Serial.println("경로 완료");
            }
        }
        Serial.println("------------------");
    }
}

// 두 점 사이 거리 계산 함수
float calculateDistance(float x1, float z1, float x2, float z2) {
    return sqrt(sq(x2 - x1) + sq(z2 - z1));
}

// 두 점 사이 각도 계산 함수
float calculateAngle(float x1, float z1, float x2, float z2) {
    return atan2(z2 - z1, x2 - x1);
}

// 목표 방향으로 모터 이동
void moveToTarget(float distance, float angle) {
    int baseSpeed = 70;
    int speedLeft, speedRight;

    if (angle > 0.1) {  // 오른쪽으로 회전
        speedLeft = baseSpeed;
        speedRight = baseSpeed - (angle * 20);  // 각도에 따라 속도 차이 조정
    } else if (angle < -0.1) {  // 왼쪽으로 회전
        speedLeft = baseSpeed + (angle * 20);
        speedRight = baseSpeed;
    } else {  // 직진
        speedLeft = baseSpeed;
        speedRight = baseSpeed;
    }

    analogWrite(PWM1, speedLeft);
    analogWrite(PWM2, speedRight);
    digitalWrite(DIR1, speedLeft >= 0 ? HIGH : LOW);
    digitalWrite(DIR2, speedRight >= 0 ? HIGH : LOW);
}

// 첫 번째 모터의 엔코더 인터럽트 핸들러
void handleEncoder1() {
    int stateA = digitalRead(encoderPinA);
    int stateB = digitalRead(encoderPinB);

    if (stateA != stateB) {
        encoderCount1++;  // A와 B가 다르면 전진
    } else {
        encoderCount1--;  // A와 B가 같으면 후진
    }
}

// 두 번째 모터의 엔코더 인터럽트 핸들러
void handleEncoder2() {
    int stateC = digitalRead(encoderPinC);
    int stateD = digitalRead(encoderPinD);

    if (stateC != stateD) {
        encoderCount2++;  // C와 D가 다르면 전진
    } else {
        encoderCount2--;  // C와 D가 같으면 후진
    }
}

// 모든 모터 정지
void stopMotors() {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
}
