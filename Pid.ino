// turntable positioning using pid controller

#define VERSION "Pid 200125a"

#define ABS(x)      (x < 0 ? -x : x)
#define SGN(x)      (x < 0 ? -1 : 1)
#define MIN(a, b)   (a < b ? a : b)

// -----------------------------------------------------------------------------
// encoder pinout
//    pin 1     output A    (pulls 1.6K resistor to V+)
//    pin 2     output B
//    pin 3     V+
//    pin 4     gnd
//    pin 5     motor
//    pin 6     motor

#define PIN_E_A     2
#define PIN_E_B     3

#define PIN_H_EN    4
#define PIN_H_A     5
#define PIN_H_B     6

#define PIN_BUT_L   11
#define PIN_BUT_R   12

#define Hb          13

int  debug = 0;

enum {
    M_NONE,
    M_CTL,
    M_MAN,
    M_LAST
};

int  mode  = M_NONE;

enum {
    CAP_NONE,
    CAP_ISR,
    CAP_PID,
};

static   int  capture  = CAP_PID;

// -----------------------------------------------------------------------------
void
myPrint (
    const char* text,
    int         val )
{
    Serial.print (text);
    Serial.print (val);
}

void
myPrintln (
    const char* text,
    int         val )
{
    myPrint (text, val);
    Serial.print ("\n");
}

// -----------------------------------------------------------------------------
#ifdef ARDUINO_AVR_UNO
# define BUF_SIZE      100
#else
# define BUF_SIZE     2500
#endif

int   bufWrap = 0;
int   bufIdx  = 0;
unsigned int   buf [BUF_SIZE];

void
dumpPr (
    int     n,
    int     i )
{
    if (! (i % 16))  {
        Serial.print ("\n  ");
        Serial.print (n);
    }
    Serial.print ("  ");
    Serial.print (buf [n], HEX);
}

// ---------------------------
void
dumpBuf (void)
{
    Serial.print   ("dumpBuf: BUF_SIZE ");
    Serial.print   (BUF_SIZE);
    Serial.print   (", bufIdx ");
    Serial.println (bufIdx);

    int  i = 0;
    if (0 != buf [bufIdx])  {
        for (int n = bufIdx; n < BUF_SIZE; n++)
            dumpPr (n, i++);
    }

    i = 0;
    for (int n = 0; n < bufIdx; n++)
        dumpPr (n, i++);
    Serial.println ("");
}

// -----------------------------------------------
void
clearBuf (void)
{
    memset (buf, 0, sizeof(buf));
}

// -----------------------------------------------
// connections to H-bridge
//                     ___________
//                    |  754410   |
//       NC --- 1,2EN | 1       16| Vcc1  ---- Brd 5V
//       NC ---    1A | 2       15| 4A    ---- Dig-11
//       NC ---    1Y | 3       14| 4Y    ---- Motor +
//  12V Gnd ---   Gnd | 4       13| Gnd
//  Brd Gnd ---   Gnd | 5       12| Gnd  ????????????
//       NC ---    2Y | 6       11| 3Y    ---- Motor -
//       NC ---    2A | 7       10| 3A    ---- Dig-10
//     12V ----  Vcc2 | 8        9| 3,4EN ---- Dig-7
//                    |___________|
// 
// 3/4 A inputs are complementary,
// tie corresponding Y pin V+/Gnd,
// controlling direction
// EN is PWM 

typedef enum {
    D_CW,
    D_CCW,
    D_SHORT,
} Dir_t;

const char* dirStr [] = { "D_CW", "D_CCW" };

Dir_t          hDir   = D_CW;
unsigned int   hPwm   = 0;

#define H_MAX   255

#define Reverse()     setDir(D_CW == hDir ? D_CCW : D_CW)
#define Dir(x)        ((x) > 0 ? D_CW : D_CCW)

void status (void);

enum {
    C_RUN = 1,
    C_REV,
    C_MAX,
    C_SLOW,
    C_COAST,
    C_STOP,
    C_LAST
};

const char *  condStr [] = {
    "",
    "C_RUN",
    "C_REV",
    "C_MAX",
    "C_SLOW",
    "C_COAST",
    "C_STOP",
};

static int   cond  = C_STOP;
static int   cond1 = C_STOP;

// ------------------------------------------------
void
setDir (
    Dir_t     d )
{
    hDir = d;
    
    if (debug)  {
        Serial.print (__func__);
        Serial.print (": ");
        Serial.print (d);
        Serial.print (" ");
        Serial.println (D_CW == hDir ? "D_CW" : "D_CCW");
    }

    if (hDir == D_CCW)  {     // depends on polarity
        digitalWrite (PIN_H_A, LOW);
        digitalWrite (PIN_H_B, HIGH);
    }
    else if (hDir == D_CW)  {
        digitalWrite (PIN_H_A, HIGH);
        digitalWrite (PIN_H_B, LOW);
    }
    else  {
        digitalWrite (PIN_H_A, LOW);
        digitalWrite (PIN_H_B, LOW);
    }
}

// ------------------------------------------------
#define SetSpeed(x)   setSpeed(x)
void
setSpeed (
    int   spd )
{
    if (debug)  {
        Serial.print   ("setSpeed: ");
        Serial.println (spd);
    }

    if (hDir != Dir(spd))
        setDir(Dir(spd));

    hPwm = ABS(spd);
    analogWrite(PIN_H_EN, hPwm);

    if (0 == hPwm)
        cond = C_STOP;
    else
        cond = C_RUN;
}

// -----------------------------------------------------------------------------
//    __      ____      _
//  A   |____|    |____|
//         ____      ____
//  B ____|    |____|
//        ^

static unsigned long  isrCnt  = 0;

static unsigned long  maxPer = 0;
static unsigned long  minPer = 0;

static          long  target = 0;

static int    cnt    = 0;
static int    cntDec = 0;

static byte enc;
static byte encLast = 0;

#define BIT_E_A     1
#define BIT_E_B     1<<1

// -------------------------------------
// just rising edge of B and check A
static unsigned long isrUsec   = 0;
static unsigned long isrDtUsec = 0;
static          long isrPos    = 0;
static          long isrPos1   = 0;
static int8_t        isrDir;

void
isr (void)
{
    byte encA   = (digitalRead(PIN_E_A));
    byte encB   = (digitalRead(PIN_E_B));

    if (0 == encB)
        return;

    unsigned long usec = micros ();
    isrDtUsec   = usec - isrUsec;
    isrUsec     = usec;

    isrPos1 = isrPos;

    if (encA)  {
        isrPos++;
        isrDir =  1;
    }
    else  {
        isrPos--;
        isrDir = -1;
    }

    enc = encB << 1 | encA;

    // -----------------------------------------------
    if (4 <= debug)  {
        Serial.print ("# isr: enc ");
        Serial.print (enc);
        Serial.print (", encLast ");
        Serial.print (encLast);
        Serial.print (", pos ");
        Serial.println (isrPos);
    }
    cond1 = cond;

    // -----------------------------------------------
    if (CAP_ISR == capture) {
     // buf [bufIdx++] = micros() & 0xFFFF;
        buf [bufIdx++] = usec & 0xFFFF;
        buf [bufIdx++] = isrPos;
        buf [bufIdx++] = hPwm;
        buf [bufIdx++] = enc << 4 | hDir;

        bufIdx = BUF_SIZE <= bufIdx ? 0 : bufIdx;
    }
}

// --------------------------------------------------------------------
typedef struct  {
    double      kp;
    double      kd;
    uint8_t     minPwm;
    uint8_t     maxPwm;
    uint16_t    minDt;
} Params_t;

Params_t params = { 2.0, 0.11, 70, 180, 0 };

// ------------------------------------------------
void
dispParams (void)
{
    for (int n = 0; n < 2; n++)  {
        Params_t *p = & params;

        Serial.print (" dispParams:");
        myPrint ("  Kd ", 100 * p->kd);
        myPrint (", Kp ", 100 * p->kp);
        myPrint (", Pwm max ", p->maxPwm);
        myPrint (", min ", p->minPwm);
        myPrint (", Dt min ", p->minDt);
        Serial.print ("\n");
    }
}

// ------------------------------------------------
#define MIN_PWM     70

int
pwmVal (
    float   x )
{
    int   val;

    if (1 >= ABS(x))
        return 0;

    Params_t  *p = & params;

    val  = p->minPwm + ABS(x);
 // val  = max < val ? max : val;
    val  = MIN(val, p->maxPwm);
    val *= SGN(x);

    return val;
}

// ------------------------------------------------
#define NVANE     30
static long dist;

float
pid (
    long        targ,
    long        pos,
    long        dtUsec,
    int         dir,
    float       kp,
    float       kd,
    int        *pDist,
    int        *pSpd )
{
    static float  spdAvg  = 0;
           float  res     = 0;

    if (0 != dtUsec)  {
#define SPD_COEF  1000000
        float spd = SPD_COEF * dir / dtUsec;

        if (0 == spdAvg)
            spdAvg  = spd;
        else
            spdAvg += (spd - spdAvg) / 8;
    }

    long dist  = targ - pos;

    if (NULL != pDist && NULL != pSpd)  {
        *pDist = dist;
        *pSpd  = spdAvg;
    }

    res = kp * dist - kd * spdAvg;
 // res = kp * dist;

    if (1000 < ABS(res))
        res = SGN(res) * 1000;

    // -----------------------------------------------
#define INT8(x)   (SGN(x) * 128 > ABS(x) ? ABS(x) : 127)

    if (CAP_PID == capture) {
        buf [bufIdx++] = micros() & 0xFFFF;
#if 1
        buf [bufIdx++] = dist;
        buf [bufIdx++] = spdAvg;
        buf [bufIdx++] = pos;
#elif 1
        buf [bufIdx++] = dtUsec;
        buf [bufIdx++] = spdAvg;
        buf [bufIdx++] = pos;
#else
        buf [bufIdx++] = dtUsec;
        buf [bufIdx++] = spdAvg;
        buf [bufIdx++] = dir;
#endif

        bufIdx = BUF_SIZE <= bufIdx ? 0 : bufIdx;
    }

    return res;
}

// --------------------------------------------------------------------
void
ctlPid (void)
{
    if (M_CTL != mode)  {
        if (1 < debug)  {
            Serial.print   ("ctlPid: mode");
            Serial.println (mode);
        }
        return;
    }

    noInterrupts();
    long   pos   = isrPos;
    int    dir   = isrDir;
    long   dt    = isrDtUsec;
    interrupts();

    dist    = target - pos;

    if (2000 < dt && 10 > ABS(dist))  {
        setSpeed (0);
    //  usecLst = 0;
        mode    = M_NONE;
        Serial.println ("control: stop");
        return;
    }

    Params_t  *p = & params;
    float err  = pid (target, pos, dt,  dir,
                        p->kp, p->kd, NULL, NULL);
    int   pwm  = pwmVal (err);
    setSpeed (pwm);

    if (0 == p->minDt)
        p->minDt = dt;
    else
        p->minDt = MIN(dt, p->minDt);

    if (debug)  {
        char s [40];
        sprintf (s, "ctlPid: err %d, pwm %d", int(err), pwm);
        Serial.println (s);
    }
}

// --------------------------------------------------------------------
void
status (void)
{
    char s [50];

    Serial.print   ("status:");

    Serial.print   ("  target ");
    Serial.print   (target);
    Serial.print   (", pos ");
    Serial.print   (isrPos);

    Serial.print   (", dist ");
    Serial.print   (dist);
    Serial.print   (", dir ");
    Serial.print   (Dir(isrDir) == D_CW ? "CW " : "CCW");
    Serial.print   ("\n");

    Serial.print   ("       ");
    Serial.print   ("  PID: Kp ");
    Serial.print   (params.kp);
    Serial.print   ("  Kd ");
    Serial.print   (params.kd);
    Serial.print   ("\n");

    sprintf (s, "%8s ISR: pos %lu, dUsec %lu, dir %d",
                    "", isrPos, isrDtUsec, isrDir);
    Serial.println (s);

    Serial.print   ("       ");
    Serial.print   ("  hPwm ");
    Serial.print   (hPwm);
    Serial.print   (", hDir ");
    Serial.print   (hDir == D_CW ? "CW " : (hDir == D_CCW ? "CCW" : "SHORT"));

    Serial.print   ("  cond ");
    Serial.print   (cond);
    Serial.print   ("  mode ");
    Serial.print   (mode);
    Serial.print   ("  capture ");
    Serial.print   (capture);
    Serial.print   ("\n");

#if 0
    Serial.print   ("\n");
    Serial.print   ("       ");

    Serial.print   ("       ");
    Serial.print   ("  maxPer ");
    Serial.print   (maxPer);
    Serial.print   (", minPer ");
    Serial.print   (minPer);

    Serial.print   (", isrCnt ");
    Serial.println (isrCnt);
#endif

    Serial.print   ("        ");

    Serial.print   (" enc ");
    Serial.print   (enc);
    Serial.print   (", 2(A) ");
    Serial.print   (digitalRead (PIN_E_A));
    Serial.print   (", 3(B) ");
    Serial.print   (digitalRead (PIN_E_B));
    Serial.print   (", 4(EN) ");
    Serial.print   (digitalRead (4));
    Serial.print   (", 5(a) ");
    Serial.print   (digitalRead (5));
    Serial.print   (", 6(b) ");
    Serial.println (digitalRead (6));
}

// -----------------------------------------------------------------------------
// process single character commands from the PC
void
pcRead (void)
{
    static long  val  = 0;
    static int   sign = 1;

    if (Serial.available()) {
        int c = Serial.read ();

        switch (c)  {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            val = c - '0' + (10 * val);
            Serial.println (val);
            return;

        case ' ':
            val  = 0;
            sign = 1;
            return;

        case '-':
            sign = -1;
            return;

        case 'B':
            status();
            dumpBuf();
            break;

        case 'b':
            SetSpeed(0);
            setDir(D_SHORT);
            break;

        case 'C':
            clearBuf();
            break;

        case 'c':
            capture = val;
            val   = 0;
            break;

        case 'D':
            debug = val;
            val   = 0;
            break;

        case 'd':
            status();
            break;

        case 'K':
            params.kd = val / 100.0;
            val = 0;
            break;

        case 'M':
            params.maxPwm = val;
            val = 0;
            break;

        case 'm':
            mode = val;
            val = 0;
            break;

        case 'p':
            dispParams();
            break;

        case 'R':
            SetSpeed(H_MAX);
            break;

        case 'r':
            setDir(D_CW == hDir ? D_CCW : D_CW);
            break;

        case 'S':
            SetSpeed(H_MAX < val * sign ? H_MAX : val * sign);
            setDir(0 < val ? D_CCW : D_CW);
            mode = M_MAN;
            Serial.print   (" pcRead: hPwm ");
            Serial.println (hPwm);
            val  = 0;
            sign = 1;
            break;

        case 's':           // stop
            mode = M_NONE;
            SetSpeed(0);
            isrPos = isrUsec= 0;
            maxPer = minPer = 0;
            status();
            break;

        case 't':
            mode   = M_CTL;
            target = val * sign;
            val    = 0;
            sign   = 1;

            Serial.print   ("pcRead: ");
            Serial.print   (" target ");
            Serial.println (target);

            isrCnt = maxPer = minPer = 0;

       //   setDir   (target < pos ? D_CCW : D_CW);
       //   SetSpeed (200);
            break;

        case 'v':
            Serial.println (VERSION);
            break;

            break;

        case '?':
            Serial.println ("   [0-9] 10*val + digit");
            Serial.println ("   sp    val = 0");
            Serial.println ("   -     sign = -1");
            Serial.println ("   B     dumpBuf()");
            Serial.println ("   b     brake");
            Serial.println ("   C     clearBuf ()");
            Serial.println ("   c     set capture (1 ISR, 2 PID");
            Serial.println ("   D     set debug");
            Serial.println ("   d     status()");
            Serial.println ("   K     set pidKd (x100)");
            Serial.println ("   M     set max pwm");
            Serial.println ("   m     set mode");
            Serial.println ("   P     set pos = spd = 0");
            Serial.println ("   R     run max speed");
            Serial.println ("   r     reverse");
            Serial.println ("   S     set speed/directio (+/- 0-255)");
            Serial.println ("   s     set pos = spd = 0 and status ()");
            Serial.println ("   t     set target pos and enable ctlPid");
            Serial.println ("   v     version");
            break;

        default:
            break;
        }
    }
}

// -----------------------------------------------------------------------------
// start/stop
void
buttons ()
{
    byte val = 3 ^ ((digitalRead(PIN_BUT_L) << 1) | digitalRead(PIN_BUT_R));

    if (debug && 0 < val)  {
        Serial.print ("buttons: ");
        Serial.println (val);
    }

    if (1 == val)  {
        if (debug)
            Serial.println ("buttons: but-1");
        setSpeed (H_MAX);
        setDir (D_CW);
        mode = M_MAN;
        cnt  =   BUF_SIZE/4/2;
        cntDec = 0;
    }
    else if (2 == val)  {
        if (debug)
            Serial.println ("buttons: but-2");
        setSpeed (H_MAX);
        setDir   (D_SHORT);
        cntDec = -1;
    }
}

// -----------------------------------------------------------------------------
// set LED if detector event occured.
void
loop ()
{
    ctlPid ();

    buttons ();
    pcRead ();
}

// -----------------------------------------------------------------------------
void
setup ()
{
    Serial.begin (9600);

#ifdef ARDUINO_AVR_UNO
    Serial.print   ("Arduino AVR UNO - ");
#endif
    Serial.println (VERSION);

    pinMode      (PIN_E_A, INPUT);
    pinMode      (PIN_E_B, INPUT);

    pinMode      (PIN_BUT_L, INPUT_PULLUP);
    pinMode      (PIN_BUT_R, INPUT_PULLUP);

    pinMode      (Hb,      OUTPUT);
    pinMode      (PIN_H_EN,OUTPUT);
    pinMode      (PIN_H_A, OUTPUT);
    pinMode      (PIN_H_B, OUTPUT);

    setDir(D_CW);
    SetSpeed(0);

    clearBuf ();

    attachInterrupt(digitalPinToInterrupt(PIN_E_B), isr, RISING);
}
