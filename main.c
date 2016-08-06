#include "NU32.h"          // config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "utilities.h"
#include "isense.h"
#include "curcontrol.h"
#include "poscontrol.h"
// include other header files here

// define global parameters
#define BUF_SIZE 200
#define PLOTPTS 99
#define NUMSAMPS 99
#define DECIMATION 1

static volatile int duty_cycle = 0;
static volatile int StoringData = 0;
static volatile float eintcur = 0, eintpos = 0, eprevpos = 0;
static volatile control_data_t Controldata[PLOTPTS];
static volatile float kpcur = 0.3, kicur = 0.04; // Tuned kp/ki values
static volatile float kppos = 160, kipos = 0, kdpos = 5000.0; // Tuned kp, ki & kd values
static volatile int angle = 0;
static volatile float posrefcur = 0; // ref current from position control
static volatile int numsam = 0; // number of samples of setp & cubic trajectories

int picur(unsigned int index, float current); // 'ITEST' mode current controller 'pi' function
int picur2(float ref, float current); // 'HOLD/TRACK' mode current controller 'pi' function
int pidpos(int ref, float act); // 'HOLD/TRACK' mode position control 'pid' function

// Timer2 ISR: freq = 5kHz; Current controller
void __ISR(_TIMER_2_VECTOR, IPL4SOFT) CurController() {
  static int decctr = 0, refcur = 0, plotind = 0, counter = 0;
  static int duty_cycle_new; 
  static float actcur = 0;
  switch (get_mode())
  {
    case IDLE:
    {
      curcontrol_set_pwm(0);
      break;
    }
    case PWM:
    {
      curcontrol_set_pwm(duty_cycle);
      break;
    }  
    case ITEST:
    {
      actcur = isense_amps();
      actcur = isense_amps();
      actcur = actcur/1000;

      refcur = 200;

      if (counter >= 25) {
        refcur = -200;
      } 
      if (counter >= 50) {
        refcur = 200;
      } 
      if (counter >= 75) {
        refcur = -200;
      } 
      duty_cycle_new = picur(counter, actcur);
      curcontrol_set_pwm(duty_cycle_new);
      if (StoringData) {
        decctr++;
        if (decctr == DECIMATION) {
          decctr = 0;          
          Controldata[plotind].refcur = refcur;
          Controldata[plotind].actcur = actcur;
          plotind++;
        }
        if (plotind == PLOTPTS) {
          plotind = 0;
          StoringData = 0;
        }
      }
      counter++;
      if (counter == NUMSAMPS) {
        counter = 0;
        set_mode(IDLE);
      }
      break;
    } 
    case HOLD:
    {
      actcur = isense_amps();
      actcur = isense_amps();
      actcur = actcur/1000;
      duty_cycle_new = picur2(posrefcur, actcur);
      curcontrol_set_pwm(duty_cycle_new);
      break;
    }   
    case TRACK:
    {
      actcur = isense_amps();
      actcur = isense_amps();
      actcur = actcur/1000;
      duty_cycle_new = picur2(posrefcur, actcur);
      curcontrol_set_pwm(duty_cycle_new);
      break;
    }
  }
  IFS0bits.T2IF = 0;
}

// Timer4 ISR: freq = 200 Hz; Position controller
void __ISR(_TIMER_4_VECTOR, IPL5SOFT) PosController() {
  static float actpos = 0;
  static int refpos = 0, decctr = 0, counter = 0, plotind = 0;
  switch (get_mode())
  {
    case HOLD:
    {
      refpos = angle;
      actpos = encoder_deg() / 10;
      posrefcur = pidpos(refpos, actpos);
      break;
    }
    case TRACK:
    {
      refpos = Controldata[counter].refpos;
      actpos = encoder_deg() / 10;
      posrefcur = pidpos(refpos, actpos);
      if (StoringData) {
        decctr++;
        if (decctr == DECIMATION) {
          decctr = 0;
          Controldata[plotind].refpos = refpos;
          Controldata[plotind].actpos = encoder_deg();
          plotind++;
        }
        if (plotind == numsam) {
          plotind = 0;
          StoringData = 0;
        }
      }
      counter++;
      if (counter == numsam) {
        counter = 0;
        set_mode(HOLD);
      }
      break;
    }
  }
  IFS0bits.T4IF = 0;
}

int main() 
{
  char buffer[BUF_SIZE];
  char garbage[100];
  
  NU32_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  
  NU32_LED1 = 1;  // turn off the LEDs
  NU32_LED2 = 1;        
  __builtin_disable_interrupts();

  // in future, initialize modules or peripherals here
  encoder_init(); 
  isense_init();
  curcontrol_init();
  poscontrol_init();

  __builtin_enable_interrupts();

  while(1)
  {
    unsigned int i = 0;
    unsigned int n1 = 0, n2 = 0;
    unsigned int kpcurtemp, kicurtemp;
    unsigned int kppostemp, kipostemp, kdpostemp;
    signed int angletemp, trjtemp;
    unsigned int mode_val;
    unsigned int numsamtemp;
    NU32_ReadUART3(buffer,BUF_SIZE); // we expect the next character to be a menu command
    NU32_LED2 = 1;                   // clear the error LED
    switch (buffer[0]) {
      // Read current sensor (ADC counts)
      case 'a':
      {
        sprintf(garbage, "%d\r\n", isense_counts());
        sprintf(buffer, "%d\r\n", isense_counts());
        NU32_WriteUART3(buffer);
        break;
      }
      // Read current sensor (mA)
      case 'b':
      {
        sprintf(garbage, "%d\r\n", isense_amps());
        sprintf(buffer, "%d\r\n", isense_amps());
        NU32_WriteUART3(buffer);
        break;
      }
      // Read encoder (counts)
      case 'c':
      {
        sprintf(buffer,"%d\r\n", encoder_counts());
        NU32_WriteUART3(buffer);
        break;
      }
      // Read encoder (deg)
      case 'd':                      
      {
        sprintf(buffer,"%d\r\n", encoder_deg());
        NU32_WriteUART3(buffer);
        break;
      }
      // Reset encoder
      case 'e':
      {
        encoder_reset();
        sprintf(buffer, "%d\r\n", encoder_counts());
        NU32_WriteUART3(buffer);
        break;
      }
      // Set PWM
      case 'f':
      {
        
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%d", &duty_cycle);
        set_mode(PWM);
        break;
      }
      // Set current gains
      case 'g':
      {
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%d %d", &kpcurtemp, &kicurtemp);
        __builtin_disable_interrupts();
        kpcur = ((float)kpcurtemp) / 1000;
        kicur = ((float)kicurtemp) / 1000;
        eintcur = 0;
        __builtin_enable_interrupts();
        break;
      }
      // Get current gains
      case 'h':
      {
        kpcurtemp = (int)(kpcur * 1000);
        kicurtemp = (int)(kicur * 1000);
        sprintf(buffer, "%d %d\r\n", kpcurtemp, kicurtemp); 
        NU32_WriteUART3(buffer);        
        break;
      }
      // Set position gains 
      case 'i':
      {
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%d %d %d", &kppostemp, &kipostemp, &kdpostemp);
        __builtin_disable_interrupts();
        kppos = ((float)kppostemp) / 1000;
        kipos = ((float)kipostemp) / 1000;
        kdpos = ((float)kdpostemp) / 1000;
        eintpos = 0;
        eprevpos = 0;
        __builtin_enable_interrupts();
        break;
      }
      // Get position gains
      case 'j':
      {
        kppostemp = (int)(kppos * 1000);
        kipostemp = (int)(kipos * 1000);
        kdpostemp = (int)(kdpos * 1000);
        sprintf(buffer, "%d %d %d\r\n", kppostemp, kipostemp, kdpostemp); 
        NU32_WriteUART3(buffer);
        break;
      }
      // Test current gains
      case 'k':
      { 
        set_mode(ITEST);
        
        StoringData = 1;        
        while (StoringData) { ; }
        sprintf(buffer, "%d\r\n", PLOTPTS);
        NU32_WriteUART3(buffer);

        for (i = 0; i < PLOTPTS; i++)
        {
          sprintf(buffer, "%d %d\r\n", Controldata[i].refcur, Controldata[i].actcur);
          NU32_WriteUART3(buffer);
        }
        break;
      }
      // Go to angle (deg)
      case 'l':
      {
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%d", &angletemp);

        __builtin_disable_interrupts();
        angle = angletemp;
        __builtin_enable_interrupts();

        curcontrol_set_pwm(0);
        set_mode(HOLD);
        curcontrol_set_pwm(0);
        break;   
      }
      // Load step trajectory
      case 'm':
      {
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%d", &numsamtemp);

        __builtin_disable_interrupts();
        numsam = numsamtemp; 
        __builtin_enable_interrupts();

        for (i = 0; i < numsam; i++)
        {
          NU32_ReadUART3(buffer, BUF_SIZE);
          sscanf(buffer, "%d", &trjtemp);
          __builtin_disable_interrupts();          
          Controldata[i].refpos = trjtemp;
          __builtin_enable_interrupts();
        }
        break;
        // curcontrol_set_pwm(0);
      }
      // Load cubic trajectory
      case 'n':
      {
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%d", &numsamtemp);

        __builtin_disable_interrupts();
        numsam = numsamtemp; 
        __builtin_enable_interrupts();

        for (i = 0; i < numsam; i++)
        {
          NU32_ReadUART3(buffer, BUF_SIZE);
          sscanf(buffer, "%d", &trjtemp);
          __builtin_disable_interrupts();         
          Controldata[i].refpos = trjtemp;
          __builtin_enable_interrupts();
        }
        break;
        // curcontrol_set_pwm(0);
      }
      // Execute trajectory
      case 'o':
      {
        curcontrol_set_pwm(0);
        set_mode(TRACK);
        StoringData = 1;        
        while (StoringData) { ; }
        sprintf(buffer, "%d\r\n", numsam);
        NU32_WriteUART3(buffer);
        for (i = 0; i < numsam; i++)
        {  
          if (Controldata[i].refpos < -6582) {
            Controldata[i].refpos = Controldata[i-1].refpos;
          }        
          sprintf(buffer, "%d %d\r\n", Controldata[i].refpos, Controldata[i].actpos / 10);
          NU32_WriteUART3(buffer);
        }
        break;
      }
      // Unpower the motor
      case 'p':
      {        
        set_mode(IDLE);
        break;
      }
      // Get mode
      case 'r':
      {
        switch (get_mode())
        {
          case IDLE: 
          {
            mode_val = 0; break; 
          }
          case PWM:
          {
            mode_val = 1; break;
          }
          case ITEST:
          {
            mode_val = 2; break;
          }
          case HOLD:
          {
            mode_val = 3; break;
          }
          case TRACK:
          {
            mode_val = 4; break;
          }          
        }
        sprintf(buffer, "%d\r\n", mode_val);
        NU32_WriteUART3(buffer);
        break;
      }
      // Quit client
      case 'q':
      {
        set_mode(IDLE);
        break;
      }
      case 'x':
      {        
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%d %d", &n1, &n2);
        sprintf(buffer,"%d\r\n", n1 + n2);
        NU32_WriteUART3(buffer);
        break;
      }      

      default:
      {
        NU32_LED2 = 0;  // turn on LED2 to indicate an error
        break;
      }
    }
  }
  return 0;

}

int picur(unsigned int index, float current) {
  float ecur, refcur, actcur;
  int temp;
  int u;
  refcur = Controldata[index].refcur;
  actcur = current;  
  ecur = refcur - actcur;
  eintcur += ecur;
  temp = kpcur*ecur + kicur*eintcur;
  if (temp < -100)
    u = -100;
  else if (temp > 100)
    u = 100;
  else u = temp; 
  return u;
}

int picur2(float ref, float current) {
  float ecur, refcur, actcur;
  int temp;
  int u;
  refcur = ref;
  actcur = current;  
  ecur = refcur - actcur;
  eintcur += ecur;
  temp = kpcur*ecur + kicur*eintcur;
  if (temp < -100)
    u = -100;
  else if (temp > 100)
    u = 100;
  else u = temp; 
  return u;
}

int pidpos(int ref, float act) {
  float epos, edotpos, refpos, actpos;
  int temp;
  int u;
  refpos = ref;
  actpos = act;  
  epos = refpos - actpos;
  edotpos = epos - eprevpos;
  eintpos += epos;
  temp = kppos*epos + kipos*eintpos + kdpos*edotpos;
  if (temp < -350)
    u = -350;
  else if (temp > 350)
    u = 350;
  else u = temp; 
  eprevpos = epos;
  return u;
}