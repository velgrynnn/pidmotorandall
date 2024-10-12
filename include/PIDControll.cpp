
#include "PIDControll.h"


PIDController::PIDController()
{
    //INI MUNGKIN GUNA
    prev_err_[0] = 0.;
    prev_err_[1] = 0.;
    prev_out_[0] = 0.;
    prev_out_[1] = 0.;
    double prev_ril_err[3] = {0.,0.,0.};
}

//FC= filter coeff, cp tuh 
void PIDController::init(int mode, float kp, float ti, float td, float ff, float fc, float cp, bool is_active, int motornum)
{
    //INI GANTI JADI KP KI KD

    motor_num = motornum;
    Kp = kp;
    Kd = td;
    Ki = ti;
    errorIntegral =0; 
    errorDerivative  = 0;
    err = 0;
    out = 0;
    //samplingTime = 5;//isi sama sampling time, seberapa lama compute actio dipanggil 
    prev_ril_err[0] = 0.;
    prev_ril_err[1] = 0.;
    prev_ril_err[2] = 0.;    
    //CARI TAU DEFINE MODE DIMANA, APAKAH DIKIRIM DARI COMPUTER, MODE 1 PI, MODE 2 PID 
    //BUAT BEBERAPA MODE PI, PID, INI KAYANYA BUAT NGITUNGIN KP KI KD LANGSUNG DECLARE AJA 
    

    // Koefisien Calman Filter
    v1Filt = 0;
    v1Prev = 0;
    R = 100;
    Q = 1;
    Pt_prev = 1;

    //INI PI 
    if (mode == 1) {
        Kp = Kp;
        Ki = Ki; 
  
        is_active_ = is_active;
    }


    // INI MAU PID 
    if(mode==2){
        
        Kp = Kp;
        Ki = Ki;
        Kd = Kd;
        is_active_ = is_active; 
    }

        prev_err_[0] = 0.;
        prev_err_[1] = 0.;
        prev_out_[0] = 0.;
        prev_out_[1] = 0.;
        is_active_ = is_active;
    }



//UNTUK CLAMPING, GA DIUBAH
double PIDController::clamp(double val, double min, double max)
{
    if (val >= max)
    {
        return max;
    }
    else if (val <= min)
    {
        return min;
    }
    else
    {
        return val;
    }
}

int PIDController::sign(double val)
{
    if (val > 0)
    {
        return 1;
    }
    else if (val < 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

double PIDController::compute_action(double target, double feedback, float ff, float samplingTime)
{
    //feedback = filter_Kalman(feedback);
    double ril_err = target - feedback; //ITUNG ERROR

    //MOVING AVERAGE 
    //err = (prev_ril_err[1] + prev_ril_err[0] + ril_err) / 3;

    err = ril_err;
    targett = target;
    
    errorIntegral = errorIntegral + err * samplingTime;
    errorDerivative = (prev_err_[0] - err)/samplingTime;
    
    errorDerivative = clamp(errorDerivative, -5,5);
    errorIntegral = clamp(errorIntegral, -500,500);
    

    //NGITUNG KELUARAN, TAPI PAKE KONTINIU
    if(is_active_)
    {
        // Basic PID
        out += err * Kp + errorIntegral * Ki + errorDerivative * Kd ;
        //CLAMP OUTPUT 
        out = clamp(out, -1., 1.);
    }

    //ITUNG PREVIOUS ERROR 
    prev_ril_err[1] = prev_ril_err[0];
    prev_ril_err[0] = ril_err;
    
    prev_err_[1] = prev_err_[0];
    prev_err_[0] = err;
    prev_out_[1] = prev_out_[0];
    prev_out_[0] = out;

    //INI KELUARAN RETURNNYA, HARUSNYA NGGA DIUBAH 
    if(is_active_) {
        if (target == 0) {
            //out = 0.0;
            return 0.0;
        }
        else return clamp(out + ff * sign(target) * feed_forward_abs(target), -1., 1.); //FF TUH FEEDFORWARD, -1 SAMA 1 TUH CLAMPING MIN MAX
        }
    else { 
        return 0.0;
    }
}


// INI BIARIN AJA 
void PIDController::setActive(bool command_active) {
    is_active_ = command_active;
}

double PIDController::filter_Kalman(double v1){

    //WITH LOWPASS FILTER 
    // v1Filt = 0.854*v1Filt + 0.0728*(v1) + 0.0728*v1Prev;
    // v1Prev = v1;
    
    //WITHOUT LOWPASS FILTER 
    v1Filt = v1;

    // kalman
    Xt_update = Xt_prev;
    Pt_update = Pt_prev + Q;
    Kt = Pt_update / (Pt_update + R);
    Xt = Xt_update + (Kt * (v1Filt - Xt_update));
    Pt = (1 - Kt) * Pt_update;
    Xt_prev = Xt;
    Pt_prev = Pt;
    kalmanFilterData = Xt;
    v1 = Xt;

    return v1;
 }

double PIDController::feed_forward_abs(double target){

    // Feed forward implementation using vel-pwm data.
    const int N = 7;
    const float matrix[4][7][2] = {{{0.22 , 0.3},
                            {0.46336, 0.35},
                            {0.675761, 0.4},
                            {0.8996, 0.45},
                            {1.1090, 0.5},
                            {1.2269, 0.55},
                            {1.3393, 0.6}
                            },{{0.322 , 0.3},
                            {0.681833 , 0.35},
                            {1.008122, 0.4},
                            {1.23872, 0.45},
                            {1.56777, 0.5},
                            {1.81210, 0.55},
                            {1.95465, 0.6}
                            },{{0.318 , 0.3},
                            {0.63634, 0.35},
                            {0.93415, 0.4},
                            {1.303405, 0.45},
                            {1.67947, 0.5},
                            {1.907619, 0.55},
                            {2.082, 0.6}
                            },{{0.255 , 0.3},
                            {0.7708002, 0.35},
                            {1.13862, 0.4},
                            {1.457401, 0.45},
                            {1.77808, 0.5},
                            {1.97890, 0.55},
                            {2.12811, 0.6}
                            }};
    float x1,x2,y1,y2;
    double out;
    float x = std::abs(target);
    if (x < matrix[motor_num-1][0][0]){
        return 0;
    }
    for (int i = 0; i < N-1; i++){
        x1 = matrix[motor_num-1][i][0];
        y1 = matrix[motor_num-1][i][1];

        x2 = matrix[motor_num-1][i+1][0];
        y2 = matrix[motor_num-1][i+1][1];
        if (x >= x1 && x < x2){
            out = (y2 - y1)/(x2 - x1) * (target - x1) + y1; // Interpolation y - y1 / y2-y1 = x - x1 / y2 - y1
            return out;
        }
    }
    return matrix[motor_num-1][N-1][1];
}


