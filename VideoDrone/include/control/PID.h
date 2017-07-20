/* FileName: PID.h
Created by Rakshit Allamraju
Date 30 June 2017

Contains class definitions for various controller to be used 
in this project

*/

class PID{
    private:
        PID();
        ~PID();

        void calcError();
        void calcIntegralError();
        void calcDerivativeError();
        void calcPID();

        float dT;
        float error, error_prev;
        float errorIntegral;
        float errorDerivative;
        float Kp, Ki, Kd;
        float Yd, Ya;
        float output;

    public:
	void setDt(float dt);
        void setKp(float kp);
        void setKi(float ki);
        void setKd(float kd);
        void setDesired(float yd);
        void setActual(float ya);
        float getoutput();

};
