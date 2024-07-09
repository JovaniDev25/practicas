//Rutina PID en Arduino 

double PID(double Kp, double Ki, double Kd, double Ref, double Retro) {
  currentTime = millis();                              
  elapsedTime = (double)(currentTime - previousTime); 

  error = Ref - Retro;                       
  cumError += error * elapsedTime;                
  rateError = (error - lastError) / elapsedTime;  

  double output = Kp * error + Ki * cumError + Kd * rateError;  

  lastError = error;           
  previousTime = currentTime;  

  return output;
}