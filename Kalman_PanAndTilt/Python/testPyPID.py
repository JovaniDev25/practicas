#Rutina PID en Python

def PID(Kp, Ki, Kd, ref, retro):
    
global tiempo, integral, time_prev, error_prev
error = ref - retro

P = Kp*e
I = I + Ki*error*(tiempo- tiempo_prev)
D = Kd*(error - error_prev)/(tiempo - time_prev)

Salida =P + I + D

error_prev = error
tiempo_prev = tiempo

return Salida