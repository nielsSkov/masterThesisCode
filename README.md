# Code for Cart Pendulum and Twin Pendulum
  
Before uploading FIRST time:  
- Remember to add the library Joint_teensy.h  
  > Sketch - Import Library - Add Library  
  
In general before upload remember:  
- Check:  
  > Tools - Board - Teensy 3.6 (Programming Port)  
  > Tools - Port - COM# (Teensy 3.6 (Programming Port))  
  > "Serial.begin(115200)" below agree with baud on terminal  
- Move the cart to the right side of the rail  
  
OBS: The cart must be at the right side of the rail @ each upload!  
  
-------SERIAL INTERFACE----------------------------------------  
  
r - \*Reset Pendulum Refference  
f - Friction Compensation Only  
  
0 - Stop all  
  
1 - Swing-Up and Sliding Mode  
2 - \*\*Twin Swing-Up and Catch with LQR  
3 - \*\*Twin Swing-Up and Catch with Sliding Mode  
  
5 - Cart Mass and Friction Estimation  
6 - Pendulum (1 & 2) Friction Estimation  

\*only the pendulums are reset for input 'r', however,  
  it is possible to enable cart position reset with 'r'  
  by uncommenting // cart.resetPos();  
  currently the cart is only reset at each new upload,  
  ( this allows resetting the pendulums without having to  
    move the cart to zero position on the rail           )  
  
\*\*the swing-up works and the stabilizing controllers work,  
    however, on the system in its current state, they do NOT  
    swing-up AND catch the twin pendulum  
  
---------------------------------------------------------------  
