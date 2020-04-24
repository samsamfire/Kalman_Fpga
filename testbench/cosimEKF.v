`include "D:/Programs/Quartus/quartus/eda/sim_lib/220model.v"
`include "../src/matrix_multiplication.v"
`include "../src/fixedpoint_multiplier.v"
`include "../src/cordic.v"
`include "../src/div2.v"
`include "../src/kalman.v"
`include "../src/state_update.v"
`include "../src/PID.v"


//By Samuel Lee
//FPGA synthesizable code for fixed point 32-bit Kalman filter

//NbSamples is an additional delay in the main loop for testing. The alorithm takes 79 clock cycles, nbSamples represent "do nothing" cycles in between each update.
//The sampling time must be set accordingly
module cosimEKF(
	input signed [31:0] valpha,vbeta,ialpham,ibetam,
	input [31:0] nbSamples,
	input clk,reset,
	output signed [31:0] omega,theta,
	output signed [31:0] ialphak,ibetak,kmatrix00,kmatrix01,kmatrix10,kmatrix11,kmatrix20,kmatrix21,kmatrix30,kmatrix31
	);
	
	
  parameter sf = 2**18;
  parameter N=32;
  parameter Q=18;

  
  


  
  
 
  kalman kalman0(.valpha(valpha),.vbeta(vbeta),.ialpham(ialpham),.ibetam(ibetam),.ctheta_t(ctheta),.stheta_t(stheta),.clk(clk),.reset(reset),.omega(omega),.theta(theta),.nbSamples(nbSamples),
  	.ialphak(ialphak),
  	.ibetak(ibetak),
  	.kmatrix00(kmatrix00),
  	.kmatrix01(kmatrix01),
  	.kmatrix10(kmatrix10),
  	.kmatrix11(kmatrix11),
  	.kmatrix20(kmatrix20),
  	.kmatrix21(kmatrix21),
  	.kmatrix30(kmatrix30),
  	.kmatrix31(kmatrix31)

  	);
  
  
  
endmodule