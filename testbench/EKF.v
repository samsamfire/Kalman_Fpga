// Code your testbench here
// or browse Examples
module EKF(
	input signed [31:0] valpha,vbeta,ialpham,ibetam,ctheta,stheta,
	input [31:0] nbSamples,
	input clk,reset,
	output signed [31:0] omega,theta,
	output signed [31:0] ialphak,ibetak,kmatrix00,kmatrix01,kmatrix10,kmatrix11,kmatrix20,kmatrix21,kmatrix30,kmatrix31
	);
	
	
  parameter sf = 2**18;
  parameter N=32;
  parameter Q=18;

  
  


  
  
  
  
  
  
  //EKF EKF0(.valpha(10*sf),.vbeta(15*sf),.ialpham(5*sf),.ibetam(7*sf),.ctheta(0.5*sf),.stheta(0.3*sf),.clk(clk),.reset(reset),.omega(omega),.theta(theta));
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
  
  
  //always #100 clk=~clk;
  

  
  
  
  
endmodule