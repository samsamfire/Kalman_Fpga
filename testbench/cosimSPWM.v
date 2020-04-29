`include "D:/Programs/Quartus/quartus/eda/sim_lib/220model.v"
`include "../src/fixedpoint_multiplier.v"
`include "../src/mult.v"
`include "../src/spwm.v"


module cosimSPWM
(	output Sau, 
	output Sal,
	output Sbu, 
	output Sbl, 
	output Scu, 
	output Scl, 
	output signed [31:0] out,
    input signed [31:0] x, 
    input signed [31:0] y, 
    input signed [31:0] z, 
    input clk,res);


spwm #(32,28) uut(
	.x(x),
	.y(y),
	.z(z),
	.clk(clk),.res(res),
	.Sau(Sau),.Sal(Sal),
	.Sbu(Sbu),.Sbl(Sbl),
	.Scu(Scu),.Scl(Scl),
	.out(out));





endmodule

