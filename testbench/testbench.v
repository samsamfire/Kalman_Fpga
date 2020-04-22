`include "D:/Programs/Quartus/quartus/eda/sim_lib/220model.v"
`include "kalman.v"
`include "div2.v"
`include "sequential_matrix.v"
`include "parallel_matrix.v"
`include "qmult.v"
`include "qadd.v"
`include "cordic.v"

`timescale 1ps / 1ps
module testbench;

reg [31:0] valpha,vbeta,ialpham,ibetam,ctheta,stheta;
wire signed [31:0] omega,theta;
reg clk,reset;

parameter [31:0] sf = 2**18;

initial begin
valpha = 0;
vbeta = 0;
ialpham = 0;
ibetam = 0;
ctheta = sf;
stheta = 0;

	clk = 1'b0;
	reset = 1'b1;
	#10 reset =1'b0;
	#10 reset = 1'b1;
	
end


kalman kalman0(.valpha(valpha),.vbeta(vbeta),.ialpham(ialpham),.ibetam(ibetam),.ctheta(ctheta),.stheta(stheta),.clk(clk),.reset(reset),.omega(omega),.theta(theta));

always #5 clk=~clk;

endmodule