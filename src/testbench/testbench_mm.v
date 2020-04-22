`include "D:/Programs/Quartus/quartus/eda/sim_lib/220model.v"
`include "sequential_matrix.v"
`include "parallel_matrix.v"
`include "qmult.v"
`include "qadd.v"


`timescale 1ps / 1ps
module testbench_mm;


parameter N=32;
parameter Q = 18;
parameter signed [N-1:0] sf = 2**Q;

wire signed [4*4*N-1:0] A,B,C;
wire signed [4*4*N-1:0] matrixResult;

reg signed [N-1:0] A00,A01,A02,A03,A10,A11,A12,A13,A20,A21,A22,A23,A30,A31,A32,A33; 
reg signed [N-1:0] B00,B01,B02,B03,B10,B11,B12,B13,B20,B21,B22,B23,B30,B31,B32,B33;
wire signed [N-1:0] C00,C01,C02,C03,C10,C11,C12,C13,C20,C21,C22,C23,C30,C31,C32,C33;

reg clk,reset;


sequential_matrix_multiply smm(A,B,C,clk,reset);

initial begin
	clk =1'b0;
	reset =1'b1;
	#10 reset = 0;
	#10 reset = 1;

	A00 = -18;A01=0;A02 = 0;A03=0;
	A10 = 0;A11=0;A12 = 0;A13=0;
	A20 = 0;A21=0;A22 = 0;A23=0;
	A30 = 0;A31=0;A32 = 0;A33=0;

	B00 = 0;B01=0;B02 = 0;B03=0;
	B10 = 0;B11=0;B12 = 0;B13=0;
	B20 = 0;B21=0;B22 = 0;B23=0;
	B30 = 0;B31=0;B32 = 0;B33=0;


	


	

	//#10000;
	
end






assign A = {{A33,A32,A31,A30},
		{A23,A22,A21,A20},
		{A13,A12,A11,A10},
		{A03,A02,A01,A00}};


assign B = {{B33,B32,B31,B30},
		{B23,B22,B21,B20},
		{B13,B12,B11,B10},
		{B03,B02,B01,B00}};


assign C00 = C[N-1:0];
assign C10 = C[4*N+N-1:4*N];
assign C20 = C[8*N+N-1:8*N];
assign C30 = C[12*N+N-1:12*N];

assign C01 = C[N+N-1:N];
assign C11 = C[N+4*N+N-1:4*N+N];
assign C21 = C[N+8*N+N-1:8*N+N];
assign C31 = C[N+12*N+N-1:12*N+N];

assign C02 = C[2*N+N-1:+2*N];
assign C12 = C[2*N+4*N+N-1:4*N+2*N];
assign C22 = C[2*N+8*N+N-1:8*N+2*N];
assign C32 = C[2*N+12*N+N-1:12*N+2*N];

assign C03 = C[3*N+N-1:3*N];
assign C13 = C[3*N+4*N+N-1:4*N+3*N];
assign C23 = C[3*N+8*N+N-1:8*N+3*N];
assign C33 = C[3*N+12*N+N-1:12*N+3*N];


always #5 clk=~clk;

endmodule