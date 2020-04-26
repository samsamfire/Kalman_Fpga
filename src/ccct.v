
///Module for calculating Park, Clark and Park inverse.


module parallel_park #(parameter N = 32, parameter Q = 18)
( 	input signed [N-1:0] id, iq,ctheta,stheta,
	output signed [N-1:0] ialpha, ibeta


);

wire signed [N-1:0] mult_temp0, mult_temp1,mult_temp2,mult_temp3,mult_temp4;
wire of0,of1,of2,of3;

qmult #(Q,N) mult0(ctheta,id,mult_temp0,of0);
qmult #(Q,N) mult1(stheta,id,mult_temp1,of1);
qmult #(Q,N) mult2(ctheta,iq,mult_temp2,of2);
qmult #(Q,N) mult3(stheta,iq,mult_temp3,of3);


assign ialpha = mult_temp0 - mult_temp3;  // ialpha = cos(theta)*id - sin(theta)*iq

assign ibeta = mult_temp1 + mult_temp2; //ibeta = cos(theta)*iq + sin(theta)*id


endmodule








module parallel_clark #(parameter N=32, parameter Q=18)
(
	input signed [N-1:0] ialpha, ibeta,
	output signed [N-1:0] ia,ib,ic 
	);

//Q17 format for parameters
localparam [17:0] a = 0.5; // 0.5
localparam [17:0] b = 18'b0110111011011001000; //sqrt3/2



assign ia = ialpha;  // ia = ialpha

assign ib = b*ibeta - a*ialpha; //ib = (sqrt(3)/2)*ibeta - 0.5*ialpha

assign ic = -b*ibeta - a*ialpha; //ic = -(sqrt(3)/2)*ibeta - 0.5*ialpha


endmodule
