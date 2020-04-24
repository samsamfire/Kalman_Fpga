module state_update#(
	parameter N = 32,
	parameter Q = 18,
	parameter T = 0.00001
)
(	input signed [N-1:0] ialpha,ibeta,valpha,vbeta,omega,theta,
	input clk,reset,
	output signed [N-1:0] ialphae,ibetae,omegae,thetae
	);


<<<<<<< HEAD
//
=======
wire signed [N-1:0] mult_temp0,mult_temp1,mult_temp2,mult_temp3,mult_temp4,mult_temp5,mult_temp6,mult_temp7,mult_temp8;
wire of0,of1,of2,of3,of4,of5,of6,of7,of8,of9,of10;
>>>>>>> cfc9864... -added cosimEKF







<<<<<<< HEAD
=======
qmult #(Q,N) mult0(valpha,Ts_Ls,mult_temp0,of0);
qmult #(Q,N) mult1(ialpha,Rs_Ts_Ls,mult_temp1,of1);
qmult #(Q,N) mult2(stheta,Lambda_Ts_Ls,mult_temp2,of2);
qmult #(Q,N) mult3(omega,mult_temp2,mult_temp3,of3);
>>>>>>> cfc9864... -added cosimEKF




<<<<<<< HEAD
=======
qmult #(Q,N) mult4(vbeta,Ts_Ls,mult_temp4,of4);
qmult #(Q,N) mult5(ibeta,Rs_Ts_Ls,mult_temp5,of5);
qmult #(Q,N) mult6(ctheta,Lambda_Ts_Ls,mult_temp6,of6);
qmult #(Q,N) mult7(omega,mult_temp6,mult_temp7,of7);

assign ibetae = ibeta + mult_temp4 - mult_temp5 - mult_temp7;


// omegae = omega

assign omegae = omega;

//thetae = theta + omega*Ts

qmult #(Q,N) mult8 (omega,T,mult_temp8,of8);

assign thetae = theta + mult_temp8;
///////////////////////////////////////////////////////////////////JACOBIAN MATRIX////////////////////////////////////////////////////////////////////


//Jacobian Matrix coefficients
wire signed [N-1:0] F0[0:3],F1[0:3],F2[0:3],F3[0:3]; //Line 1 to 4


//Line 1
assign F0[0] = F00;   // 1 - R*Ts/Ls
assign F0[1] = 0;
assign F0[2] = mult_temp2;  // sin(theta)*(Lambda*Ts/Ls)
assign F0[3] = mult_temp7; // omega*cos(theta)*(Lambda*Ts/Ls)

//Line 2
assign F1[0] = 0;
assign F1[1] = F00; // 1 - R*Ts/Ls
assign F1[2] = -mult_temp6; //  -cos(theta)*(Lambda*Ts/Ls)
assign F1[3] = mult_temp3; // omega*sin(theta)*(Lambda*Ts/ls)

//Line 3
assign F2[0] =0;
assign F2[1] = 0;
assign F2[2] = sf;
assign F2[3] = 0;
//Line 4

assign F3[0] =0;
assign F3[1] = 0;
assign F3[2] = T;
assign F3[3] = sf;

assign F = {F3[3],F3[2],F3[1],F3[0],F2[3],F2[2],F2[1],F2[0],F1[3],F1[2],F1[1],F1[0],F0[3],F0[2],F0[1],F0[0]};
assign F_transpose = {F3[3],F2[3],F1[3],F0[3],F3[2],F2[2],F1[2],F0[2],F3[1],F2[1],F1[1],F0[1],F3[0],F2[0],F1[0],F0[0]};
>>>>>>> cfc9864... -added cosimEKF



endmodule