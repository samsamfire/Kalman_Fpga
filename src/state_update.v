module state_update#(
	parameter N = 32,
	parameter Q = 18,
	parameter Ts = 0.00001
)
(	input signed [N-1:0] ialpha,ibeta,valpha,vbeta,omega,theta,stheta,ctheta,
	input clk,reset,
	output signed [N-1:0] ialphae,ibetae,omegae,thetae,
	output signed [4*4*N-1:0] F,F_transpose  //Jacobian matrix
);

////////////////////////////////////////////////////////////////MACHINE PARAMETERS//////////////////////////////////////////////////////////////
parameter sf = 2**Q;
parameter Rs = 1.477;
parameter Lambda = 0.2026;
parameter Ls = 0.0211;
parameter signed [N-1:0] Lambda_Ts_Ls = ((Lambda*Ts)/Ls)*sf;
parameter signed [N-1:0] Rs_Ts_Ls = ((Rs*Ts)/Ls)*sf;
parameter signed [N-1:0] F00 = sf - Rs_Ts_Ls;
parameter signed [N-1:0] Ts_Ls = (Ts/Ls)*sf;
parameter signed [N-1:0] T = Ts*sf;
parameter signed [N-1:0] numerator = 2**31-1;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

wire signed [N-1:0] mult_temp0,mult_temp1,mult_temp2,mult_temp3,mult_temp4,mult_temp5;





////////////////////////////////////////////////////////////////STATE ESTIMATION///////////////////////////////////////////////////////////////////

//ialphae = ialpha + (valpha -R*ialpha + omega*Lambda*sin(theta)) * (Ts/Ls)

qmult #(Q,N) mult0(valpha,Ts_Ls,mult_temp0);
qmult #(Q,N) mult1(ialpha,Rs_Ts_Ls,mult_temp1);
qmult #(Q,N) mult2(stheta,Lambda_Ts_Ls,mult_temp2);
qmult #(Q,N) mult3(omega,mult_temp2,mult_temp3);

assign ialphae = ialpha + mult_temp0 - mult_temp1 + mult_temp3;


//ibetae = ibeta + (vbeta -R*ibeta - omega*Lambda*cos(theta)) * (Ts/Ls)

qmult #(Q,N) mult4(vbeta,Ts_Ls,mult_temp4);
qmult #(Q,N) mult5(ibeta,Rs_Ts_Ls,mult_temp5);
qmult #(Q,N) mult6(ctheta,Lambda_Ts_Ls,mult_temp6);
qmult #(Q,N) mult7(omega,mult_temp6,mult_temp7);

assign ibetae = ibeta + mult_temp4 - mult_temp5 - mult_temp7;


// omegae = omega

assign omegae = omega;

//thetae = theta + omega*Ts

qmult #(Q,N) mult8 (omega,Ts,mult_temp8);

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



endmodule