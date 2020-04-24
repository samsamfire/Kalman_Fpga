//Fixed point 32-bit kalman filter Q = 18 & N = 32
module kalman #(
  parameter N=32,
  parameter Q=18
)
(
	input signed [N-1:0] valpha,vbeta,
	input signed [N-1:0] ialpham,ibetam,
	input signed [N-1:0] ctheta_t,stheta_t,
	input signed [N-1:0] nbSamples,
	input clk,reset,
	output reg signed [N-1:0] omega,theta,
	output reg signed [N-1:0] ialphak,ibetak,kmatrix00,kmatrix01,kmatrix10,kmatrix11,kmatrix20,kmatrix21,kmatrix30,kmatrix31
);

  //Store inputs 

reg signed [N-1:0] valpha_i,vbeta_i;


//Constant parameters declaration;
parameter signed [N-1:0] sf = 2**Q;
parameter signed [N-1:0] sf_5 = 5*2**Q;
parameter P = 4;


////////////////////////////////////////////////////////////////Machine Parameters//////////////////////////////////////////////////////////////
parameter Rs = 1.477;
parameter Lambda = 0.2026;
parameter Ts = 0.00001;
parameter Ls = 0.0211;
parameter signed [N-1:0] Lambda_Ts_Ls = ((Lambda*Ts)/Ls)*sf;
parameter signed [N-1:0] Rs_Ts_Ls = ((Rs*Ts)/Ls)*sf;
parameter signed [N-1:0] F00 = sf - Rs_Ts_Ls;
parameter signed [N-1:0] Ts_Ls = (Ts/Ls)*sf;
parameter signed [N-1:0] T = Ts*sf;
parameter signed [N-1:0] numerator = 2**31-1;


/////////////////////////////////////////////////////////////Matrixes declaration/////////////////////////////////////////////////////////////

//4x4 matrixes
reg signed [4*4*N-1:0] Pk_prev, Pk_mult_result, P_matrix , temp_matrix; 
wire signed [4*4*N-1:0] I_KC,Pk_C_transpose,matrix_invert,Pk,F,F_transpose,I;
//Q matrix (only diagonal coefficients matter)
parameter signed [N-1:0] Q00 = 0.01*sf;
parameter signed [N-1:0] Q11=0.01*sf;
parameter signed [N-1:0] Q22=20*sf;
parameter signed [N-1:0] Q33=0.0001*sf;

//R matrix 2x2 matrix
parameter signed [N-1:0] R00 = 0.92*sf;
parameter signed [N-1:0] R01 = 0;
parameter signed [N-1:0] R10 = 0;
parameter signed [N-1:0] R11 = 0.92*sf;

//Jacobian Matrix coefficients
wire signed [N-1:0] F0[0:3],F1[0:3],F2[0:3],F3[0:3]; //Line 1 to 4

//P Matrix coefficients
wire signed [N-1:0] P0[0:3],P1[0:3],P2[0:3],P3[0:3]; //Line 1 to 4

//K matrix coefficients
wire signed [N-1:0] K00,K01,K10,K11,K20,K21,K30,K31;				

//Intermediate variables
wire signed [N-1:0] Q_intermediate0,Q_intermediate1,Q_intermediate2,Q_intermediate3;
wire signed [N-1:0] R_intermediate0,R_intermediate1,R_intermediate2,R_intermediate3;
wire signed [N-1:0] R0[0:1],R1[0:1];


////////////////////////////////////////////////////////////////MISC//////////////////////////////////////////////////////////////////////////////////




 
////////////////////////////////////////////////////////State vectors//////////////////////////////////////////////////////////////////////////////
  
wire signed [N-1:0] xe[0:3],x_intermediate1[0:3],x_intermediate2[0:3],x[0:3]; //Xe is a 4x1 vector containing ialpha,ibeta,omega,theta
wire signed [N-1:0] xe_intermediate[0:1];

wire signed [N-1:0] error_i[0:1]; //Current error between measured and estimated
reg signed [N-1:0] yk[0:1],xe_prev[0:3];
reg signed [N-1:0] omegae,thetae; //Omega estimated and Theta estimated



//Temporary variables


wire signed [N-1:0] mult_temp1,mult_temp2,mult_temp3,mult_temp4,mult_temp5,mult_temp6,mult_temp7,mult_temp8,dtheta;
wire signed [N-1:0] mult_temp10,mult_temp11;

  
  
//////////////////////////////////////////////////////////STATE MACHINE VARS///////////////////////////////////////////////////////////
  
reg [3:0] state;
parameter [2:0] s0=3'b000;

parameter compute_F_x_Pk = 3'b001;
parameter compute_F_x_Pk_x_Ftranspose = 3'b010;
parameter compute_K = 3'b011;

parameter compute_P = 3'b100;
parameter update_prev =3'b101;
parameter wait_state = 3'b110;


reg [N-1:0] wait_counter;
reg [4:0] compute_P_counter = 5'b0000; 

  
///////////////////////////////////////////////////////MODULE INSTANCIATIONS ////////////////////////////////////////////////////////////////////////////

  
////Matrix multiply module  
reg reset_smm;
reg signed [4*4*N-1 : 0] matrix_inA,matrix_inB;
wire signed [4*4*N-1 : 0] result_Matrix;

 
sequential_matrix_multiply #(N,Q,4) smm(matrix_inA,matrix_inB,result_Matrix,clk,reset_smm);


/////Determinant calculation for C*Pk*C + R module
wire signed [2*N-1:0] div_intermediate;
wire signed [N-1:0] quotient,remain;
wire signed [N-1:0] inverse_det;
wire signed[N-1:0] det;
wire signed [N-1:0] det_interm1,det_interm2;

div2 det_calculate(det,numerator,quotient,remain);

//////Overflow variables
wire of1,of2,of3,of4,of5,of6,of7,of8,of9,of10,of11,of12,of13,of14,of15,of16,of17,of18,of19,of20,of21,of22,of23,of24;


wire signed [N-1:0] ctheta,stheta;


///////Cordic module for calculating sine and cosine
parameter width = 20;
parameter signed [N-1:0] ratio_cos =2067.6;
parameter An = (2**Q)/1.647;
reg [width-1:0] Xin,Yin;


CORDIC cordic0(.clock(clk),.cosine(ctheta),.sine(stheta),.x_start(Xin),.y_start(Yin),.angle(thetae * ratio_cos));

////State estimation module

state_update #(N,Q,Ts) state_estimate(
	.ialpha(xe_prev[0]),.ibeta(xe_prev[1]),
	.vbeta(vbeta_i),.valpha(valpha_i),
	.omega(xe_prev[2]),
	.theta(xe_prev[3]),.ctheta(ctheta),.stheta(stheta),
	.ialphae(xe[0]),.ibetae(xe[1]),.omegae(xe[2]),.thetae(xe[3]),
	.F(F),.F_transpose(F_transpose)
	);




/////////////////////////////////////////////////////////////////////////MAIN///////////////////////////////////////////////////////////////////////////////////
  
always @(posedge clk, negedge reset) begin

if(!reset) begin

  Xin <= An;
  Yin <= 0;
  omegae <= 0;
  thetae <= 0;
  xe_prev[0] <= 0;
  xe_prev[1] <=0;
  xe_prev[2] <= 0;
  xe_prev[3] <=0;
  wait_counter <=0;
  ////P0 initialisation
  Pk_prev <= {{1*sf,32'b0,32'b0,32'b0},{32'b0,10*sf,32'b0,32'b0},{32'b0,32'b0,5*sf,32'b0},{32'b0,32'b0,32'b0,5*sf}};
  temp_matrix <= 0;
  //reset_smm=1;
  state <= s0;
  reset_smm <=0;
end
else begin
  case(state) 
    s0: begin
	  
	
    state <= compute_F_x_Pk;
    //Get curents and stuff
    yk[0] <= ialpham;
    yk[1] <= ibetam;
    valpha_i <= valpha;
    vbeta_i <= vbeta;


   
      
    end
    //Computing Pk
  	compute_F_x_Pk: begin   //First is F*Pk_prev
  	  reset_smm <= 1;
      matrix_inA <= F;   
      matrix_inB <= Pk_prev; 
      
      if(compute_P_counter < 18) begin  //Wait 18 clock cycles before getting result
      	compute_P_counter <= compute_P_counter + 1;
    	end
      else begin
        temp_matrix <= result_Matrix; //Result of F*Pk_prev has been processed, next step is to multiply by Ftranspose
        state <= compute_F_x_Pk_x_Ftranspose;
        compute_P_counter <=0;  //Reset counter and Multiply module
        reset_smm <=0;
        
    	end
    end
    
    compute_F_x_Pk_x_Ftranspose:begin 
      //Initially put prev_result and F_transpose into multiply module 
      if(compute_P_counter == 0) begin
      reset_smm <= 1;
      matrix_inA <= temp_matrix;
      matrix_inB <= F_transpose;
      end
      
      if(compute_P_counter < 18) begin
        compute_P_counter <= compute_P_counter + 1;
    	end
      else begin
        
        state <= compute_K;
        compute_P_counter <=0;   //Pk has been calculated
        Pk_mult_result <= result_Matrix; // put matrix result inside Pk_mult_result;
        reset_smm <=0;
        
        
       
      end
     
    end
    //Second step is to compute K (kalman gain)
    compute_K: begin
       
      reset_smm <= 1;
      //Pk is ready
      matrix_inA <= Pk_C_transpose; //This is a wire so doesnt change because Pk doesnt change
      matrix_inB <= matrix_invert; //This is also a wire so won't change during this part of execution
      
      if(compute_P_counter < 18) begin
        compute_P_counter <= compute_P_counter + 1;
    	end
      else begin
        //K is almost totally computed. we still need to divide by determinant of matrix inverse
        temp_matrix <= result_Matrix;
        state <= compute_P;
        compute_P_counter <=0;
        reset_smm <=0;
        //$display("P00",,$itor(P0[0])/sf,,"K00",$itor(K00)/sf ,, $itor(R_intermediate1)/sf);
     
        
      end
      
    end
///////////////Compute P = (I-K*C)*Pk       
    compute_P: begin
      reset_smm <= 1;
      
      matrix_inA <= I_KC;
      matrix_inB <= Pk;
      if(compute_P_counter < 18) begin
        compute_P_counter <= compute_P_counter + 1;
    	end
      else begin
        //P is computed will be the next Pk_prev
        P_matrix <= result_Matrix;
        state <= update_prev;
        compute_P_counter <=0;
        reset_smm <=0;
      end
    end
    
    
    
    update_prev: begin
      
      Pk_prev <= P_matrix;
      xe_prev[0] <= x[0];
      xe_prev[1] <= x[1];
      xe_prev[2] <= x[2];
      xe_prev[3] <= x[3];
      
      
    omegae <= x[2];
    thetae <= x[3];
		 
	omega <= x[2];
	theta <= x[3];

	ialphak <= xe[0];
	ibetak <= xe[1];

	kmatrix00 <= K00;
	kmatrix01 <= K01;
	kmatrix10 <= K10;
	kmatrix11 <= K11;
	kmatrix20 <= K20;
	kmatrix21 <= K21;
	kmatrix30 <= K30;
	kmatrix31 <= K31;

	// kmatrix00 <= P0[0];
	// kmatrix01 <= P0[1];
	// kmatrix10 <= P1[0];
	// kmatrix11 <= P1[1];
	// kmatrix20 <= P2[0];
	// kmatrix21 <= P2[1];
	// kmatrix30 <= P3[0];
	// kmatrix31 <= P3[1];


	state <=wait_state;
	end

	wait_state: begin
	  //Waiting for a new value
    if(wait_counter<= nbSamples) begin
      wait_counter <= wait_counter + 1;
	      //state <= wait_state;
    end
    else begin
      wait_counter <= 0;
      state <= s0;
    end

    end
    
      
endcase 
end
end
      

  assign Q_intermediate0 = Pk_mult_result[N-1:0] + Q00;
  assign Q_intermediate1 = Pk_mult_result[N-1+5*N:5*N]+Q11;
  assign Q_intermediate2 = Pk_mult_result[N-1+10*N:10*N]+Q22;
  assign Q_intermediate3 = Pk_mult_result[N-1+15*N:15*N]+Q33;
  
  
  assign R_intermediate0 = (R00 + P0[0]);
  assign R_intermediate1 = (R11 + P1[1]);
  assign R_intermediate2 =  -P1[0];
  assign R_intermediate3 =  -P0[1];

  qmult #(Q,N) mult10(R_intermediate0,inverse_det,R1[1],of11);
  qmult #(Q,N) mult11(R_intermediate1,inverse_det,R0[0],of12);

  qmult #(Q,N) mult12(R_intermediate2,inverse_det,R1[0],of13);
  qmult #(Q,N) mult13(R_intermediate3,inverse_det,R0[1],of14);

  
  assign Pk = { Q_intermediate3,Pk_mult_result[15*N-1:11*N], Q_intermediate2,Pk_mult_result[10*N-1:6*N], Q_intermediate1,Pk_mult_result[5*N-1:N], Q_intermediate0};
  
  genvar i;
  generate
    for (i=0;i<4;i=i+1) begin : P_m
      assign P0[i] = Pk[(i+1)*N-1:i*N];
      assign P1[i] = Pk[(i+1)*N-1 + 4*N:i*N+4*N];
      assign P2[i] = Pk[(i+1)*N-1 + 8*N:i*N+8*N];
      assign P3[i] = Pk[(i+1)*N-1 + 12*N:i*N+12*N];
    end
  endgenerate
  
  assign Pk_C_transpose = {{32'b0,32'b0,P3[1],P3[0]},
                           {32'b0,32'b0,P2[1],P2[0]},
                           {32'b0,32'b0,P1[1],P1[0]},
                           {32'b0,32'b0,P0[1],P0[0]}};

  assign matrix_invert = {{32'b0,32'b0,32'b0,32'b0},
                          {32'b0,32'b0,32'b0,32'b0},
                          {32'b0,32'b0,R1[1],R1[0]},
                          {32'b0,32'b0,R0[1],R0[0]}};
  
  
  assign I = {{sf,32'b0,32'b0,32'b0},
                   {32'b0,sf,32'b0,32'b0},
                   {32'b0,32'b0,sf,32'b0},
                   {32'b0,32'b0,32'b0,sf}};
    
  
  
  
  //Determinant calculation
  qmult #(Q,N) mult22(R_intermediate0,R_intermediate1,det_interm1,of23);
  qmult #(Q,N) mult23(P1[0],P0[1],det_interm2,of24);
  //assign det_interm1 = (R00+P0[0])*(R11+P1[1]);
  //assign det_interm2 = -P1[0]*P0[1];
  
  assign det = det_interm1 - det_interm2;
  
  
  
  //K matrix
  
  assign K00 = temp_matrix[N-1:0];
  assign K01 = temp_matrix[2*N-1:N];
  assign K10 = temp_matrix[N-1+P*N:P*N];
  assign K11 = temp_matrix[2*N-1 + P*N:N+P*N];
  assign K20 = temp_matrix[N-1+2*P*N:2*P*N];
  assign K21 = temp_matrix[2*N-1+2*P*N:N+2*P*N];
  assign K30 = temp_matrix[N-1+3*P*N:3*P*N];
  assign K31 = temp_matrix[2*N-1+3*P*N:N+3*P*N];
  
  ////////////////////////////state calculation//////////////
  
  
  assign error_i[0] = yk[0]-xe[0];
  assign error_i[1] = yk[1]-xe[1]; 
  


  qmult #(Q,N) mult14(K00,error_i[0],x_intermediate1[0],of15);
  qmult #(Q,N) mult15(K10,error_i[0],x_intermediate1[1],of16);
  qmult #(Q,N) mult16(K20,error_i[0],x_intermediate1[2],of17);
  qmult #(Q,N) mult17(K30,error_i[0],x_intermediate1[3],of18);

  qmult #(Q,N) mult18(K01,error_i[1],x_intermediate2[0],of19);
  qmult #(Q,N) mult19(K11,error_i[1],x_intermediate2[1],of20);
  qmult #(Q,N) mult20(K21,error_i[1],x_intermediate2[2],of21);
  qmult #(Q,N) mult21(K31,error_i[1],x_intermediate2[3],of22);

  
  assign x[0] = xe[0] +  x_intermediate1[0] + x_intermediate2[0];
  assign x[1] = xe[1] +  x_intermediate1[1] + x_intermediate2[1];
  assign x[2] = xe[2] +  x_intermediate1[2] + x_intermediate2[2];
  assign x[3] = xe[3] +  x_intermediate1[3] + x_intermediate2[3];
               
  ////////////////////////Compute P///////////////////////////:             
  
  assign I_KC ={{sf,32'b0,-K31,-K30},
             {32'b0,sf,-K21,-K20},
             {32'b0,32'b0,sf-K11,-K10},
             {32'b0,32'b0,-K01,sf-K00}};
             
            
  assign div_intermediate = (2**Q)/det;  //assign div_intermediate = det*quotient/(2**13);

  //assign inverse_det = div_intermediate[N-1+Q:Q] + remain/2**13;
  assign inverse_det = quotient*2**5;

endmodule



