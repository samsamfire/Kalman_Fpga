// Code your design here

// 4x4 32 bits matrix multiplication

module parallel_matrix_multiply(A,B,C,clk,reset);
  
  //Parallel
  parameter N=32;
  parameter P=4;
  
  
  input signed [P*P*N-1:0] A;
  input signed [P*P*N-1:0] B;
  
  output signed [P*P*N-1:0] C;
  input clk,reset;
  
  genvar i,j;
  generate
    for(j=0;j<P;j=j+1) begin : gen1
      for (i=0;i<P;i=i+1) begin : gen2
        coefficient_calc #(32,18,4) coeff(A[(P*N*(i+1))-1:P*N*i],{B[N+3*P*N-1+j*N:3*P*N+j*N],B[N+2*P*N-1+j*N:2*P*N+j*N],B[N+P*N-1+j*N:P*N+j*N],B[N-1+j*N:j*N]},C[j*N*P+(i+1)*N-1:j*N*P+i*N],clk,reset);
        
      end
       
    end
  endgenerate
  
  
  
  
endmodule

//coefficient_calc #(32,18,4) coeff(A[(P*N*(i+1))-1:P*N*(i+1)],{B[N-1+j*N:j*N],B[N+P*N-1+j*N:P*N+j*N],B[N+2*P*N-1+j*N:2*P*N+j*N],B[N+3*P*N-1+j*N:3*P*N+j*N]},C[j*N*P+(i+1)*N-1;j*N*P+i*N],clk,reset);
 
  



module coefficient_calc #(
  parameter N=32, //Size of coefficients
  parameter Q = 18, //Number of fractional Bits
  parameter P=4)
  (A,B,out, clk, reset,overflow_coeff);
  
  //Input A as a column and B as a row. Must have the same nb of elements defined by P.
  

  
  input signed [P*N-1:0] A,B;
  input clk,reset;
  output overflow_coeff;
  output signed [N-1:0] out;
  
  wire signed [N-1:0] cout[0:3];
  wire signed [N-1:0] cout_total;
  wire overflow [0:3];
  
  
  
  //Generate multipliers
  genvar i;
  generate
    for (i=0;i<P; i=i+1) begin : gen
      //assign cout[i] = A[(i+1)*N-1: i*N]*B[(i+1)*N-1: i*N];
      qmult #(Q,N) mult(A[(i+1)*N-1: i*N],B[(i+1)*N-1: i*N],cout[i],overflow[i]);
    end
  endgenerate
  
  assign cout_total = cout[0] + cout[1] + cout[2] + cout[3];
  initial begin
   
  end
  
  assign out = cout_total;
  assign overflow_coeff = overflow[0] || overflow[1] || overflow[2] || overflow[3];
    

  
  
 
endmodule

