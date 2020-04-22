//Module for creating a BRAM matrix
module matrixmem(

	input clk,
  	input [3:0] addr,
  	input [31:0] data_in,
  	output reg [31:0] data_out,
	input write,
	input read
);

reg [31:0] mem [0:15]; //16 32-bit elements, enough to store one 4x4 array.

always @(posedge clk) begin
   if(write)
       mem[addr] <= data_in;
   else if (read)
       data_out <= mem[addr];
end
endmodule


module sequential_matrix_multiply #(parameter N = 32, parameter Q = 18,parameter P=4) (A,B,C,clk,reset);
  
  input clk,reset;
  input signed [P*P*N-1:0] A;
  input signed [P*P*N-1:0] B;
  
  output reg signed [P*P*N-1:0] C;
  
  reg signed [P*N-1:0] lineA,columnB;
  reg  [3:0] addressA,aB;
  
  reg [3:0] state,next_state;
  
  wire signed [N-1:0] cii;
  wire overflow_coeff;
  
  
  
  coefficient_calc #(N,Q,P) coef_calc(lineA,columnB,cii,clk,reset,overflow_coeff);
  
  always @(posedge clk,negedge reset) begin
    
    if(!reset) begin
      state <= 0;
    end
    else  begin
    case(state)
      0: begin
        lineA <= A[N*P-1:0];
        columnB <= {B[N-1 +3*P*N:3*P*N],B[N-1 +2*P*N:2*P*N],B[N-1+N*P:N*P],B[N-1:0]};
        C[16*N-1:15*N] <= cii;
        
        state <= 1;
      end
      1: begin
        columnB <= {B[2*N-1 +3*P*N:3*P*N+N],B[2*N-1 +2*P*N:2*P*N+N],B[2*N-1+N*P:N*P+N],B[2*N-1:N]};
        C[N-1:0] <= cii;
        state <= 2;
      end
      2: begin
        columnB <= {B[3*N-1 +3*P*N:3*P*N+2*N],B[3*N-1 +2*P*N:2*P*N+2*N],B[3*N-1+N*P:N*P+2*N],B[3*N-1:2*N]};
        C[2*N-1:N] <= cii;
        state <= 3;
      end
      3: begin
        columnB <= {B[4*N-1 +3*P*N:3*P*N+3*N],B[4*N-1 +2*P*N:2*P*N+3*N],B[4*N-1+N*P:N*P+3*N],B[4*N-1:3*N]};
        C[3*N-1:2*N] <= cii;
        state <= 4;
      end
      ///////////////////////////////////////////////////////////////////////////////
      4: begin
        lineA <= A[2*N*P-1:N*P];
        columnB <= {B[N-1 +3*P*N:3*P*N],B[N-1 +2*P*N:2*P*N],B[N-1+N*P:N*P],B[N-1:0]};
        C[4*N-1:3*N] <= cii;
        state <= 5;
      end
      5: begin
        columnB <= {B[2*N-1 +3*P*N:3*P*N+N],B[2*N-1 +2*P*N:2*P*N+N],B[2*N-1+N*P:N*P+N],B[2*N-1:N]};
        C[5*N-1:4*N] <= cii;
        state <= 6;
      end
      6: begin
        columnB <= {B[3*N-1 +3*P*N:3*P*N+2*N],B[3*N-1 +2*P*N:2*P*N+2*N],B[3*N-1+N*P:N*P+2*N],B[3*N-1:2*N]};
        C[6*N-1:5*N] <= cii;
        state <= 7;
      end
      7: begin
        columnB <= {B[4*N-1 +3*P*N:3*P*N+3*N],B[4*N-1 +2*P*N:2*P*N+3*N],B[4*N-1+N*P:N*P+3*N],B[4*N-1:3*N]};
        C[7*N-1:6*N] <= cii;
        state <= 8;
      end
      ///////////////////////////////////////////////////////////////////////////////
      8: begin
        lineA <= A[3*N*P-1:2*N*P];
        columnB <= {B[N-1 +3*P*N:3*P*N],B[N-1 +2*P*N:2*P*N],B[N-1+N*P:N*P],B[N-1:0]};
        C[8*N-1:7*N] <= cii;
        state <= 9;
      end
      9: begin
        columnB <= {B[2*N-1 +3*P*N:3*P*N+N],B[2*N-1 +2*P*N:2*P*N+N],B[2*N-1+N*P:N*P+N],B[2*N-1:N]};
        C[9*N-1:8*N] <= cii;
        state <= 10;
      end
      10: begin
        columnB <= {B[3*N-1 +3*P*N:3*P*N+2*N],B[3*N-1 +2*P*N:2*P*N+2*N],B[3*N-1+N*P:N*P+2*N],B[3*N-1:2*N]};
        C[10*N-1:9*N] <= cii;
        state <= 11;
      end
      11: begin
        columnB <= {B[4*N-1 +3*P*N:3*P*N+3*N],B[4*N-1 +2*P*N:2*P*N+3*N],B[4*N-1+N*P:N*P+3*N],B[4*N-1:3*N]};
        C[11*N-1:10*N] <= cii;
        state <= 12;
      end
      ///////////////////////////////////////////////////////////////////////////////
      12: begin
        lineA <= A[4*N*P-1:3*N*P];
        columnB <= {B[N-1 +3*P*N:3*P*N],B[N-1 +2*P*N:2*P*N],B[N-1+N*P:N*P],B[N-1:0]};
        C[12*N-1:11*N] <= cii;
        state <= 13;
      end
      13: begin
        columnB <= {B[2*N-1 +3*P*N:3*P*N+N],B[2*N-1 +2*P*N:2*P*N+N],B[2*N-1+N*P:N*P+N],B[2*N-1:N]};
        C[13*N-1:12*N] <= cii;
        state <= 14;
      end
      14: begin
        columnB <= {B[3*N-1 +3*P*N:3*P*N+2*N],B[3*N-1 +2*P*N:2*P*N+2*N],B[3*N-1+N*P:N*P+2*N],B[3*N-1:2*N]};
        C[14*N-1:13*N] <= cii;
        state <= 15;
      end
      15: begin
        columnB <= {B[4*N-1 +3*P*N:3*P*N+3*N],B[4*N-1 +2*P*N:2*P*N+3*N],B[4*N-1+N*P:N*P+3*N],B[4*N-1:3*N]};
        C[15*N-1:14*N] <= cii;
        state <= 0;
      end
      
      default:state <= 1'b0;
      
        
    endcase
    end
  end
        
        
    
    
   

                 
        
  
  
endmodule