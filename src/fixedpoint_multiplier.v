//Created by Samuel Lee


module qmult#(//Parameterized values
  parameter Q = 18,
  parameter N = 32
  )
  (
    input     [N-1:0] a,
    input     [N-1:0] b,
  output      [N-1:0] o_result,
  output  reg       ovr
  );
  
  wire sign_a,sign_b;
  wire signed [N-1:0] a_inv,b_inv;
  reg [2*N-1:0] temp_result;
  reg signed [2*N-1:0] result;
  reg signed [2*N-1:0] result_absolute;
  
  
  always @(a,b) begin
    //If a is negative and b positive
    if(sign_a && !sign_b) begin
      result_absolute = a_inv*b;
      if(result_absolute[N-1+Q:Q] == 0) begin   //Check if result should be 0, (avoids rounding to smallest negative)
        temp_result = 32'b0;
      end
      else begin
        temp_result = - a_inv*b;
      end
    end
    //If a is positive and b negative
    else if(!sign_a && sign_b) begin
      result_absolute = a*b_inv;
      if(result_absolute[N-1+Q:Q] == 0) begin
        temp_result = 32'b0;
      end
      else begin
        temp_result = - a*b_inv;
      end
    end
    //If a and b are both negative
    else if(sign_a && sign_b) begin
      result_absolute = a_inv*b_inv;
      temp_result = a_inv*b_inv;
    end
    //If a and b are both positive
    else if(!sign_a  &&  !sign_b)begin
      result_absolute = a * b;
      temp_result = a*b;
    end
    
    //Check for overflow, overflow happens if the upper bits of result_absolute are non-zero
    
    if(result_absolute[2*N-1:N-1+Q] != 0)
       ovr = 1'b1;
    else
      ovr = 1'b0;
    
    
  end
  
  
 
  assign sign_a = a[N-1];
  assign sign_b = b[N-1];
  assign o_result = temp_result[N-1+Q:Q];
  assign a_inv = -a;
  assign b_inv = -b;
  assign test = result_absolute[N-1+Q:Q];
  
  
  
endmodule
