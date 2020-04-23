//Created by Samuel Lee
// 32-bit signed fixed point multiplier with overflow
//Q is fraction length and N is integer length


//Normal LPM multiplier with 32-bit inputs and 64-bit output is 8 embedded 9-bit multipliers & 92 logic elements
//This module for 32-bit fixed-point signed multiplication is 6 embedded 9-bit multiplliers & 207 logic elements

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
  reg signed [2*N-1:0] result_absolute;


  // //Using ALTERA-LPM module

  reg signed [N-1:0] multiplier_ina,multiplier_inb;
  wire signed [N-1:0] multiplier_out;

  mult mult0(multiplier_ina,multiplier_inb,multiplier_out);
  
  
always @(a,b) begin
  //If a is negative and b positive
  if(sign_a && !sign_b) begin

    multiplier_ina = a_inv;
    multiplier_inb = b;
    result_absolute = multiplier_out;
    temp_result = - result_absolute;
  end
  //If a is positive and b negative
  else if(!sign_a && sign_b) begin
    multiplier_ina = a;
    multiplier_inb = b_inv;
    result_absolute = multiplier_out;
    temp_result = - result_absolute;
  end
  //If a and b are both negative
  else if(sign_a && sign_b) begin
    multiplier_ina = a_inv;
    multiplier_inb = b_inv;
    result_absolute = multiplier_out;
    temp_result = result_absolute;
  end
  //If a and b are both positive
  else begin
    multiplier_ina = a_inv;
    multiplier_inb = b_inv;
    result_absolute = multiplier_out;
    temp_result = result_absolute;
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
 
  
  
  
endmodule
