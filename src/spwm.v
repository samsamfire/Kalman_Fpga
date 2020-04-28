//`timescale 1ns/1ns

//32'b0000_1100_1100_1100_1100_1100_1100_1101 = 0.8

/*
>> Counter will generate a 5kHz traingel wave for a 10ns clock period
*/

module spwm #(parameter N, parameter Q, parameter signed [31:0] m = 32'b0001_0000_0000_0000_0000_0000_0000_0000)
    (output reg Sau, output reg Sal, output reg Sbu, output reg Sbl, output reg Scu, output reg Scl, output signed [N-1:0] out,
    input signed [N-1:0] x, input signed [N-1:0] y, input signed [N-1:0] z, input clk, input res);
    
    wire signed [N-1:0] count;
    wire of1,of2,of3;
    
    counter uut(clk,res, count);
    
    assign out = count;
  
	 wire signed [N-1:0] x_m, y_m, z_m;
	 
    qmult u1 (x, m, x_m,of1);
    qmult u2 (y, m, y_m,of2);
    qmult u3 (z, m, z_m,of3);
    
    always@(count, x, y, z)
    begin
	 
    if (x_m < count) begin
        Sau <= 1'b0;
        Sal <= 1'b1;
    end
    else begin
        Sau <= 1'b1;
        Sal <= 1'b0;
    end
	 
    if (y_m < count) begin
        Sbu <= 1'b0;
        Sbl <= 1'b1;
    end
    else begin
        Sbu <= 1'b1;
        Sbl <= 1'b0;
    end
    
    if (z_m < count) begin
        Scu <= 1'b0;
        Scl <= 1'b1;
    end
    else begin
        Scu <= 1'b1;
        Scl <= 1'b0;
    end
	 
	 end
endmodule

//TODO Check frequency
module counter(
  input  clk,res,
  output reg signed [31:0] out2 );

reg downup;

always @(posedge clk) begin
  if (res) begin // synchronous reset
    downup <= 1'b0;
    out2   <= 32'sh00000000;
  end
  else begin // synchronous logic
    if (out2 <= 32'sb1111_0000_0000_0000_0000_0000_0000_0000) begin
      downup <= 1'b0; // up
      out2 <= 32'sb1111_0000_0000_0000_0000_0000_0000_0001;
    end
    else if (out2 >=32'sb0001_0000_0000_0000_0000_0000_0000_0000) begin
      downup <= 1'b1; // down
      out2 <= 32'sb0000_1111_1111_1111_1111_1111_1111_1111;
    end
    else if (downup) begin // down
      out2 <= out2 - 32'sb0000_0000_0000_0000_1101_0001_1011_0111;
    end
    else begin // up
      out2 <= out2 + 32'sb0000_0000_0000_0000_1101_0001_1011_0111;
    end
  end
end
endmodule

