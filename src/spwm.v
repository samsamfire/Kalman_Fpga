//`timescale 1ns/1ns

//32'b0000_1100_1100_1100_1100_1100_1100_1101 = 0.8

/*
>> Counter will generate a 5kHz traingel wave for a 10ns clock period


module spwm #(parameter signed [31:0] m = 32'b0001_0000_0000_0000_0000_0000_0000_0000)
    (output reg Sau, output reg Sal, output reg Sbu, output reg Sbl, output reg Scu, output reg Scl, output signed [31:0] out,
    input signed [31:0] x, input signed [31:0] y, input signed [31:0] z, input clk, input res);
    
    wire signed [31:0] count;
    
    counter uut(clk,res, count);
    
    assign out = count;
  
	 wire signed [31:0] x_m, y_m, z_m;
	 
    qmult u1 (x, m, x_m);
    qmult u2 (y, m, y_m);
    qmult u3 (z, m, z_m);
    
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



module qmult #(
	//Parameterized values
	parameter Q = 28,
	parameter N = 32
	)
	(
	 input			[N-1:0]	i_multiplicand,
	 input			[N-1:0]	i_multiplier,
	 output			[N-1:0]	o_result
	 );
	 
	reg ovr;
  reg [N-1:0] multiplicand, multiplier, RetVal;
  
	 //	The underlying assumption, here, is that both fixed-point values are of the same length (N,Q)
	 //		Because of this, the results will be of length N+N = 2N bits....
	 //		This also simplifies the hand-back of results, as the binimal point 
	 //		will always be in the same location...
	
	reg [2*N-1:0]	r_result;		//	Multiplication by 2 values of N bits requires a 
											//		register that is N+N = 2N deep...
	reg [N-1:0]		r_RetVal;
	
//--------------------------------------------------------------------------------
	assign o_result = r_RetVal;	//	Only handing back the same number of bits as we received...
											//		with fixed point in same location...
	
//---------------------------------------------------------------------------------
	always @(i_multiplicand, i_multiplier)	begin						//	Do the multiply any time the inputs change
      
      ///////////////////////////////////////////////
      if (i_multiplicand[N-1] == 1) begin
        multiplicand = ~i_multiplicand + 1;
        multiplicand[N-1] = 1;
      end
      else
        multiplicand = i_multiplicand;
      
      if (i_multiplier[N-1] == 1) begin
        multiplier = ~i_multiplier + 1;
        multiplier[N-1] = 1;
      end
      else
        multiplier = i_multiplier;
      ////////////////////////////////////////////////
      	 
      r_result = multiplicand[N-2:0] * multiplier[N-2:0];	//	Removing the sign bits from the multiply - that 
																					//		would introduce *big* errors	
		ovr = 1'b0;															//	reset overflow flag to zero
		end
	
		//	This always block will throw a warning, as it uses a & b, but only acts on changes in result...
	always @(r_result) begin													//	Any time the result changes, we need to recompute the sign bit,
		RetVal[N-1] = multiplicand[N-1] ^ multiplier[N-1];	//		which is the XOR of the input sign bits...  (you do the truth table...)
		RetVal[N-2:0] = r_result[N-2+Q:Q];								//	And we also need to push the proper N bits of result up to 
																						//		the calling entity...
     
      if (RetVal[N-1] == 1) begin
        RetVal[N-1] = 0;
        r_RetVal = ~RetVal +1;
      end
      else
        r_RetVal = RetVal;
      
      if (r_result[2*N-2:N-1+Q] > 0)										// And finally, we need to check for an overflow
			ovr <= 1'b1;
		end

endmodule