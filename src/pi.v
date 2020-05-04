module pi #(parameter Ki=32'sb0000_0000_0000_0000_0001_0100_0000_1010, Kp=32'sb0000_0000_1000_0100_1010_0010_0011_0011,
Vmin = 32'sb1111_0110_0000_0000_0000_0000_0000_0000, Vmax = 32'sb0000_1010_0000_0000_0000_0000_0000_0000) 
(output reg signed [31:0] vc, input signed [31:0] epi, input rst, input clk);

wire signed [31:0]  i, p;
reg signed [31:0] vint, vcont;

qmult u2 (Ki,epi,i);
qmult u3 (Kp,epi,p);

always@(posedge clk)
	if (rst == 1)begin
		vint = 32'sd0;
        vc = 32'sd0;
	end
	
	else begin
	//epi = xref - xf;
    
    vint = vint + i;

    vcont = vint + p;
    
    if(vcont>=Vmax) begin 
        vc = Vmax;
        vint = Vmax;
        end
    else if (vcont <= Vmin) begin
        vc = Vmin;
        vint = Vmin;
        end
    else
        vc = vcont;
	end   
endmodule

//Discrete time integration methods
//https://in.mathworks.com/help/simulink/slref/discretetimeintegrator.html#f6-529389