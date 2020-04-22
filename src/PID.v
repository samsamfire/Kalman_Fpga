//Created by Samuel Lee


//Fixed Point 32-bit PID
//Parameters Kp & Ki are 32 bit constants and instanciated with the module

module combinational_pid  #(
	parameter [31:0] Kp = 1,
	parameter [31:0] Ki = 0,
	parameter N=32,
	parameter Q = 18)
( 
 	input signed [N-1:0] data_in,
 	input reset,clk,
 	output signed [N-1:0] data_out,
 	output of
);

//Variable declarations 
reg signed [N-1:0] prev_value,output_value,temp_value;
reg signed [N-1:0] prev_in, prev_KIout;


wire signed [N-1:0] out_adderKP,out_adderKI;
wire signed [2*N-1:0] out_gainKP, out_gainKI;
wire of_Kp,of_Ki; //overflows

//Module instanciations,
//Probably possible to do better because parameters are constants ?
qmult #(Q,N) mult0(data_in,Kp,out_gainKP,of_Kp);
qmult #(Q,N) mult1(out_adderKP,Ki,out_gainKI,of_Ki);

always @(posedge clk, negedge reset) begin

	if(!reset) begin
		prev_value <= 0;
		prev_KIout <= 0;
		end
	
	else begin
		prev_in <= data_in;
		prev_KIout <= out_adderKI;
		end
		
	end
	

assign out_adderKP = prev_in + data_in;
assign out_adderKI = out_gainKI[N-1+Q:Q] + prev_KIout;


assign data_out = out_gainKP[N-1+Q:Q] + out_adderKI;

//TODO : Overflow control ? and also implement saturations

assign of = of_Kp || of_Ki;

endmodule





module sequential_pid #(
	parameter [31:0] Kp = 1,
	parameter [31:0] Ki = 0,
	parameter N=32,
	parameter Q = 18)
( 
 	input signed [N-1:0] data_in,
 	input reset,clk,
 	output signed [N-1:0] data_out,
 	output of
);


parameter s0 = 3'b000;
parameter s1 = 3'b001;
parameter s2 = 3'b010;
parameter s3 = 3'b011;
parameter s4 = 3'b100;
parameter s5 = 3'b101;
parameter s6 = 3'b110;
parameter s7 = 3'b111;

reg [2:0] state,next_state;
reg signed [N-1:0] in_adda,in_addb,in_multa,in_multb;
wire signed [N-1:0] out_add,out_mult;
wire signed [2*N-1:0] out_mult_fp;
wire overf;

reg signed [N-1:0] temp_add,in_prev,out_KIprev, result;

reg signed [2*N-1:0] temp_mult,KP_out;

//RAM parameters


	reg	[7:0]  address;
	reg	[31:0]  data_ram_in;
	reg	  wren;
	wire	[31:0]  data_ram_out;


qadd #(18,32) add0(in_adda,in_addb,out_add);
qmult #(18,32) mult0(in_multa,in_multb,out_mult,overf);
//ramlpm(.address(address),.clock(clk),.data(data_ram_in),.q(data_ram_out),.wren(wren));


//add32 add0(in_adda,in_addb,out_add);
//multiplier32 mult0(in_multa,in_multb,out_mult);


	
	
always @(posedge clk, negedge reset) begin

		if(!reset) begin
				state <= s0;
				
				in_prev <= 0;
				out_KIprev <=0;
				
				end
				
			else begin
				out_KIprev <=temp_add;
				in_prev <=data_in;
				case(state) 
					s0: state <= s1;
					s1: state <= s2;
					s2: state <= s3;
					s3: state <= s0;
				endcase
			end
	end
	
	

always @(posedge clk) begin
	
	case(state)
		s0:begin
			temp_add <= sum(in_prev,data_in);
			KP_out <= mult(data_in,Kp);
		end
		
		s1:begin
			temp_mult <= mult(Ki,temp_add);
		end
		
		s2:begin
			temp_add <= sum(temp_mult[N-1+Q:Q],out_KIprev);
			
		end
		
		s3:begin
			
			result <= sum(KP_out[N-1+Q:Q],temp_add);
		end
		
		default :
			state <= s0;
	endcase
		
		
		
		
end
	

	function [N-1:0] mult;
		input [N-1:0] multa,multb;
		begin
			in_multa = multa;
			in_multb = multb;
			mult = out_mult;
			
		end
	endfunction
	
	function [N-1:0] sum;
		input [N-1:0] adda,addb;
		begin
			in_adda = adda;
			in_addb = addb;
			sum = out_add;
		end
	endfunction
	

	
	
	assign data_out = result;
	


	
	
endmodule

