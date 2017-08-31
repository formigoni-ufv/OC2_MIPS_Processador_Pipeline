module programcounter(
	input wire clock,
	input wire PCWrite,
	input wire[31:0] in,
	input wire reset,
	output reg[31:0] out
);

	always@(clock, PCWrite)begin
		if(PCWrite == 1)begin
			if(reset == 0)begin
				out <= in;
			end else begin
				out <= 32'b0;
			end
		end
	end

endmodule
