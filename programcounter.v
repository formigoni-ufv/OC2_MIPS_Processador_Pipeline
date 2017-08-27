module programcounter(
	input wire clock,
	input wire[31:0] in,
	input wire reset,
	output reg[31:0] out
);

	reg[31:0] programCounter;

	always@(posedge clock)begin
		if(reset == 0)begin
			programCounter <= in;
		end else begin
			programCounter <= 32'b0;
		end
	end

	always@(negedge clock)begin
		out <= programCounter;
	end

endmodule
