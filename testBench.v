`include "processor.v"

module testBench();
	reg clk;

	initial begin
		$display("Stage 1: IF|ID IN   :         PC + 4       | %b", );
		$display("Stage 1: IF|ID IN   :       Instruction    |\n");
		$display("Stage 2: IF|ID OUT  :         PC + 4       |");
		$display("Stage 2: IF|ID OUT  :       Instruction    |");
		$display("Stage 2: ID|EX IN   :           RS         |");
		$display("Stage 2: ID|EX IN   :           RT         |");
		$display("Stage 2:            :        Branch Alu    |\n");
		$display("Stage 3: ID|EX OUT  :           RS         |");
		$display("Stage 3: ID|EX OUT  :           RT         |");
		$display("Stage 3: MEM|WB IN  :          ALU         |");
		$display("Stage 3: MEM|WB IN  :           RT         |\n");
		$display("Stage 4: MEM|WB OUT :          ALU         |");
		$display("Stage 4: MEM|WB OUT :           RT         |");
		$display("Stage 4: EX|MEM IN  :        MEM DATA      |");
		$display("Stage 4: EX|MEM IN  :          ALU         |\n");
		$display("Stage 5: WB MUX OUT :          WB          |");
	end
endmodule
