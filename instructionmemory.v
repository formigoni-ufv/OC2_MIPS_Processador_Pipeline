module instructionmemory (
	input wire[31:0] addr,
	output reg[31:0] instruction
	);

	reg[31:0] memory[0:63];

	///////////MEMORIA DE INSTRUCOES//////////////////////
	initial begin
		//Teste de Forwarding
		memory[0]  <= 32'b00000001000010011000000000100000;			//add $s0 $t0 $t1 // 8 + 4 = 12
		memory[4]  <= 32'b00000010000010101000100000100010;			//sub $s1 $s0 $t2 // 12 - 10 = 2
		memory[8]  <= 32'b00000010000100011001000000100000;			//add $s2 $s0 $s1 // 12 + 2 = 14
		memory[12] <= 32'b00000010010100011001100000100000;			//add $s3 $s2 $s1 // 14 + 2 = 16

		//Teste de Stall
		// memory[0] <= 32'b10101100000010110000000000000100;			//sw $t3 4($0) //Guarda 132 na memoria
		// memory[4] <= 32'b10001100000100000000000000000100;			//lw $s0 4($0)	 //Carrega 132 da memoria
		// memory[8] <= 32'b00000010000010011000100000100000;			//add $s1 $s0 $t1 // 132 + 4 = 136
	end

	always @ (addr) begin
		instruction[31:0] = memory[addr];
	end
endmodule
